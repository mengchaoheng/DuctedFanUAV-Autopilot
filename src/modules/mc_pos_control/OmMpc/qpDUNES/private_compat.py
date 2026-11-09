"""Checked C/PX4 adaptations for the private float32 backend, not upstream."""

from pathlib import Path
import re
import sys


def _body(text, name, replacement):
    """Replace one function body while retaining its public signature."""
    declaration = re.search(r'^\w+\s+' + name + r'\s*\(', text, re.M)
    if declaration is None:
        raise ValueError('Missing private-backend function: ' + name)
    start = text.index('{', declaration.end())
    tokens = re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]', text[start:], re.S)
    depth = 0
    for token in tokens:
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                end = start + token.end()
                return text[:start] + replacement + text[end:]
    raise ValueError('Unbalanced private-backend function: ' + name)


def declarations(text):
    text = re.sub(r'^(\w+\s+(?:getTime|qpDUNES_setupDefaultOptions))\(\s*(?:/\*.*?\*/\s*)?\)',
                  r'\1(void)', text, flags=re.M)
    return re.sub(r'\breturn_t(\s+qpDUNES_exists(?:Matrix|Vector)\s*\()', r'boolean_t\1', text)


def headers(source, destination):
    """Keep adapted declarations private and before upstream in include paths."""
    for original in Path(source).rglob('*'):
        if original.suffix not in ('.h', '.hpp'):
            continue
        target = Path(destination) / original.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        text = declarations(original.read_text())
        # Dimensions fit int_t: hardware N<=20, POSIX N<=160; nx=9, nu=4.
        for member in ('nI', 'nV', 'nD', 'nDttl'):
            text = re.sub(r'\((qpData->(?:intervals\[ I \]->)?' + member + r')\)',
                          r'((int_t)\1)', text)
        target.write_text(text)


def source(text, filename):
    text = declarations(text)
    text = text.replace('uint_t ii, kk;', 'int_t ii, kk;')
    text = text.replace('uint_t ii = 0;\n\tuint_t kk = 0;', 'int_t ii = 0;\n\tint_t kk = 0;')
    text = re.sub(r'\b(ii|kk)(\s*<\s*)nI\b', r'\1\2(int_t)nI', text)
    text = text.replace('interval->id < _NI_', '(int_t)interval->id < _NI_')
    text = re.sub(r'^\s*int kk = 1234567890;[^\n]*', '', text, flags=re.M)
    text = text.replace('"QP on interval %d infeasible!", kk',
                        '"QP on interval %d infeasible!", (int_t)interval->id')
    text = re.sub(r'([<!=]=?|>=?)\s*(interval->nV|_dim)\b', r'\1 (int_t)\2', text)
    text = re.sub(r'([<>=!]=?)\s*qpData->options.(maxNumLineSearch(?:Refinement)?Iterations)',
                  r'\1 (uint_t)qpData->options.\2', text)
    # Exact structural zeros, using the zero-tolerance fabs comparison from
    # PX4 matrix::isEqualF. Avoid C fpclassify library calls in the inner loop.
    text = re.sub(r'\bvalue == 0\.f\b', 'fabsf(value) <= 0.f', text)
    text = text.replace('M2->data[ii * dim1 + ll] == 0.f',
                        'fabsf(M2->data[ii * dim1 + ll]) <= 0.f')

    if filename == 'setup_qp.c':
        text, count = re.subn(r'(options\.QPDUNES_ZERO\s*=\s*)1\.e-50f;', r'\1FLT_MIN;', text)
        if count != 1:
            raise ValueError('Missing float32 zero-threshold anchor')
        text = '#include <float.h>\n' + text

    if filename == 'dual_qp.c':
        # Abandoned diagnostics do not contribute to the Cholesky result.
        text = re.sub(r'/\* TEMPORARY\s*\*/.*?/\* END TEMPORARY\s*\*/', '', text, flags=re.S)
        for name in ('qpDUNES_reductionLineSearchWithASChange', 'qpDUNES_goldenSectionIntervalSearch'):
            text = _body(text, name, '{\n\t/* Upstream marks this algorithm unfinished; never execute it. */\n'
                         '\treturn QPDUNES_ERR_UNKNOWN_LS_TYPE;\n}')
        text = re.sub(r'^\s*#pragma omp[^\n]*', '', text, flags=re.M)
        text = text.replace('#ifndef __QPDUNES_PARALLEL__\n\tint_t errCntr = 0;',
                            '#ifdef __QPDUNES_PARALLEL__\n\tint_t errCntr = 0;')
        text = text.replace('statusFlag =  QPDUNES_ERR_INVALID_ARGUMENT;\n\t\t\t#endif',
                            'return QPDUNES_ERR_INVALID_ARGUMENT;\n\t\t\t#endif')
        text = text.replace('statusFlag = QPDUNES_ERR_INVALID_ARGUMENT;\n\t\t#endif',
                            'return QPDUNES_ERR_INVALID_ARGUMENT;\n\t\t#endif')

    if filename == 'qpdunes_utils.c':
        text = text.replace('if ( ( nRows < 1 ) || ( nCols < 1 ) || ( M == 0 ) )\n\t\treturn QPDUNES_OK;',
                            'if ( ( nRows < 1 ) || ( nCols < 1 ) || ( M == 0 ) )\n\t\treturn QPDUNES_MATRIX_UNDEFINED;')
        text = text.replace('sparsityM = QPDUNES_DENSE;\n\t\treturn QPDUNES_OK;',
                            'return QPDUNES_DENSE;')
    else:
        # Removing the call also removes otherwise-evaluated varargs/norm work.
        at = list(re.finditer(r'^\s*#include[^\n]*', text, re.M))[-1].end()
        quiet = '\n'.join('#undef ' + name + '\n#define ' + name + '(...) ((void)0)'
                          for name in ('qpDUNES_printSuccess', 'qpDUNES_printError',
                                       'qpDUNES_printWarning', 'qpDUNES_printInfo'))
        text = text[:at] + '\n' + quiet + '\n' + text[at:]

    # Retain all meaningful ratio checks and also reject exact zero / NaN.
    # Arguments can themselves be matrix-access macros. Match their parentheses
    # explicitly rather than letting a lazy wildcard consume a closing bracket.
    argument = r'((?:[^()]|\([^()]*\))*)'
    condition = re.compile(r'if\s*\(\s*(!?)\s*\(?\s*fabsf\(' + argument + r'\)\s*(>=|<)\s*'
                           r'qpData->options.QPDUNES_ZERO(?:\s*\*\s*fabsf\(' + argument + r'\))?\s*\)?\s*\)')
    def divisor(match):
        negative, denominator, comparison, numerator = match.groups()
        invert = (comparison == '<') != bool(negative)
        return 'if (' + ('!' if invert else '') + 'om_float_divisor_valid(' + denominator + ', ' + (numerator or '1.f') + '))'
    text = condition.sub(divisor, text)
    return text


if __name__ == '__main__':
    headers(sys.argv[1], sys.argv[2])
