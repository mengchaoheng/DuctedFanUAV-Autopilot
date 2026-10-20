#!/usr/bin/env python3
"""Generate a private float32 nx=9/nu=4 qpDUNES backend; upstream remains intact."""
import pathlib,re,sys
text=pathlib.Path(sys.argv[1]).read_text()
for name in ('sqrt','fabs','fmin','fmax'):
 text=re.sub(r'(?<![A-Za-z0-9_])'+name+r'\s*\(',name+'f(',text)
# Prevent silent double promotion in mixed arithmetic on Cortex-M7. Keep
# comments/strings and integer constants intact, equivalent to GCC's
# -fsingle-precision-constant on the hardware backend.
pattern=re.compile(r'(?P<keep>/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*")|(?P<number>(?<![A-Za-z0-9_])(?:[0-9]+\.[0-9]*|\.[0-9]+|[0-9]+[eE][+-]?[0-9]+)(?:[eE][+-]?[0-9]+)?)(?P<suffix>[fFlL]?)(?![A-Za-z0-9_])',re.S)
def literal(match):
 if match.group('keep') is not None:return match.group()
 return match.group('number')+(match.group('suffix') or 'f')
text=pattern.sub(literal,text)
includes=list(re.finditer(r'^\s*#include[^\n]*',text,re.M))
if not includes:raise SystemExit('missing include anchor')
pos=includes[-1].end()
quiet='' if pathlib.Path(sys.argv[1]).name=='qpdunes_utils.c' else '\n#undef qpDUNES_printf\n#undef qpDUNES_printf_noNewLine\n#define qpDUNES_printf(...) ((void)0)\n#define qpDUNES_printf_noNewLine(...) ((void)0)\n'
text=text[:pos]+quiet+'\n#undef _NX_\n#undef _NU_\n#undef _NZ_\n#define _NX_ 9\n#define _NU_ 4\n#define _NZ_ 13\n'+text[pos:]
# the model's C=[A B] has many exact structural zeros. Skip whole zero rows of
# each diagonal-cost outer product; keep the order of all nonzero sums.
if pathlib.Path(sys.argv[1]).name == 'matrix_vector.c':
 # Algebraic expansion of C diag(invH) C^T for the on-manifold block footprint.
 # Accumulate columns in the original order; active input/state columns
 # remain excluded using the same dual activeness test. No model freezing.
 footprints=[[0,3],[1,4],[2,5],[3,6,7,9],[4,6,7,9],[5,6,7,9]]+[[6,7,8,10,11,12]]*3
 expanded='\n    if (om_float_om_structured_c && cholM1->sparsityType == QPDUNES_DIAGONAL && dim0 == 9 && dim1 == 13) {\n        qpDUNES_makeMatrixDense(res,9,9);\n'
 for col in range(13):
  rows=[r for r in range(9) if col in footprints[r]]
  expanded+=f'        if (y[{2*col}] <= qpData->options.equalityTolerance && y[{2*col+1}] <= qpData->options.equalityTolerance) {{\n'
  for row in rows:
   expanded+=f'            const real_t v{row}=M2->data[{row*13+col}];\n'
   expanded+=f'            if (!(fabsf(cholM1->data[{col}]) >= qpData->options.QPDUNES_ZERO*fabsf(v{row}))) return QPDUNES_ERR_DIVISION_BY_ZERO;\n'
   expanded+=f'            const real_t s{row}=v{row}/cholM1->data[{col}];\n'
  for i,row in enumerate(rows):
   for upper in rows[:i+1]: expanded+=f'            res->data[{row*9+upper}]+=v{row}*s{upper};\n'
  expanded+='        }\n'
 expanded+='        for (int i=0;i<9;i++) for (int j=i+1;j<9;j++) res->data[i*9+j]=res->data[j*9+i];\n        return QPDUNES_OK;\n    }\n'
 at=text.index('{',text.index('return_t addMultiplyMatrixInvMatrixMatrixT('))+1
 text=text[:at]+expanded+text[at:]
 text='extern int om_float_om_structured_c;\n'+text
 # Fixed-size on-manifold diagonal-H kernel: enumerate exact nonzeros in each C
 # column once, divide only those entries, then accumulate their outer product.
 # No tolerance-based pruning; preserve column/nonzero accumulation order.
 function='return_t addMultiplyMatrixInvMatrixMatrixT('
 begin=text.index('{',text.index(function))+1
 kernel=r"""
    if (!om_float_om_structured_c && cholM1->sparsityType == QPDUNES_DIAGONAL && dim0 == 9 && dim1 == 13) {
        int rows[9];
        real_t values[9], scaled[9];
        qpDUNES_makeMatrixDense(res, 9, 9);
        for (int col = 0; col < 13; ++col) {
            if (!(y[2*col] <= qpData->options.equalityTolerance &&
                  y[2*col+1] <= qpData->options.equalityTolerance)) { continue; }
            int count = 0;
            for (int row = 0; row < 9; ++row) {
                const real_t value = M2->data[row*13+col];
                if (value == 0.f) { continue; }
                if (!(fabsf(cholM1->data[col]) >= qpData->options.QPDUNES_ZERO * fabsf(value))) {
                    return QPDUNES_ERR_DIVISION_BY_ZERO;
                }
                rows[count] = row;
                values[count] = value;
                scaled[count] = value / cholM1->data[col];
                ++count;
            }
            for (int i = 0; i < count; ++i) {
                real_t *destination = res->data + rows[i]*9;
                for (int j = 0; j <= i; ++j) {
                    destination[rows[j]] += values[i] * scaled[j];
                }
            }
        }
        // C inv(H) C' is symmetric. Reverse Cholesky consumes the lower
        // triangle, whose arithmetic is unchanged; avoid calculating the
        // redundant upper triangle and mirror the completed lower entries.
        for (int i = 0; i < 9; ++i) {
            for (int j = i+1; j < 9; ++j) {
                res->data[i*9+j] = res->data[j*9+i];
            }
        }
        return QPDUNES_OK;
    }
"""
 text=text[:begin]+kernel+text[begin:]
 anchor='res->data[ii * dim0 + jj] += M2->data[ii * dim1 + ll] * Ztmp->data[ll * dim0 + jj];'
 if text.count(anchor)!=1:raise SystemExit('missing diagonal outer-product anchor')
 start=text.rfind('for (ii = 0; ii < dim0; ++ii) {',0,text.index(anchor))
 if start<0:raise SystemExit('missing row loop anchor')
 end=start+len('for (ii = 0; ii < dim0; ++ii) {')
 text=text[:end]+'\n                    if (M2->data[ii * dim1 + ll] == 0.f) { continue; }'+text[end:]
# C-vector products occur in local gradients, Newton gradients and line search.
# The same verified OM-MPC footprint can skip structural zero products here too.
# Preserve each output's original accumulation order. Nonfinite operands or
# overlapping buffers take the unchanged dense path (0*Inf must still be NaN).
if pathlib.Path(sys.argv[1]).name == 'matrix_vector.c':
 footprint=[]
 for row in range(9):
  footprint.append([row,row+3] if row<3 else [row,6,7,9] if row<6 else [6,7,8,10,11,12])
 for name,transposed in [('multiplyMatrixVectorDense',False),('multiplyMatrixTVectorDense',True)]:
  declaration=re.search(r'^return_t '+name+r'\s*\(',text,re.M)
  if declaration is None:raise SystemExit('missing C-vector declaration: '+name)
  at=text.index('{',declaration.start())+1
  inputs,outputs=(9,13) if transposed else (13,9)
  kernel=f'\n    if (om_float_om_structured_c && dim0 == 9 && dim1 == 13) {{\n'
  kernel+=f'        const uintptr_t dst=(uintptr_t)res, vec=(uintptr_t)x, mat=(uintptr_t)M;\n'
  kernel+=f'        int safe=(dst+{outputs}*sizeof(real_t)<=vec || vec+{inputs}*sizeof(real_t)<=dst) && (dst+{outputs}*sizeof(real_t)<=mat || mat+117*sizeof(real_t)<=dst);\n'
  kernel+=f'        for (int i=0;i<{inputs};++i) safe &= isfinite(x[i]);\n'
  kernel+='        if (safe) {\n'
  for output in range(outputs):
   kernel+=f'            res[{output}]=0.f;\n'
   terms=[(row,output,row) for row in range(9) if output in footprint[row]] if transposed else [(output,col,col) for col in footprint[output]]
   for row,col,index in terms:kernel+=f'            res[{output}]+=M[{row*13+col}]*x[{index}];\n'
  kernel+='            return QPDUNES_OK;\n        }\n    }\n'
  text=text[:at]+kernel+text[at:]
 text='#include <stdint.h>\n'+text
if pathlib.Path(sys.argv[1]).name == 'stage_qp_solver_clipping.c':
 # Merge the diagonal box-QP step's vector passes without changing its
 # arithmetic or bound tests. Keep the generic identity/dense path intact.
 declaration=re.search(r'^return_t directQpSolver_doStep\(',text,re.M)
 if declaration is None: raise SystemExit('missing clipping step declaration')
 at=text.index('{',declaration.start())+1
 text=text[:at]+r"""
    if (interval->H.sparsityType == QPDUNES_DIAGONAL) {
        for (int i = 0; i < interval->nV; ++i) {
            const real_t trial = interval->qpSolverClipping.zUnconstrained.data[i] + alpha * stepDir->data[i];
            zUnconstrained->data[i] = trial;
            const real_t low = (interval->zLow.data[i] - trial) * interval->H.data[i];
            const real_t high = (trial - interval->zUpp.data[i]) * interval->H.data[i];
            mu->data[2*i] = low;
            mu->data[2*i+1] = high;
            z->data[i] = low >= -qpData->options.activenessTolerance ? interval->zLow.data[i] :
                (high >= -qpData->options.activenessTolerance ? interval->zUpp.data[i] : trial);
            q->data[i] = interval->q.data[i] + alpha * interval->qpSolverClipping.qStep.data[i];
        }
        *p = interval->p + alpha * interval->qpSolverClipping.pStep;
        return QPDUNES_OK;
    }
"""+text[at:]
if pathlib.Path(sys.argv[1]).name == 'dual_qp.c':
 # Expand the nine in-block triangular rows so all inner bounds/offsets
 # are compile-time constants. Keep reverse block order, subtraction order,
 # factor layout, division and the optional upstream division guard unchanged.
 declaration='return_t qpDUNES_solveNewtonEquationBottomUp('
 start=text.index('{',text.index(declaration))
 end=start+1; depth=1
 while depth:
  if text[end]=='{': depth+=1
  elif text[end]=='}': depth-=1
  end+=1
 body='{\n'
 for reverse in (True,False):
  body+=('    for (int kk=_NI_-1; kk>=0; --kk) {\n' if reverse else '    for (int kk=0; kk<_NI_; ++kk) {\n')
  body+='        const real_t *block=cholHessian->data+kk*162;\n        real_t *row=res->data+kk*9;\n'
  for i in (range(8,-1,-1) if reverse else range(9)):
   body+=f'        {{ real_t sum={"gradient->data[kk*9+"+str(i)+"]" if reverse else "row["+str(i)+"]"};\n'
   if not reverse:
    body+=f'          if (kk>0) for (int j=0;j<9;j++) sum-=block[{i*18}+j]*res->data[(kk-1)*9+j];\n'
   for j in (range(i+1,9) if reverse else range(i)):
    body+=f'          sum-=block[{j*18+9+i if reverse else i*18+9+j}]*row[{j}];\n'
   if reverse:
    body+=f'          if (kk<_NI_-1) for (int j=0;j<9;j++) sum-=block[162+j*18+{i}]*res->data[(kk+1)*9+j];\n'
   body+=f'          #if defined(__USE_ASSERTS__)\n          if (fabsf(block[{i*18+9+i}]) < qpData->options.QPDUNES_ZERO*fabsf(sum)) return QPDUNES_ERR_DIVISION_BY_ZERO;\n          #endif\n          row[{i}]=sum/block[{i*18+9+i}]; }}\n'
  body+='    }\n'
 body+='    return QPDUNES_OK;\n}'
 text=text[:start]+body+text[end:]
 # Complete the upstream TODO: eliminate active columns during construction
 # rather than dividing a full dense temporary and masking it afterwards.
 # the model's A block has exact structural zeros. Preserve every nonzero division,
 # including its rounding, and preserve the old sign of inactive zero entries.
 start=text.index('statusFlag = multiplyAInvQ(')
 end=text.index('\n\t\t}\n\t}\t/* END OF sub-diagonal block',start)
 old=text[start:end]
 if 'cheap way of annihilating columns' not in old:
  raise SystemExit('missing clipping sub-diagonal anchor')
 text=text[:start]+r"""if (intervals[kk]->cholH.sparsityType == QPDUNES_DIAGONAL) {
                    for (jj = 0; jj < _NX_; ++jj) {
                        const boolean_t inactive =
                            intervals[kk]->y.data[2*jj] <= qpData->options.equalityTolerance &&
                            intervals[kk]->y.data[2*jj+1] <= qpData->options.equalityTolerance;
                        const real_t diagonal = intervals[kk]->cholH.data[jj];
                        for (ii = 0; ii < _NX_; ++ii) {
                            const real_t value = intervals[kk]->C.data[ii*_NZ_+jj];
                            accHessian(kk, -1, ii, jj) = !inactive ? 0.f :
                                (value == 0.f ? -value : -(value / diagonal));
                        }
                    }
                } else {
"""+old+r"""
                }
"""+text[end:]
# Fixed stage sizes expose the existing scalar clipping step to the compiler.
# Preserve element order, bound tests, arithmetic and the generic size fallback.
if pathlib.Path(sys.argv[1]).name == 'stage_qp_solver_clipping.c':
 declaration=re.search(r'^return_t directQpSolver_doStep\(',text,re.M)
 start=text.index('    if (interval->H.sparsityType == QPDUNES_DIAGONAL)',declaration.end())
 end=start+text[start:].index('\n    }')+len('\n    }')
 body=text[start:end]
 variants=''
 for size in (13,9):
  specialized=body.replace('interval->H.sparsityType == QPDUNES_DIAGONAL','interval->H.sparsityType == QPDUNES_DIAGONAL && interval->nV == '+str(size)).replace('i < interval->nV','i < '+str(size))
  variants+=specialized+'\n'
 text=text[:start]+variants+text[start:]
pathlib.Path(sys.argv[2]).write_text(text)
