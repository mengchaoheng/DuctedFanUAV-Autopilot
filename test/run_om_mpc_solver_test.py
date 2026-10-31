#!/usr/bin/env python3
"""Build the actual private C backend and wrapper with ASan/UBSan (native host)."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OM = ROOT / 'src/modules/mc_pos_control/OmMpc'
QP = OM / 'qpDUNES'


def run(args):
    subprocess.run([str(a) for a in args], check=True)


with tempfile.TemporaryDirectory(prefix='om_solver_test_') as work:
    out = Path(work)
    run([sys.executable, QP / 'private_compat.py', QP / 'include', out / 'include'])
    flags = ['-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
             '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
             '-D__USE_SINGLE_PRECISION__', '-D__SIMPLE_BOUNDS_ONLY__',
             '-D__SUPPRESS_ALL_OUTPUT__', '-D__SUPPRESS_ALL_WARNINGS__',
             '-I' + str(OM), '-I' + str(QP), '-I' + str(out / 'include'), '-I' + str(out / 'include/qp'),
             '-include', str(QP / 'float_prefix.h')]
    cc, cxx = os.environ.get('CC', 'cc'), os.environ.get('CXX', 'c++')
    objects = []
    for name in ['dual_qp', 'matrix_vector', 'setup_qp', 'qpdunes_utils', 'stage_qp_solver_clipping']:
        source = out / (name + '.c')
        run([sys.executable, QP / 'specialize_float.py', QP / (name + '.c'), source])
        if name == 'dual_qp':
            anchor = 'for ((*itCntr) = 1; (*itCntr) <= qpData->options.maxIter; ++(*itCntr)) {'
            text = source.read_text()
            assert text.count(anchor) == 1
            source.write_text('extern int om_float_om_qp_deadline_expired(void);\n' + text.replace(
                anchor, anchor + '\nif (om_float_om_qp_deadline_expired()) return QPDUNES_ERR_ITERATION_LIMIT_REACHED;'))
        obj = source.with_suffix('.o')
        run([cc, '-std=c11', *flags, '-Dcalloc=om_float_om_qp_calloc',
             '-Dfree=om_float_om_qp_free', '-c', source, '-o', obj])
        objects.append(obj)
    arena = out / 'arena.o'
    run([cc, '-std=c11', *flags, '-c', OM / 'OmQpArena.c', '-o', arena])
    binary = out / 'solver_test'
    run([cxx, '-std=c++17', *flags, ROOT / 'test/om_mpc_solver_test.cpp',
         OM / 'OmQpSolver.cpp', arena, *objects, '-o', binary])
    run([binary])
