Source: https://github.com/jfrasch/qpDUNES
Commit: 665cbaac32be1a2477a991722ad99b2bb84d0631
Matches geometric_controller/scripts/install_ommpc_solvers.sh.
Only the simple-bounds C solver and public headers are included, unchanged.
Original LGPL-2.1-or-later notices are preserved in each source/header.

PX4 integration: only these upstream C sources/public headers are vendored.
`specialize_float.py` generates build-directory sources with float literals/math,
fixed nx/nu, and zero-aware diagonal-cost kernels. `float_prefix.h` keeps their
symbols private. `../CMakeLists.txt` adds an iteration-boundary deadline hook.
No upstream file is rewritten in place. Original LGPL notices remain intact.

`private_compat.py` adapts only the generated backend and its private header
copies for PX4: C prototypes, bounded signed loop dimensions, correct enum/bool
returns and explicit rejection of two unfinished upstream line searches. It
removes disabled diagnostic argument work and abandoned Cholesky diagnostics.
The float32 zero threshold is fixed to FLT_MIN; division guards reject zero and
subnormal divisors (including FPU flush-to-zero) and nonfinite operands, even
when the threshold product underflows. One divisor classification plus the
ratio check avoids repeating finite checks inside the matrix kernels.
Structural zeros use an exact `fabsf(x) <= 0.f` comparison, matching the
zero-tolerance comparison in PX4's matrix helpers, without fpclassify libc calls.
Original public headers and their struct layouts remain unchanged.

Native memory/undefined-behavior regression: `python3 test/run_om_mpc_solver_test.py`
from the repository root (requires C/C++ compilers with ASan/UBSan support).
