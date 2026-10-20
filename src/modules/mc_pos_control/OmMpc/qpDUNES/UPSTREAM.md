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
