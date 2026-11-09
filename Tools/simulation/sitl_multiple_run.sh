#!/usr/bin/env bash
# Run multiple instances of the 'px4' binary, without starting an external simulator.
# It assumes px4 is already built with the specified build target.
#
# Usage: ./Tools/simulation/sitl_multiple_run.sh [num_instances] [model] [build_target]
# Examples:
#   ./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
#   ./Tools/simulation/sitl_multiple_run.sh 2 gazebo-classic_iris px4_sitl_default
#   ./Tools/simulation/sitl_multiple_run.sh     # defaults: 2 instances, gazebo-classic_iris, px4_sitl_default

sitl_num=${1:-2}
sim_model=${2:-gazebo-classic_iris}
build_target=${3:-px4_sitl_default}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../"

build_path=${src_path}/build/${build_target}

if [ ! -x "$build_path/bin/px4" ]; then
	echo "PX4 binary not found: $build_path/bin/px4" >&2
	exit 1
fi

# Starting another batch must never stop an unrelated flight. Check the same
# per-instance POSIX locks as PX4 before opening/truncating any instance logs.
if ! python3 - "$sitl_num" <<'PY'
import fcntl
import os
import sys

try:
    count = int(sys.argv[1])
    if not 1 <= count <= 255:
        raise ValueError
except ValueError:
    sys.exit("Instance count must be an integer from 1 to 255")

for instance in range(count):
    try:
        fd = os.open(f"/tmp/px4_lock-{instance}", os.O_RDWR)
    except FileNotFoundError:
        continue
    try:
        fcntl.lockf(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit(f"PX4 instance {instance} is already running; no instances were started or stopped")
    finally:
        os.close(fd)
PY
then
	exit 1
fi

export PX4_SIM_MODEL=${sim_model}

n=0
while [ $n -lt $sitl_num ]; do
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $n in $(pwd)"
	$build_path/bin/px4 -i $n -d "$build_path/etc" >out.log 2>err.log &
	popd &>/dev/null

	n=$(($n + 1))
done
