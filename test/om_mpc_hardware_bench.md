# MPC hardware ground benchmark

Build the explicit test firmware (inherits the current FMUv5 flight configuration):

```sh
source ~/px4_arm_env.sh
make px4_fmu-v5_mpcbench
```

The normal `make px4_fmu-v5` build excludes all benchmark entry points, fields,
branches and implementation. No flight parameters, controller gates or solver
algorithms are changed by running the test. Do not flash firmware as part of an
automated test without operator authorization.

After installing the test firmware, keep the vehicle disarmed and disconnect
motor/servo power. GPS, local position, arming and SIH are not needed. The normal
`mc_pos_control` module must be running; start it if the terminal reports otherwise.

The test reads current `MPC_N`, `MPC_DT`, Q/R, terminal scale, iteration limit,
thrust parameter and rate limits once. It does not change them. For N20/dt=.05,
set those parameters to the desired values before testing.

Run from the PX4/QGC terminal, waiting for the first test to complete:

```sh
mc_pos_control mpc_bench 16000 10
mc_pos_control mpc_bench 8000 10
```

Arguments: period in microseconds (2000..1000000), total run time in seconds
(1..10, default 10). These examples impose 62.5 and 125 Hz independently of
estimator publication, corresponding to the desired hardware 50/100 modes when
local position publishes at 125 Hz. Actual flight mode cadence can differ.

```sh
mc_pos_control mpc_bench status
mc_pos_control mpc_bench stop
```

The same private work queue and single QP workspace are used exclusively during
the benchmark. Synthetic figure-eight geometry and bounded position/velocity/
attitude errors feed the existing model builder and structured qpDUNES backend.
No synthetic sensor/reference messages or active control commands are published.
The test rejects armed or stale vehicle status, and aborts on either condition
during execution. It never changes arming state. Normal controller execution
resumes after cleanup. The benchmark does not prevent a user from attempting to
arm, so keep the vehicle disarmed for the whole test.

Cold setup is reported separately; warm statistics report achieved rate,
build/update/solve wall time, per-call thread CPU time, core CPU contribution,
failures, deadline misses, skipped releases and QP arena/problem memory. Missed
releases are skipped rather than executed in bursts. The first cold setup is
included in the requested total duration, not added to it. A final in-progress
bounded solve/cleanup may finish slightly after that duration.

System CPU is sampled from the existing `cpuload` topic, including a pre-test
sample and the measured test mean/peak. Pre-test CPU is a snapshot, not a long
baseline average; repeat the two tests in reverse order for comparison. Input
generation, terminal messages and test bookkeeping also contribute to total
system CPU; they are excluded from the timed model/update/solve region. No
per-solve terminal output or additional runtime diagnostics are added to flight.

This measures synthetic MPC load under real ground background tasks; it does
not reproduce all armed-flight task loads or worst-case constrained trajectories.
Use the measured core CPU increment with an existing real-flight CPU baseline.
