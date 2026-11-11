# O3CPU host-performance benchmark

This directory contains a deterministic benchmark for measuring the host
throughput of gem5's X86 O3CPU model. It is intended for local performance
engineering and does not impose a wall-clock threshold on correctness CI.

The target contains six small workloads:

- `arithmetic`: independent integer arithmetic with a small working set;
- `branch`: deterministic irregular branches;
- `stream`: sequential loads and stores;
- `pointer`: dependent, shuffled loads;
- `mixed`: arithmetic, branches, loads, and stores; and
- `queue`: independent integer operations plus a configurable dependency
  chain.

The gem5 configuration supplies small, medium, and large O3CPU parameter
sets. These are synthetic scaling points and do not claim to model any
commercial processor.

## Build

Build the target with a fixed compiler and flags. For example:

```sh
mkdir -p build/o3-perf
gcc -O2 -g -static -fno-pie -no-pie \
    -fno-tree-vectorize -fno-unroll-loops \
    -Wall -Wextra -Werror \
    -o build/o3-perf/o3_microbench.x86 \
    util/o3_perf/o3_microbench.c
scons build/X86/gem5.opt -j "$(nproc)"
```

Record the compiler version, target hash, gem5 hash, and gem5 build flags
with the results. The harness records both binary hashes automatically.

## Run

A plain single-binary run is useful for calibration:

```sh
python3 util/o3_perf/run_benchmarks.py \
    --gem5 baseline=build/X86/gem5.opt \
    --config-script util/o3_perf/o3_se.py \
    --target build/o3-perf/o3_microbench.x86 \
    --output-root build/o3-perf/baseline \
    --baseline-commit "$(git rev-parse HEAD)" \
    --build-type gem5.opt \
    --build-flags "record exact SCons and compiler flags here"
```

For a comparison, give the harness both binaries:

```sh
python3 util/o3_perf/run_benchmarks.py \
    --gem5 baseline=/path/to/gem5-baseline.opt \
    --gem5 candidate=/path/to/gem5-candidate.opt \
    --config-script util/o3_perf/o3_se.py \
    --target build/o3-perf/o3_microbench.x86 \
    --output-root build/o3-perf/comparison-session-1 \
    --baseline-commit BASELINE_COMMIT \
    --build-type gem5.opt \
    --build-flags "identical flags used for both binaries" \
    --session session-1 \
    --runs 8
```

The two-binary order alternates between `A B B A` and `B A A B` blocks.
Use a quiet host on AC power, pin the process to an otherwise idle core,
and collect at least two sessions. Do not combine results from binaries
built with different compilers, flags, libraries, or source baselines.

`raw_runs.csv` contains per-run host timing, peak RSS, simulated
statistics, exit information, and hashes. `summary.csv` contains the
median, quartiles, range, coefficient of variation, and simulated
instructions per host second.

The optional `--counter-mode perf` records host hardware counters through
`perf stat`. Use it in a separate profiler-overhead run rather than for
ordinary wall-clock results. Unsupported counters are recorded as such;
they must not be inferred from timing or simulated statistics.

## Correctness comparison

Compare the complete `stats.txt` files from corresponding baseline and
candidate runs:

```sh
python3 util/o3_perf/compare_stats.py \
    /path/to/baseline/stats.txt \
    /path/to/candidate/stats.txt \
    --output statistics_comparison.json
```

The comparison requires exact equality for simulated statistics and
reports any missing statistic. Statistics whose names start with `host`
are reported separately because host timing and rate values are expected
to change between runs.
