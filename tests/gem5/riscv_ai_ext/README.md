# RISC-V AI Extension Tests And Benchmarks

This directory now contains both:

- a small SE-mode smoke/regression suite for the custom RISC-V instructions
- a local performance benchmark harness for comparing `gem5_baseline` against
  the updated `gem5` tree on `o3`

## Smoke Coverage

- `ai_ops_smoke`
  - `mac`, `dot4_acc`, `relu`, `clamp`
  - `x0` source/destination behavior
- `p_lw_smoke`
  - load result
  - post-increment of `rs1`
  - `rd = x0` write suppression with pointer update preserved
- `hwloop_smoke`
  - `lp.setup` loop redirect behavior
  - `count = x0` / zero-count path
  - minimum valid immediate (`imm = 4`)
  - intended to be run on `atomic`, `minor`, and `o3`

## Build The Smoke Binaries

```bash
python3 tests/gem5/riscv_ai_ext/build_binaries.py
```

## Run The Smoke Suite

From `tests/`:

```bash
./main.py run gem5/riscv_ai_ext --length=quick --skip-build
```

Use `--skip-build` when `gem5.opt` is already up to date; omit it when you need
the test driver to rebuild gem5 first.

## Performance Benchmarks

Two standalone benchmark families are provided in `perf_src/`:

- `dot4_pipeline`
  - scalar reference: standard packed-byte dot-product accumulation in C++
  - custom optimized: `lp.setup + p.lw + dot4_acc + relu + clamp`
  - this is the main "full custom pipeline" benchmark
- `mac_pipeline`
  - scalar reference: large signed multiply-accumulate recurrence in C++
  - custom optimized: large signed recurrence accelerated with `mac`, then
    finalized with `relu + clamp`
  - this benchmark is intentionally arithmetic-heavy so the `mac` instruction
    itself is the main signal, instead of hardware-loop redirect cost

## Build The Performance Binaries

```bash
python3 tests/gem5/riscv_ai_ext/build_perf_binaries.py
```

The local build scripts look for one of these compilers:

- `riscv64-linux-gnu-gcc`
- `riscv64-unknown-linux-gnu-gcc`
- `riscv64-unknown-elf-gcc`

You can override the prefix if needed:

```bash
CROSS_COMPILE=riscv64-linux-gnu- python3 tests/gem5/riscv_ai_ext/build_perf_binaries.py
```

## Run The Performance Comparison

Default usage:

```bash
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py
```

Useful variants:

```bash
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --skip-build
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --out-dir /tmp/riscv_ai_perf
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --baseline-repo /home/duydonv/gem5_baseline --update-repo /home/duydonv/gem5
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --no-cache
```

The last command forwards `--no-cache` to `perf_binary_run.py` (DRAM-only, for
apples-to-apples with old NoCache results).

For **O3**, `configs/perf_binary_run.py` enables the **decoupled front-end** (FDP) by
default so the BTB-driven fetch path and commit-time hardware-loop BTB priming
apply. The script also switches the branch predictor to **LTAGE** (stdlib
`DecoupledProcessor` pattern): the default Tournament BP cannot serve FDP’s
`branchPlaceholder` path. Pass `--no-o3-fdp` to use the classic coupled O3
front-end and default BP for comparison.

**Memory hierarchy (perf + `local_binary_run.py`):** by default the board uses
**classic private L1I (32KiB, 8-way) + private L1D (32KiB, 8-way) + shared L2
(256KiB, 16-way)** via `PrivateL1SharedL2CacheHierarchy`, still backed by
`SingleChannelDDR3_1600`. This is closer to real cores than `NoCache` (every
fetch/load hit DRAM) and usually raises IPC on memory-heavy kernels like
`dot4_pipeline`. Pass **`--no-cache`** to any of these config scripts to
restore the old direct-to-memory setup when comparing against historical
numbers. Sizes are fixed in the script for reproducibility; tune there to match
a specific FPGA/ASIC.

The script runs three cases per benchmark:

- `baseline_scalar`
  - `gem5_baseline/build/RISCV/gem5.opt`
  - scalar reference binary
- `update_scalar`
  - `gem5/build/RISCV/gem5.opt`
  - same scalar binary as a simulator-control case
- `update_custom`
  - `gem5/build/RISCV/gem5.opt`
  - custom optimized binary

## Hardware-Loop Microbenchmark

A separate, faster microbenchmark harness is provided for isolating the
front-end cost of hardware-loop redirects without mixing it into the larger
`dot4` / `mac` comparison flow.

Source files live in `hwloop_perf_src/`:

- `hwloop_redirect_ref`
  - same arithmetic work as the looped cases
  - inner kernel is fully unrolled, so it serves as the no-redirect reference
- `hwloop_redirect_swloop`
  - same arithmetic work
  - inner kernel uses a software back-edge (`addi`/`bnez`)
- `hwloop_redirect_hwloop`
  - same arithmetic work
  - inner kernel uses `lp.setup`, so repeated loop-backs appear as
    `nonControlRedirects`

Three scales are built for each variant:

- `small`
- `medium`
- `large`

This keeps each run quick while still allowing a simple slope check against
the number of redirects.

## Build The Hardware-Loop Microbench Binaries

```bash
python3 tests/gem5/riscv_ai_ext/build_hwloop_perf_binaries.py
```

## Run The Hardware-Loop Microbenchmark

Default usage:

```bash
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py
```

Useful variants:

```bash
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --skip-build
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --out-dir /tmp/riscv_ai_hwloop
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --cpu o3
```

Prefer `--skip-build` when `hwloop_perf_bin/` is already built.

### Baseline snapshot and regression compare (optimization workflow)

After you trust a gem5 build (smoke tests green), record a reference summary:

```bash
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --save-baseline
```

This writes `hwloop_baseline/reference_summary.json`. Later runs can assert that
checksums are unchanged and `hwloop` `numCycles` does not regress beyond a
slack factor (default 3%):

```bash
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --compare-baseline
```

Shared constants and the loop-back formula live in `hwloop_expectations.py`
(keep in sync with `hwloop_perf_common.hpp`).

The hardware-loop harness runs only on the updated `gem5` tree. It is meant to
answer a narrower question than the main perf suite:

- how many logical loop-backs the hardware loop should execute
- how many of those loop-backs still survive as backend
  `nonControlRedirects`
- how many extra cycles the hardware-loop case pays relative to the unrolled
  reference
- whether hardware loop is better or worse than a software back-edge loop on
  the same hot body

## Hardware-Loop Metrics

The microbenchmark summary reports:

- `board.processor.cores.core.commit.branchMispredicts`
- `board.processor.cores.core.branchPred.mispredicted_0::total`
- `board.processor.cores.core.branchPred.mispredictDueToPredictor_0::total`
- `board.processor.cores.core.branchPred.mispredictDueToBTBMiss_0::total`
- `board.processor.cores.core.branchPred.targetWrong_0::total`
- `board.processor.cores.core.iew.nonControlRedirects`
- `board.processor.cores.core.iew.dispSquashedInsts`
- `board.processor.cores.core.commit.commitSquashedInsts`
- `board.processor.cores.core.fetch.icacheSquashes`

The runner also computes:

- expected loop-back count for the hardware-loop case
- observed backend redirect count for the hardware-loop case
- early-elided redirect count and elision ratio
- `swloop` cycles over `ref`
- `hwloop` cycles over `ref`
- estimated `cycles / observed backend redirect`
- estimated `cycles / expected loop-back`
- `hwloop` speedup vs. `swloop`

When the front-end learns to predict loop-back early, `nonControlRedirects`
should drop below the expected loop-back count. That is a good sign: it means
the fetch path already followed the right target and the backend no longer had
to repair the PC on every iteration.

## Reported Metrics

The summary printed to the terminal and saved to `summary.json` / `summary.csv`
includes:

- `simTicks`
- `simSeconds`
- `hostSeconds`
- `simInsts`
- `board.processor.cores.core.numCycles`
- `board.processor.cores.core.ipc`
- `board.processor.cores.core.cpi`
- `board.processor.cores.core.branchPred.lookups_0::total`
- `board.processor.cores.core.branchPred.condIncorrect`
- `board.processor.cores.core.branchPred.BTBLookups`
- `board.processor.cores.core.branchPred.BTBHits`
- `board.processor.cores.core.iew.branchMispredicts` or commit fallback
- `board.processor.cores.core.iew.nonControlRedirects`

The script also computes:

- speedup vs. baseline by cycles
- speedup vs. baseline by ticks
- control delta for `update_scalar` vs. `baseline_scalar`
- checksum equality across all three cases

## Output Layout

Each run creates a timestamped directory under `perf_results/` unless
`--out-dir` is specified. Inside each benchmark/case directory you will find:

- `stats.txt`
- `stdout.log`
- `stderr.log`
- standard gem5 output files for that run

## Notes

- The custom benchmark code forces `.option norvc` inside asm blocks so
  hardware-loop body sizes stay predictable.
- `dot4_pipeline` is expected to show the clearest gain because `dot4_acc`
  compresses four signed byte multiplies into one custom op.
- `mac_pipeline` is a more conservative benchmark on `o3`; scalar RV64 integer
  `mul + add` is already fairly strong there, so the gain can be smaller than
  the `dot4` case.
- `nonControlRedirects` helps separate ISA-managed PC rewrites (for example
  hardware-loop redirects) from true branch predictor misses in `branchMispredicts`.
- the hardware-loop microbenchmark currently provides the cleanest way to judge
  whether O3 is recognizing loop-back early enough, because all three variants
  share the same arithmetic body and differ only in the inner control-flow
  mechanism
