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
./main.py run gem5/riscv_ai_ext --length=quick
```

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
```

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
