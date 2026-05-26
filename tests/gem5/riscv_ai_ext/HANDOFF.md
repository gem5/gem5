# RISC-V AI Extension gem5 Handoff

Date: 2026-05-26

This note captures the current gem5-side state for the custom RISC-V AI
extension work. It is intended for continuing the work in a new chat/session.

## Project Context

The FPGA implementation is based on CV32E40P on the DE2i-150 board. The
hardware-side instruction mapping is treated as the current source of truth.
The gem5 work is being aligned to that implementation so the theory/simulation
story and the FPGA story stay consistent.

Main local paths:

- FPGA SoC repo: `/home/duydonv/de2i150_cv32e40p_soc`
- CV32E40P core repo: `/home/duydonv/cv32e40p`
- gem5 repo: `/home/duydonv/gem5`
- gem5 test suite: `/home/duydonv/gem5/tests/gem5/riscv_ai_ext`

## ISA Semantics Chosen

The gem5 prototype encoding still uses the custom-0 opcode space. Exact opcode
bits do not need to match the FPGA kit, but instruction semantics should match
the mapped CV32E40P behavior.

- `mac`: signed multiply-accumulate, `rd = rd + rs1 * rs2`.
- `dot4_acc`: packed signed int8 dot product over four lanes, accumulated into
  old `rd`.
- `relu`: signed max with zero.
- `clamp`: clamp signed `rs1` into `[0, rs2]`; the upper bound is read from a
  register, not from an immediate.
- `p.lw`: load word from address `rs1`, write result to `rd`, then update
  `rs1 += sext(imm12)`.
- `lp.setup`: uses raw CORE-V immediate convention, `raw_imm = body_words + 1`.
  gem5 stores the loop tail PC internally as `PC + (raw_imm << 2) - 4` because
  the existing redirect path compares against the current PC.

## Core gem5 Files Touched

Instruction semantics and decode/model plumbing were updated in:

- `src/arch/riscv/insts/ai_ext.hh`
- `src/arch/riscv/insts/ai_ext.cc`
- `src/arch/riscv/insts/mem.cc`
- `src/arch/riscv/isa/decoder.isa`
- `src/arch/riscv/isa/formats/custom.isa`

Build gem5 with all available laptop cores when needed:

```bash
scons build/RISCV/gem5.opt -j$(nproc)
```

## Smoke Tests

Smoke tests were updated for the new semantics:

- `src/ai_ext_macros.inc`
- `src/ai_ops_smoke.S`
- `src/p_lw_smoke.S`
- `src/hwloop_smoke.S`

Expected coverage:

- `clamp` register upper bound
- `p.lw` load-before-post-increment
- `lp.setup` raw immediate behavior

Previously observed result: smoke tests passed after the semantic sync.

Useful commands:

```bash
python3 tests/gem5/riscv_ai_ext/build_binaries.py
python3 tests/gem5/riscv_ai_ext/test.py
```

## Main Performance Benchmarks

The old `dot4_pipeline` and `mac_pipeline` benchmark sources were replaced by
three scenarios that match the FPGA benchmark story better:

- `mac_clamp`
  - scalar: normal multiply/add loop plus C clamp
  - custom: `mac` plus register-bound `clamp`
- `dot4_acc_clamp`
  - scalar: normal loads and scalar packed-byte dot4
  - custom: normal loads/control plus `dot4_acc` and register-bound `clamp`
- `dot4_plw_lp_clamp`
  - scalar: same scalar reference as `dot4_acc_clamp`
  - custom: `lp.setup`, `p.lw`, `dot4_acc`, register-bound `clamp`

Relevant files:

- `perf_src/perf_common.hpp`
- `perf_src/custom_kernels.hpp`
- `perf_src/mac_clamp_scalar.cpp`
- `perf_src/mac_clamp_custom.cpp`
- `perf_src/dot4_acc_clamp_scalar.cpp`
- `perf_src/dot4_acc_clamp_custom.cpp`
- `perf_src/dot4_plw_lp_clamp_scalar.cpp`
- `perf_src/dot4_plw_lp_clamp_custom.cpp`
- `build_perf_binaries.py`
- `run_perf_compare.py`
- `configs/perf_binary_run.py`

Important harness detail:

- Input data is initialized before the measured region.
- The benchmark calls guest m5ops `resetstats` immediately before the kernel and
  `dumpresetstats` immediately after the kernel.
- `run_perf_compare.py` reads the first stats section from `stats.txt`; that is
  the kernel-only region. Later stats sections are print/exit overhead.

Build and run:

```bash
python3 tests/gem5/riscv_ai_ext/build_perf_binaries.py
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --skip-build
python3 tests/gem5/riscv_ai_ext/run_perf_compare.py --skip-build --no-cache
```

Observed results on 2026-05-26:

- Cached O3:
  - `mac_clamp`: custom slightly slower than scalar, about `0.985x`.
  - `dot4_acc_clamp`: custom about `2.93x`.
  - `dot4_plw_lp_clamp`: custom about `2.67x`.
- NoCache + DDR3:
  - `mac_clamp`: custom barely faster, about `1.002x`.
  - `dot4_acc_clamp`: custom about `3.26x`.
  - `dot4_plw_lp_clamp`: custom about `8.91x`.

Interpretation so far:

- Cache does not "recognize" custom instructions. It only sees instruction and
  data addresses.
- Cached O3 changes the bottleneck from memory/fetch to O3 pipeline behavior.
- `mac_clamp` can be weak on O3 because scalar `mulw` can overlap better, while
  fused `mac` puts the accumulator dependency directly on the multiply-like op.
- The current custom `mac` is modeled as an `IntMultOp`; O3 default integer
  multiply latency is a possible modeling mismatch versus the FPGA core.
- `--no-cache` is useful as a sensitivity test, but current `NoCache` still
  talks to `SingleChannelDDR3_1600`, so it may overstate memory/fetch cost versus
  FPGA firmware executing from RAM.

## Hardware-Loop Microbenchmark

The hardware-loop-only benchmark is intended to isolate loop redirect/front-end
cost from the larger dot4/mac kernels.

Files:

- `hwloop_perf_src/hwloop_perf_common.hpp`
- `hwloop_perf_src/hwloop_redirect_ref.cpp`
- `hwloop_perf_src/hwloop_redirect_swloop.cpp`
- `hwloop_perf_src/hwloop_redirect_hwloop.cpp`
- `build_hwloop_perf_binaries.py`
- `run_hwloop_perf_compare.py`
- `hwloop_expectations.py`

Current semantics:

- The body has 8 non-compressed instructions.
- `lp.setup` now encodes raw immediate `9`, i.e. `body_words + 1`.
- Disassembly confirmed the encoded immediate is `0x009...`, not old byte
  immediate `32`.

Build and run:

```bash
python3 tests/gem5/riscv_ai_ext/build_hwloop_perf_binaries.py
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --skip-build
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --skip-build --no-cache
```

Observed results on 2026-05-26:

- Cached O3:
  - Checksums match across `ref`, `swloop`, and `hwloop`.
  - `hwloop` is much slower than `swloop`.
  - Example large: `hwloop speedup vs swloop = 0.5785x`.
  - `nonControlRedirects` is nonzero and scales as `2048/8192/32768`, much lower
    than logical loop-backs but still expensive.
- NoCache + DDR3:
  - Checksums match.
  - `hwloop` becomes faster than `swloop`.
  - Example large: `hwloop speedup vs swloop = 1.3928x`.
  - `nonControlRedirects = 0`, so the current front-end path fully hides backend
    loop redirects in this configuration.

Current read:

- NoCache results look more favorable for the current hardware-loop path.
- Cached O3 results show the current custom O3/FDP implementation is not yet
  clean or optimized.
- FDP was custom-added earlier to make O3 run hardware loops correctly; it is
  not standard upstream O3 behavior and should be reviewed separately.

## Current Worktree Notes

There are many intentional modified/untracked files in `tests/gem5/riscv_ai_ext`
and RISC-V ISA files. Do not blindly reset the worktree.

Known generated/result directories:

- `tests/gem5/riscv_ai_ext/perf_bin`
- `tests/gem5/riscv_ai_ext/perf_results`
- `tests/gem5/riscv_ai_ext/hwloop_perf_bin`
- `tests/gem5/riscv_ai_ext/hwloop_perf_results`

If a new baseline is needed, regenerate it only after the model/test direction
is settled:

```bash
python3 tests/gem5/riscv_ai_ext/run_hwloop_perf_compare.py --save-baseline
```

## Next Work Items

1. Add a more kit-like memory mode.
   - Current cached O3 is not close to the FPGA core.
   - Current `--no-cache` uses DDR3 and may over-penalize memory.
   - Proposed next mode: `NoCache + SingleChannelSimpleMemory` with low latency
     and high bandwidth, e.g. a `fast-ram` mode, to approximate firmware running
     from RAM without L1/L2 cache.

2. Review and clean the custom O3/FDP hardware-loop path.
   - Understand why cached O3 still has high `commitSquashedInsts` and expensive
     residual redirects.
   - Confirm whether the loop-back should be fully predicted in fetch or still
     repaired in execute/commit.
   - Keep this separate from the main instruction semantics sync.

3. Revisit operation latency/modeling.
   - `mac` currently behaves like an integer multiply op in O3.
   - If FPGA `cv.mac` effectively has different latency/throughput, gem5 should
     either document the mismatch or add a better op-class/latency model.

4. Decide final reporting structure.
   - Main thesis story should emphasize real FPGA execution.
   - gem5 should support the theory/semantic/relative trend story, not override
     kit measurements when O3 is structurally different from CV32E40P.
   - Recommended tables: cached O3, no-cache DDR3 sensitivity, and fast-RAM/no
     cache once implemented.

5. After model/test direction is stable, update README and regenerate any
   reference summaries.

