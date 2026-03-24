# RISC-V AI Extension Smoke Tests

This directory contains a small SE-mode regression suite for the custom
RISC-V instructions added in `src/arch/riscv`.

## Coverage

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

## Build the test binaries

The binaries are built locally and are not pulled from `gem5-resources`.

```bash
python3 tests/gem5/riscv_ai_ext/build_binaries.py
```

The build script looks for one of these compilers:

- `riscv64-linux-gnu-gcc`
- `riscv64-unknown-linux-gnu-gcc`
- `riscv64-unknown-elf-gcc`

You can also override the toolchain prefix:

```bash
CROSS_COMPILE=riscv64-linux-gnu- python3 tests/gem5/riscv_ai_ext/build_binaries.py
```

## Run the suite

From `tests/`:

```bash
./main.py run gem5/riscv_ai_ext --length=quick
```

If you only want one test:

```bash
./main.py run gem5/riscv_ai_ext --length=quick --include-tags "quick"
```

## Notes

- The assembly files force `.option norvc` so loop-body offsets remain easy to
  reason about.
- This suite is intentionally small and is meant to catch semantic regressions
  before running larger comparison and performance experiments.
