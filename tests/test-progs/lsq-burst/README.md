# lsq-burst

Small SE-mode stress program that issues many independent cache-line-strided
loads. It is intended to exercise high memory-level parallelism in the O3
LSQ / IEW path.

## Motivation

Useful for reproducing
[gem5 issue #3096](https://github.com/gem5/gem5/issues/3096): with a narrow
writeback width, a burst of LSQ completions can push `IEW::instToCommit`
past the `iewQueue` TimeBuffer `future` bound and abort with:

```
Assertion `idx >= -past && idx <= future' failed
```

## Build

```sh
cd tests/test-progs/lsq-burst/src
make CROSS_COMPILE=riscv64-linux-gnu-
make install
```

This produces `bin/riscv/linux/lsq-burst`.

## Reproduce issue #3096 with the stock SE config

Narrow the O3 pipeline so the per-cycle writeback capacity
`(forwardComSize + 1) * wbWidth` is small, then run this binary:

```sh
./build/RISCV/gem5.opt configs/deprecated/example/se.py \
  --cpu-type=RiscvO3CPU \
  --caches \
  --cmd=tests/test-progs/lsq-burst/bin/riscv/linux/lsq-burst \
  -P 'system.cpu[0].wbWidth = 1' \
  -P 'system.cpu[0].fetchWidth = 1' \
  -P 'system.cpu[0].decodeWidth = 1' \
  -P 'system.cpu[0].renameWidth = 1' \
  -P 'system.cpu[0].issueWidth = 1' \
  -P 'system.cpu[0].commitWidth = 1' \
  -P 'system.cpu[0].squashWidth = 1'
```

On an unfixed tree this typically aborts quickly inside `IEW::instToCommit`.
