#!/usr/bin/env bash

GEM5=build/RISCV/gem5.debug
CONFIG=configs/example/gem5_library/riscv-ubuntu-run.py

${GEM5} \
  --debug-flags=TLB,PageTableWalker\
  ${CONFIG}
