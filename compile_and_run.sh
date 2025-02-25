#!/bin/bash
export RISCV_BIN=/opt/tools/riscv/bin
export GEM5=/opt/gem5_hp/build/RISCV/gem5.opt
export VISUALIZER=/opt/riscv_gem5_visualizer/target/debug/risc5

filename=$(basename $1)
prog_name="build/${filename%.*}"
log_name="log/${filename%.*}.log"

mkdir -p build
mkdir -p log

$RISCV_BIN/riscv32-unknown-elf-gcc -march=rv32imf -mabi=ilp32 -mno-relax -nostartfiles $1 -o $prog_name

$GEM5 --debug-flags=MinorGUI gem5_config.py --cpu-type MinorCPU --caches --l1d_size 8388608 --l1i_size 8388608 --cacheline_size 512 --cpu-clock 1MHz --sys-clock 10GHz -c $prog_name > $log_name

$VISUALIZER $log_name

unset RISCV_BIN
unset GEM5
unset VISUALIZER
