#!/bin/bash
# encoding: utf-8

# manual run of each bench for each ISA
# this runs the base benchmarks

# setup
mkdir -p ./ece_562_benchmarks/arm/base_results/sha
mkdir -p ./ece_562_benchmarks/arm/base_results/queens
mkdir -p ./ece_562_benchmarks/arm/base_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/arm/base_results/bfs

mkdir -p ./ece_562_benchmarks/riscv/base_results/sha
mkdir -p ./ece_562_benchmarks/riscv/base_results/queens
mkdir -p ./ece_562_benchmarks/riscv/base_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/riscv/base_results/bfs

mkdir -p ./ece_562_benchmarks/x86/base_results/sha
mkdir -p ./ece_562_benchmarks/x86/base_results/queens
mkdir -p ./ece_562_benchmarks/x86/base_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/x86/base_results/bfs


mkdir -p ./ece_562_benchmarks/arm/lfru_results/sha
mkdir -p ./ece_562_benchmarks/arm/lfru_results/queens
mkdir -p ./ece_562_benchmarks/arm/lfru_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/arm/lfru_results/bfs

mkdir -p ./ece_562_benchmarks/riscv/lfru_results/sha
mkdir -p ./ece_562_benchmarks/riscv/lfru_results/queens
mkdir -p ./ece_562_benchmarks/riscv/lfru_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/riscv/lfru_results/bfs

mkdir -p ./ece_562_benchmarks/x86/lfru_results/sha
mkdir -p ./ece_562_benchmarks/x86/lfru_results/queens
mkdir -p ./ece_562_benchmarks/x86/lfru_results/blocked-matmul
mkdir -p ./ece_562_benchmarks/x86/lfru_results/bfs

## sha
### base
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/arm.py ./ece_562_benchmarks/arm/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/arm/base_results/sha
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/riscv.py ./ece_562_benchmarks/riscv/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/riscv/base_results/sha
./build/X86/gem5.opt ./ece_562_benchmarks/x86/x86.py ./ece_562_benchmarks/x86/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/x86/base_results/sha
### lfru
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/lfru_arm.py ./ece_562_benchmarks/arm/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/arm/lfru_results/sha
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/lfru_riscv.py ./ece_562_benchmarks/riscv/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/riscv/lfru_results/sha
./build/X86/gem5.opt ./ece_562_benchmarks/x86/lfru_x86.py ./ece_562_benchmarks/x86/benchmarks/sha && mv m5out/* ./ece_562_benchmarks/x86/lfru_results/sha

## blocked-matmul
### base
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/arm.py ./ece_562_benchmarks/arm/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/arm/base_results/blocked-matmul
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/riscv.py ./ece_562_benchmarks/riscv/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/riscv/base_results/blocked-matmul
./build/X86/gem5.opt ./ece_562_benchmarks/x86/x86.py ./ece_562_benchmarks/x86/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/x86/base_results/blocked-matmul
### lfru
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/lfru_arm.py ./ece_562_benchmarks/arm/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/arm/lfru_results/blocked-matmul
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/lfru_riscv.py ./ece_562_benchmarks/riscv/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/riscv/lfru_results/blocked-matmul
./build/X86/gem5.opt ./ece_562_benchmarks/x86/lfru_x86.py ./ece_562_benchmarks/x86/benchmarks/blocked-matmul && mv m5out/* ./ece_562_benchmarks/x86/lfru_results/blocked-matmul

## queens
### base
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/arm.py ./ece_562_benchmarks/arm/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/arm/base_results/queens
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/riscv.py ./ece_562_benchmarks/riscv/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/riscv/base_results/queens
./build/X86/gem5.opt ./ece_562_benchmarks/x86/x86.py ./ece_562_benchmarks/x86/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/x86/base_results/queens
### lfru
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/lfru_arm.py ./ece_562_benchmarks/arm/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/arm/lfru_results/queens
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/lfru_riscv.py ./ece_562_benchmarks/riscv/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/riscv/lfru_results/queens
./build/X86/gem5.opt ./ece_562_benchmarks/x86/lfru_x86.py ./ece_562_benchmarks/x86/benchmarks/queens && mv m5out/* ./ece_562_benchmarks/x86/lfru_results/queens

## bfs
### base
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/arm.py ./ece_562_benchmarks/arm/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/arm/base_results/bfs
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/riscv.py ./ece_562_benchmarks/riscv/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/riscv/base_results/bfs
./build/X86/gem5.opt ./ece_562_benchmarks/x86/x86.py ./ece_562_benchmarks/x86/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/x86/base_results/bfs
### lfru
./build/ARM/gem5.opt ./ece_562_benchmarks/arm/lfru_arm.py ./ece_562_benchmarks/arm/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/arm/lfru_results/bfs
./build/RISCV/gem5.opt ./ece_562_benchmarks/riscv/lfru_riscv.py ./ece_562_benchmarks/riscv/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/riscv/lfru_results/bfs
./build/X86/gem5.opt ./ece_562_benchmarks/x86/lfru_x86.py ./ece_562_benchmarks/x86/benchmarks/BFS && mv m5out/* ./ece_562_benchmarks/x86/lfru_results/bfs
