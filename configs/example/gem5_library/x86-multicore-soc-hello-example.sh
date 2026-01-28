#!/bin/bash
# Example usage of x86-multicore-soc-hello.py
# This script demonstrates different configurations

set -e

GEM5_BINARY="./build/X86/gem5.opt"
CONFIG_SCRIPT="configs/example/gem5_library/x86-multicore-soc-hello.py"

# Check if gem5 binary exists
if [ ! -f "$GEM5_BINARY" ]; then
    echo "Error: gem5 binary not found at $GEM5_BINARY"
    echo "Build gem5 first: scons build/X86/gem5.opt -j\`nproc\`"
    exit 1
fi

echo "========================================"
echo "Multi-CPU X86 SoC Example Configurations"
echo "========================================"

# Example 1: Default configuration
echo ""
echo "Example 1: Default configuration (2 cores, KVM → Timing)"
echo "--------------------------------------------------------"
$GEM5_BINARY $CONFIG_SCRIPT \
    --num-cores 2 \
    --boot-cpu kvm \
    --exec-cpu timing \
    --memory-size 3GiB

# Example 2: 4-core configuration
echo ""
echo "Example 2: 4-core configuration"
echo "--------------------------------"
$GEM5_BINARY $CONFIG_SCRIPT \
    --num-cores 4 \
    --boot-cpu atomic \
    --exec-cpu o3 \
    --l2-size 1MiB \
    --memory-size 4GiB

# Example 3: Debug mode (KVM only)
echo ""
echo "Example 3: Debug mode (KVM only, no switch)"
echo "--------------------------------------------"
$GEM5_BINARY $CONFIG_SCRIPT \
    --num-cores 2 \
    --boot-cpu kvm \
    --exec-cpu kvm \
    --no-switch \
    --kvm-perf

echo ""
echo "========================================"
echo "All examples completed"
echo "========================================"
