# Multi-CPU X86 SoC with Ubuntu Boot - Usage Guide

## Overview

The `x86-multicore-soc-hello.py` script provides a configurable multi-core X86 System-on-Chip simulation that:
1. Boots Ubuntu Linux using KVM acceleration
2. Runs a "hello world" workload after boot
3. Exits cleanly via `m5 exit`

## Quick Start

```bash
# Build gem5 for X86
scons build/X86/gem5.opt -j`nproc`

# Run with default configuration (2 cores, KVM boot, Timing execution)
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py
```

## Configuration Options

### Core Configuration
- `--num-cores`: Number of CPU cores (default: 2)
- `--boot-cpu`: CPU type for boot phase: kvm, atomic, timing, o3, minor (default: kvm)
- `--exec-cpu`: CPU type for execution phase (default: timing)
- `--no-switch`: Disable CPU switching (use boot CPU throughout)

### Cache Configuration
- `--l1d-size`: L1 data cache size (default: 32KiB)
- `--l1i-size`: L1 instruction cache size (default: 32KiB)
- `--l2-size`: L2 cache size (default: 512KiB)

### Memory Configuration
- `--memory-size`: Main memory size (default: 3GiB)
- `--clk-freq`: System clock frequency (default: 3GHz)

### KVM Settings
- `--kvm-perf`: Enable perf for KVM CPUs (default: disabled)

## Examples

### 4-core server configuration
```bash
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 4 \
    --boot-cpu kvm \
    --exec-cpu o3 \
    --l2-size 2MiB \
    --memory-size 8GiB
```

### Debug configuration (fast boot only)
```bash
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 2 \
    --boot-cpu kvm \
    --exec-cpu kvm \
    --no-switch \
    --kvm-perf
```

### Detailed timing simulation
```bash
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 1 \
    --boot-cpu atomic \
    --exec-cpu timing \
    --memory-size 2GiB
```

## Simulation Phases

1. **Boot Phase**: Ubuntu boots using specified boot CPU type
2. **Switch Phase**: CPU switches to execution type (if enabled)
3. **Workload Phase**: Hello world program executes via `m5 readfile`
4. **Exit Phase**: Simulation terminates via `m5 exit`

## Output

The script provides clear phase-based output:
```
============================================================
Starting Multi-CPU X86 SoC Simulation
============================================================
Configuration:
  Cores: 2
  Boot CPU: KVM
  Exec CPU: TIMING
  CPU Switch: Enabled
  L1D Cache: 32KiB
  L1I Cache: 32KiB
  L2 Cache: 512KiB
  Memory: 3GiB
  Clock: 3GHz
============================================================

[Phase 1] Ubuntu kernel booted with 2 KVM cores
[Phase 2] Systemd started, ready for workload execution
[Phase 2] Switching from KVM to TIMING CPUs
[Phase 3] Workload execution complete
[Phase 3] Exiting simulation via m5 exit

============================================================
Simulation Complete
============================================================
```

## Requirements

- gem5 built for X86 with MESI_TWO_LEVEL protocol
- KVM enabled on host (for KVM CPU type)
- Internet access for Ubuntu resource download (first run only)
- Sufficient disk space for Ubuntu image (~2GB)

## Troubleshooting

### KVM errors
- Ensure KVM is enabled: `lsmod | grep kvm`
- Check user permissions: user in `kvm` group
- Verify gem5 built with KVM support

### Resource download errors
- Check internet connection
- Verify gem5 resources server accessibility
- Check disk space

### Build errors
- Ensure gem5 built with: `scons build/X86/gem5.opt`
- Verify MESI_TWO_LEVEL protocol included in build
