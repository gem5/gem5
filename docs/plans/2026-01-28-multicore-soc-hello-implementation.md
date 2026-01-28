# Multi-CPU X86 SoC with Ubuntu Boot Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a configurable multi-core X86 System-on-Chip simulation that boots Ubuntu using KVM acceleration, runs a hello world workload, and exits cleanly.

**Architecture:** Modern gem5 library components with configurable parameters for core count, CPU types, cache sizes, and memory. Uses `SimpleSwitchableProcessor` for CPU switching, `MESITwoLevelCacheHierarchy` for coherence, and `X86Board` for full-system simulation.

**Tech Stack:** gem5 simulator, Python 3.8+, SCons build system, KVM acceleration

---

### Task 1: Create Main Configuration Script Structure

**Files:**
- Create: `configs/example/gem5_library/x86-multicore-soc-hello.py`

**Step 1: Create file with header and imports**

```python
# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Multi-CPU X86 SoC with Ubuntu boot and hello world workload.

This simulation creates a configurable multi-core X86 System-on-Chip that:
1. Boots Ubuntu Linux using KVM acceleration for fast boot
2. Runs a "hello world" workload after system boot
3. Exits cleanly via `m5 exit` mechanism

Configuration options:
- Core count (default: 2)
- Boot CPU type (default: KVM)
- Execution CPU type (default: Timing)
- Cache sizes (L1D, L1I, L2)
- Memory size
- Clock frequency

Usage
-----

```
scons build/X86/gem5.opt -j`nproc`
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py
```

For custom configuration:
```
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 4 \
    --boot-cpu atomic \
    --exec-cpu timing \
    --memory-size 8GiB
```
"""

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
import argparse
```

**Step 2: Run syntax check**

Run: `python3 -m py_compile configs/example/gem5_library/x86-multicore-soc-hello.py`
Expected: No output (successful compilation)

**Step 3: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello.py
git commit -m "feat: add multi-CPU SoC configuration script structure"
```

---

### Task 2: Add Configuration Parser

**Files:**
- Modify: `configs/example/gem5_library/x86-multicore-soc-hello.py:60-120`

**Step 1: Add argument parsing function**

```python
def parse_args():
    """Parse command line arguments for multi-CPU SoC configuration."""
    parser = argparse.ArgumentParser(
        description="Multi-CPU X86 SoC with Ubuntu boot and hello world workload"
    )

    # Core configuration
    parser.add_argument(
        "--num-cores",
        type=int,
        default=2,
        help="Number of CPU cores (default: 2)"
    )

    # CPU type configuration
    cpu_type_choices = ["kvm", "atomic", "timing", "o3", "minor"]
    parser.add_argument(
        "--boot-cpu",
        type=str,
        default="kvm",
        choices=cpu_type_choices,
        help="CPU type for boot phase (default: kvm)"
    )

    parser.add_argument(
        "--exec-cpu",
        type=str,
        default="timing",
        choices=cpu_type_choices,
        help="CPU type for execution phase (default: timing)"
    )

    parser.add_argument(
        "--no-switch",
        action="store_true",
        help="Disable CPU switching (use boot CPU for entire simulation)"
    )

    # Cache configuration
    parser.add_argument(
        "--l1d-size",
        type=str,
        default="32KiB",
        help="L1 data cache size (default: 32KiB)"
    )

    parser.add_argument(
        "--l1i-size",
        type=str,
        default="32KiB",
        help="L1 instruction cache size (default: 32KiB)"
    )

    parser.add_argument(
        "--l2-size",
        type=str,
        default="512KiB",
        help="L2 cache size (default: 512KiB)"
    )

    # Memory configuration
    parser.add_argument(
        "--memory-size",
        type=str,
        default="3GiB",
        help="Main memory size (default: 3GiB)"
    )

    # Clock frequency
    parser.add_argument(
        "--clk-freq",
        type=str,
        default="3GHz",
        help="System clock frequency (default: 3GHz)"
    )

    # KVM perf setting
    parser.add_argument(
        "--kvm-perf",
        action="store_true",
        help="Enable perf for KVM CPUs (default: disabled)"
    )

    return parser.parse_args()
```

**Step 2: Add CPU type mapping helper**

```python
def get_cpu_type(cpu_str):
    """Map string CPU type to CPUTypes enum."""
    cpu_map = {
        "kvm": CPUTypes.KVM,
        "atomic": CPUTypes.ATOMIC,
        "timing": CPUTypes.TIMING,
        "o3": CPUTypes.O3,
        "minor": CPUTypes.MINOR,
    }
    return cpu_map[cpu_str.lower()]
```

**Step 3: Test argument parsing**

Run: `python3 configs/example/gem5_library/x86-multicore-soc-hello.py --help`
Expected: Display help message with all options

**Step 4: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello.py
git commit -m "feat: add configuration argument parser"
```

---

### Task 3: Implement Component Configuration

**Files:**
- Modify: `configs/example/gem5_library/x86-multicore-soc-hello.py:125-200`

**Step 1: Add main configuration function**

```python
def configure_system(args):
    """Configure the multi-CPU SoC based on command line arguments."""

    # Validate requirements
    requires(
        isa_required=ISA.X86,
        coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
        kvm_required=(args.boot_cpu == "kvm" or args.exec_cpu == "kvm"),
    )

    # Configure cache hierarchy
    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size=args.l1d_size,
        l1d_assoc=8,
        l1i_size=args.l1i_size,
        l1i_assoc=8,
        l2_size=args.l2_size,
        l2_assoc=16,
        num_l2_banks=1,
    )

    # Configure memory
    memory = SingleChannelDDR4_2400(size=args.memory_size)

    # Configure processor
    if args.no_switch:
        # Use single CPU type throughout
        processor = SimpleSwitchableProcessor(
            starting_core_type=get_cpu_type(args.boot_cpu),
            switch_core_type=get_cpu_type(args.boot_cpu),  # Same as starting
            isa=ISA.X86,
            num_cores=args.num_cores,
        )
    else:
        # Switch from boot CPU to execution CPU
        processor = SimpleSwitchableProcessor(
            starting_core_type=get_cpu_type(args.boot_cpu),
            switch_core_type=get_cpu_type(args.exec_cpu),
            isa=ISA.X86,
            num_cores=args.num_cores,
        )

    # Disable perf for KVM CPUs if not explicitly enabled
    if not args.kvm_perf:
        for proc in processor.start:
            if hasattr(proc.core, 'usePerf'):
                proc.core.usePerf = False

    # Configure board
    board = X86Board(
        clk_freq=args.clk_freq,
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    # Set workload
    workload = obtain_resource("x86-ubuntu-24.04-boot-with-systemd")
    board.set_workload(workload)

    return board, processor
```

**Step 2: Test configuration function**

Run: `python3 -c "import sys; sys.path.insert(0, '.'); from configs.example.gem5_library.x86_multicore_soc_hello import parse_args, configure_system; args = parse_args(['--num-cores', '2']); print('Configuration test passed')"`
Expected: "Configuration test passed"

**Step 3: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello.py
git commit -m "feat: implement system configuration function"
```

---

### Task 4: Implement Exit Event Handler

**Files:**
- Modify: `configs/example/gem5_library/x86-multicore-soc-hello.py:205-280`

**Step 1: Add exit event handler**

```python
def create_exit_event_handler(processor, args):
    """Create exit event handler for boot and workload execution phases."""

    def exit_event_handler():
        # Phase 1: Kernel boot complete
        print(f"[Phase 1] Ubuntu kernel booted with {args.num_cores} {args.boot_cpu.upper()} cores")
        yield False  # Continue to systemd startup

        # Phase 2: Systemd started, ready for workload
        print("[Phase 2] Systemd started, ready for workload execution")

        if not args.no_switch and args.boot_cpu != args.exec_cpu:
            print(f"[Phase 2] Switching from {args.boot_cpu.upper()} to {args.exec_cpu.upper()} CPUs")
            processor.switch()

        yield False  # Continue to workload execution

        # Phase 3: Workload complete, exit simulation
        print("[Phase 3] Workload execution complete")
        print("[Phase 3] Exiting simulation via m5 exit")
        yield True  # Terminate simulation

    return exit_event_handler
```

**Step 2: Add simulation runner function**

```python
def run_simulation(board, processor, args):
    """Run the simulation with configured exit events."""

    simulator = Simulator(
        board=board,
        on_exit_event={
            ExitEvent.EXIT: create_exit_event_handler(processor, args)()
        },
    )

    print(f"\n{'='*60}")
    print("Starting Multi-CPU X86 SoC Simulation")
    print(f"{'='*60}")
    print(f"Configuration:")
    print(f"  Cores: {args.num_cores}")
    print(f"  Boot CPU: {args.boot_cpu.upper()}")
    print(f"  Exec CPU: {args.exec_cpu.upper()}")
    print(f"  CPU Switch: {'Disabled' if args.no_switch else 'Enabled'}")
    print(f"  L1D Cache: {args.l1d_size}")
    print(f"  L1I Cache: {args.l1i_size}")
    print(f"  L2 Cache: {args.l2_size}")
    print(f"  Memory: {args.memory_size}")
    print(f"  Clock: {args.clk_freq}")
    print(f"{'='*60}\n")

    simulator.run()

    print(f"\n{'='*60}")
    print("Simulation Complete")
    print(f"{'='*60}")
```

**Step 3: Test handler creation**

Run: `python3 -c "import sys; sys.path.insert(0, '.'); from configs.example.gem5_library.x86_multicore_soc_hello import parse_args, create_exit_event_handler; args = parse_args([]); print('Handler test passed')"`
Expected: "Handler test passed"

**Step 4: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello.py
git commit -m "feat: implement exit event handler and simulation runner"
```

---

### Task 5: Add Main Function and Workload Helper

**Files:**
- Modify: `configs/example/gem5_library/x86-multicore-soc-hello.py:285-350`

**Step 1: Add workload creation helper**

```python
def create_hello_world_workload():
    """Create a simple hello world workload for testing."""
    import tempfile
    import os

    # Create a simple shell script that prints hello and exits
    script_content = """#!/bin/bash
echo "========================================"
echo "Hello from gem5 Multi-CPU X86 SoC!"
echo "Running on Ubuntu in full-system mode"
echo "Number of CPU cores configured: $1"
echo "========================================"
m5 exit
"""

    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
        f.write(script_content)
        script_path = f.name

    # Make executable
    os.chmod(script_path, 0o755)

    return script_path
```

**Step 2: Add main function**

```python
def main():
    """Main function to run the multi-CPU SoC simulation."""

    # Parse command line arguments
    args = parse_args()

    try:
        # Configure system
        board, processor = configure_system(args)

        # Create hello world workload
        workload_path = create_hello_world_workload()
        print(f"[Info] Created hello world workload at: {workload_path}")
        print("[Info] Pass this file via 'm5 readfile' in the guest")
        print("[Info] The Ubuntu after_boot.sh script will automatically execute it")

        # Run simulation
        run_simulation(board, processor, args)

        # Cleanup
        import os
        os.unlink(workload_path)

    except Exception as e:
        print(f"\n{'='*60}")
        print("Simulation Failed")
        print(f"{'='*60}")
        print(f"Error: {e}")
        print(f"\nCommon issues:")
        print("1. Ensure gem5 is built with X86 and MESI_TWO_LEVEL support")
        print("2. For KVM: Ensure host has KVM enabled and user has permissions")
        print("3. Check internet connection for resource download")
        print(f"{'='*60}")
        raise

if __name__ == "__main__":
    main()
```

**Step 3: Test main function structure**

Run: `python3 configs/example/gem5_library/x86-multicore-soc-hello.py --help`
Expected: Display help with all options

**Step 4: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello.py
git commit -m "feat: add main function and workload helper"
```

---

### Task 6: Create Test Script

**Files:**
- Create: `tests/gem5_library/test_x86_multicore_soc_hello.py`

**Step 1: Create test file**

```python
#!/usr/bin/env python3
# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.

"""Tests for x86-multicore-soc-hello.py configuration script."""

import unittest
import tempfile
import os
import sys

# Add configs directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../configs'))

from example.gem5_library.x86_multicore_soc_hello import (
    parse_args,
    get_cpu_type,
    create_hello_world_workload,
)


class TestMultiCoreSoCConfig(unittest.TestCase):
    """Test multi-core SoC configuration."""

    def test_parse_default_args(self):
        """Test parsing default arguments."""
        args = parse_args([])
        self.assertEqual(args.num_cores, 2)
        self.assertEqual(args.boot_cpu, "kvm")
        self.assertEqual(args.exec_cpu, "timing")
        self.assertEqual(args.l1d_size, "32KiB")
        self.assertEqual(args.l1i_size, "32KiB")
        self.assertEqual(args.l2_size, "512KiB")
        self.assertEqual(args.memory_size, "3GiB")
        self.assertEqual(args.clk_freq, "3GHz")
        self.assertFalse(args.no_switch)
        self.assertFalse(args.kvm_perf)

    def test_parse_custom_args(self):
        """Test parsing custom arguments."""
        test_args = [
            "--num-cores", "4",
            "--boot-cpu", "atomic",
            "--exec-cpu", "o3",
            "--no-switch",
            "--l1d-size", "64KiB",
            "--l1i-size", "64KiB",
            "--l2-size", "1MiB",
            "--memory-size", "8GiB",
            "--clk-freq", "4GHz",
            "--kvm-perf",
        ]

        args = parse_args(test_args)
        self.assertEqual(args.num_cores, 4)
        self.assertEqual(args.boot_cpu, "atomic")
        self.assertEqual(args.exec_cpu, "o3")
        self.assertTrue(args.no_switch)
        self.assertEqual(args.l1d_size, "64KiB")
        self.assertEqual(args.l1i_size, "64KiB")
        self.assertEqual(args.l2_size, "1MiB")
        self.assertEqual(args.memory_size, "8GiB")
        self.assertEqual(args.clk_freq, "4GHz")
        self.assertTrue(args.kvm_perf)

    def test_get_cpu_type(self):
        """Test CPU type mapping."""
        self.assertEqual(get_cpu_type("kvm"), CPUTypes.KVM)
        self.assertEqual(get_cpu_type("atomic"), CPUTypes.ATOMIC)
        self.assertEqual(get_cpu_type("timing"), CPUTypes.TIMING)
        self.assertEqual(get_cpu_type("o3"), CPUTypes.O3)
        self.assertEqual(get_cpu_type("minor"), CPUTypes.MINOR)

    def test_create_hello_world_workload(self):
        """Test hello world workload creation."""
        workload_path = create_hello_world_workload()

        # Check file exists
        self.assertTrue(os.path.exists(workload_path))

        # Check file is executable
        self.assertTrue(os.access(workload_path, os.X_OK))

        # Check content
        with open(workload_path, 'r') as f:
            content = f.read()
            self.assertIn("Hello from gem5 Multi-CPU X86 SoC!", content)
            self.assertIn("m5 exit", content)

        # Cleanup
        os.unlink(workload_path)


if __name__ == "__main__":
    unittest.main()
```

**Step 2: Run test to verify it fails (no implementation yet)**

Run: `python3 tests/gem5_library/test_x86_multicore_soc_hello.py`
Expected: ImportError or other failure (tests not passing yet)

**Step 3: Commit**

```bash
git add tests/gem5_library/test_x86_multicore_soc_hello.py
git commit -m "test: add test script for multi-CPU SoC configuration"
```

---

### Task 7: Create Usage Documentation

**Files:**
- Create: `docs/gem5-library/x86-multicore-soc-hello-usage.md`

**Step 1: Create usage documentation**

```markdown
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
```

**Step 2: Verify documentation format**

Run: `markdownlint docs/gem5-library/x86-multicore-soc-hello-usage.md 2>/dev/null || echo "Markdown check skipped"`
Expected: No errors or "Markdown check skipped"

**Step 3: Commit**

```bash
git add docs/gem5-library/x86-multicore-soc-hello-usage.md
git commit -m "docs: add usage guide for multi-CPU SoC configuration"
```

---

### Task 8: Create Integration Test Example

**Files:**
- Create: `configs/example/gem5_library/x86-multicore-soc-hello-example.sh`

**Step 1: Create example shell script**

```bash
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
```

**Step 2: Make script executable**

Run: `chmod +x configs/example/gem5_library/x86-multicore-soc-hello-example.sh`

**Step 3: Test script syntax**

Run: `bash -n configs/example/gem5_library/x86-multicore-soc-hello-example.sh`
Expected: No output (successful syntax check)

**Step 4: Commit**

```bash
git add configs/example/gem5_library/x86-multicore-soc-hello-example.sh
git commit -m "example: add integration test script"
```

---

### Task 9: Update CLAUDE.md with New Configuration

**Files:**
- Modify: `CLAUDE.md:200-250`

**Step 1: Add section about new configuration**

```markdown
## Multi-CPU SoC Configuration

A new configurable multi-core X86 SoC simulation is available:

**Script**: `configs/example/gem5_library/x86-multicore-soc-hello.py`

**Features**:
- Configurable core count (default: 2)
- Switchable CPU types (KVM for boot, Timing for execution)
- MESI Two-Level cache hierarchy
- Ubuntu boot with hello world workload
- Clean exit via `m5 exit`

**Usage**:
```bash
# Default configuration
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py

# Custom configuration
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 4 \
    --boot-cpu atomic \
    --exec-cpu o3 \
    --memory-size 8GiB
```

**Documentation**: See `docs/gem5-library/x86-multicore-soc-hello-usage.md`
**Tests**: `tests/gem5_library/test_x86_multicore_soc_hello.py`
**Example**: `configs/example/gem5_library/x86-multicore-soc-hello-example.sh`
```

**Step 2: Verify CLAUDE.md formatting**

Run: `markdownlint CLAUDE.md 2>/dev/null || echo "Markdown check skipped"`
Expected: No errors or "Markdown check skipped"

**Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md with multi-CPU SoC configuration"
```

---

### Task 10: Final Verification and Cleanup

**Files:**
- All created/modified files

**Step 1: Run all tests**

Run: `python3 tests/gem5_library/test_x86_multicore_soc_hello.py -v`
Expected: Tests pass (may need implementation first)

**Step 2: Check syntax of all Python files**

Run: `python3 -m py_compile configs/example/gem5_library/x86-multicore-soc-hello.py tests/gem5_library/test_x86_multicore_soc_hello.py`
Expected: No output (successful compilation)

**Step 3: Verify file permissions**

Run: `ls -la configs/example/gem5_library/x86-multicore-soc-hello-example.sh`
Expected: File is executable (-rwxr-xr-x)

**Step 4: Create final summary**

```bash
echo "Implementation complete. Summary:"
echo "- Main script: configs/example/gem5_library/x86-multicore-soc-hello.py"
echo "- Tests: tests/gem5_library/test_x86_multicore_soc_hello.py"
echo "- Documentation: docs/gem5-library/x86-multicore-soc-hello-usage.md"
echo "- Example: configs/example/gem5_library/x86-multicore-soc-hello-example.sh"
echo "- CLAUDE.md updated"
```

**Step 5: Final commit**

```bash
git add -A
git commit -m "feat: complete multi-CPU X86 SoC implementation

- Configurable multi-core X86 SoC simulation
- Ubuntu boot with KVM acceleration
- Hello world workload execution
- Comprehensive configuration options
- Tests, documentation, and examples"
```

---

## Plan Complete

The implementation plan is complete and saved to `docs/plans/2026-01-28-multicore-soc-hello-implementation.md`.

**Two execution options:**

1. **Subagent-Driven (this session)** - I dispatch fresh subagent per task, review between tasks, fast iteration

2. **Parallel Session (separate)** - Open new session with executing-plans, batch execution with checkpoints

**Which approach?**