# gem5 Development Notes

This file contains development notes and documentation for gem5 configurations and features.

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
