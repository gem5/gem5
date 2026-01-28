# Multi-CPU X86 SoC with Ubuntu Boot and Hello World Workload

## Design Overview

This document outlines the design for a configurable multi-core X86 System-on-Chip (SoC) simulation in gem5 that:
1. Boots Ubuntu Linux using KVM acceleration for fast boot
2. Runs a "hello world" workload after system boot
3. Exits cleanly via `m5 exit` mechanism

The design emphasizes configurability, reusability, and follows gem5 library patterns.

## Architecture

### Target Platform
- **Architecture**: X86 (supports KVM hardware acceleration)
- **Simulation Mode**: Full-system (FS) with complete OS boot
- **Core Components**: Modern gem5 library components (`gem5.components.*`)

### Design Principles
1. **Configurability**: All major parameters exposed as configuration options
2. **Performance**: KVM acceleration for fast boot when debugging/speed is needed
3. **Clarity**: Clean separation between system configuration and workload
4. **Extensibility**: Modular design for future enhancements

## Component Design

### CPU Subsystem
- **Processor Type**: `SimpleSwitchableProcessor` for flexible CPU switching
- **Core Count**: Configurable parameter (default: 2 cores)
- **CPU Types**:
  - Boot CPU: Configurable (default: `CPUTypes.KVM` for fast boot)
  - Execution CPU: Configurable (default: `CPUTypes.TIMING` for detailed simulation)
- **ISA**: `ISA.X86` (required for KVM support)

### Cache Hierarchy
- **Type**: `MESITwoLevelCacheHierarchy` (MESI coherence protocol)
- **L1 Cache**: Private per-core, configurable size and associativity
- **L2 Cache**: Shared among cores, configurable size and bank count
- **Default Configuration**:
  - L1D: 32KiB, 8-way associative
  - L1I: 32KiB, 8-way associative
  - L2: 512KiB, 16-way associative, 1 bank

### Memory System
- **Type**: `SingleChannelDDR4_2400`
- **Size**: Configurable (default: "3GiB" for Ubuntu)
- **Frequency**: 2400 MT/s (configurable via different DDR4 variants)

### Board Configuration
- **Board Type**: `X86Board` (full-system X86 support)
- **Clock Frequency**: Configurable (default: "3GHz")
- **Integration**: Combines CPU, cache, memory, and I/O subsystems

## Boot and Workflow Execution

### Ubuntu Boot Process
1. **Resource Acquisition**: Uses `obtain_resource("x86-ubuntu-24.04-boot-with-systemd")`
2. **KVM Acceleration**: Fast boot using KVM CPUs (configurable)
3. **Exit Event Phases**:
   - First `m5 exit`: Kernel boot complete
   - Second `m5 exit`: Systemd started
   - Third `m5 exit`: Ready for workload execution

### Workload Delivery and Execution
1. **Program Creation**: Simple "hello world" program (C or shell script)
2. **File Transfer**: Via `m5 readfile` mechanism (standard gem5 pattern)
3. **Automatic Execution**: Ubuntu's `after_boot.sh` script runs any file passed via `m5 readfile`
4. **Clean Exit**: After execution, script calls `m5 exit` to terminate simulation

### Exit Event Handler
```python
def exit_event_handler():
    # Phase 1: Kernel boot complete
    print("Ubuntu kernel booted")
    yield False  # Continue to systemd startup

    # Phase 2: Systemd started, ready for workload
    print("Systemd started, passing hello world via m5 readfile")
    yield False  # Continue to workload execution

    # Phase 3: Workload complete, exit simulation
    print("Hello world executed, exiting simulation")
    yield True  # Terminate simulation
```

## Configuration Interface

### Configuration File Structure
```python
# configs/example/gem5_library/x86-multicore-soc-hello.py

# Configuration parameters (with defaults)
config = {
    "num_cores": 2,
    "boot_cpu": CPUTypes.KVM,
    "exec_cpu": CPUTypes.TIMING,
    "switch_on_boot": True,
    "l1d_size": "32KiB",
    "l1i_size": "32KiB",
    "l2_size": "512KiB",
    "memory_size": "3GiB",
    "clk_freq": "3GHz",
}
```

### Command-line Usage
```bash
# Basic usage
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py

# Custom configuration
./build/X86/gem5.opt configs/example/gem5_library/x86-multicore-soc-hello.py \
    --num-cores 4 \
    --boot-cpu atomic \
    --exec-cpu o3 \
    --memory-size 8GiB
```

### Workload Preparation Script
```bash
# create-hello-world.sh
echo '#!/bin/bash
echo "Hello from gem5 multi-CPU SoC!"
m5 exit' > hello.sh

# Or C program
echo '#include <stdio.h>
int main() { printf("Hello World!\\n"); return 0; }' > hello.c
gcc -static -o hello hello.c
```

## Implementation Details

### Core Classes
1. **`X86MultiCoreSoC`**: Main configuration class managing all parameters
2. **`SoCExitEventHandler`**: Handles phase transitions and CPU switching
3. **`HelloWorldWorkload`**: Manages workload creation and execution

### Design Patterns
1. **Factory Pattern**: Component creation based on configuration
2. **Builder Pattern**: SoC assembly from configured components
3. **Strategy Pattern**: Different CPU switching strategies
4. **Observer Pattern**: Exit event monitoring

### Error Handling
- **Pre-simulation**: Validate KVM availability, resource existence
- **Runtime**: Monitor for kernel panics, boot failures
- **Post-simulation**: Verify workload executed correctly

## Extensibility Points

### CPU Enhancements
- Support for additional CPU types (O3, Minor)
- Heterogeneous core configurations (big.LITTLE style)
- Custom CPU parameters (branch predictors, ROB sizes)

### Memory System
- Multi-channel memory configurations
- HBM (High Bandwidth Memory) support
- NVM (Non-Volatile Memory) interfaces
- External memory system integration (DRAMSim2/3)

### Cache Hierarchy
- L3 cache addition
- Different coherence protocols (CHI, MOESI)
- Custom cache replacement policies

### Workload System
- Benchmark suites (SPEC CPU, NPB, PARSEC)
- Custom application integration
- Multi-program workloads
- Interactive workloads

### Monitoring and Analysis
- Performance counter integration
- Power estimation models
- Thermal modeling
- Custom statistics collection

## Testing Strategy

### Unit Tests
- Individual component validation
- Configuration parameter testing
- Error condition handling

### Integration Tests
- Full boot + workload execution
- Different CPU type combinations
- Various core count configurations

### Performance Tests
- Boot time measurements
- Simulation speed comparisons
- Memory usage profiling

### Regression Tests
- Ensure existing functionality preserved
- Backward compatibility checks
- Resource dependency validation

## Performance Considerations

### Boot Time Optimization
- **KVM**: Fastest boot (seconds) - ideal for debugging
- **Atomic**: Moderate speed - good for warmup
- **Timing**: Slowest but most detailed - for final simulations

### Memory Configuration
- **Default**: 3GiB sufficient for Ubuntu boot
- **Scalable**: Adjust based on core count and workload needs
- **Caching**: Disk images cached locally after first download

### Simulation Speed Trade-offs
- **KVM only**: Maximum speed, less timing detail
- **KVM → Timing**: Fast boot + detailed execution
- **Timing only**: Consistent timing, slower overall

## Dependencies and Requirements

### Build Requirements
- gem5 compiled for X86 with MESI_TWO_LEVEL protocol
- KVM support enabled in gem5 build
- SCons build system

### Runtime Requirements
- KVM enabled host system (for KVM acceleration)
- Internet access for resource download (first run only)
- Sufficient disk space for Ubuntu image (~2GB)

### Python Dependencies
- gem5 Python library components
- Standard Python 3.8+ libraries

## Future Enhancements

### Short-term (Phase 2)
1. Checkpoint/restore support
2. Multiple workload support
3. Enhanced monitoring and statistics

### Medium-term (Phase 3)
1. Heterogeneous CPU configurations
2. Advanced memory systems
3. Network and storage device integration

### Long-term (Phase 4)
1. GPU acceleration support
2. Complex I/O subsystem
3. Power and thermal modeling

## Conclusion

This design provides a flexible, configurable multi-core X86 SoC simulation framework that balances performance (via KVM acceleration) with detailed simulation capabilities. The modular design follows gem5 best practices and provides multiple extension points for future enhancements while maintaining a clean, user-friendly interface for common simulation tasks.
