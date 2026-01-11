# Disk Image Validator

This utility is used to verify that full system workloads in gem5 function as expected. It checks:

- That expected exit events occur in the correct order
- The `gem5-bridge` driver is properly installed and functioning
- The workload can at least boot with various configurations

## gem5-Bridge Driver Validator

To validate the `gem5-bridge` driver, run the `gem5-bridge-driver-validate.py` script. This script verifies the driver installation by executing a simple C program that makes an m5 hypercall (hypercall number 1234), without requiring superuser privileges.

### Usage

```bash
build/ALL/gem5.opt util/disk-image-validator/gem5-bridge-driver-validate.py --isa <ISA> --workload <WORKLOAD_ID> --resource-version <RESOURCE_VERSION>
```

### Arguments

- `--isa`
  The instruction set architecture (ISA) to use for simulation.
  **Options:** `x86`, `arm`, `riscv`

- `--workload`
  The workload ID to run.

- `--resource-version`
  The version of the workload resource.

### Example

```bash
build/ALL/gem5.opt util/disk-image-validator/gem5-bridge-driver-validate.py --isa x86 --workload x86-ubuntu-24.04-boot-no-systemd --resource-version 4.0.0
```

### Requirements

- gem5 v25.0 or higher must be installed and configured correctly.The version of gem5 that is used must support hypercalls.
- Prebuilt demo board classes must be available.
- The gem5 resources must be able to fetch the specified workload.

### Functionality

- Selects the appropriate demo board based on the specified ISA (`x86`, `arm`, or `riscv`)
- Loads the given workload and optional resource version
- Runs a shell script (`test_gem5_bridge.sh`) during the simulation to test whether m5 hypercall 1234 executes successfully (without requiring `sudo`)
- Prints a success message if the test passes

### Exit Conditions

- If the hypercall executes successfully, a success message is printed. No message is printed upon unsuccessful execution.
- If an invalid ISA is provided, the script raises an exception.

## gem5 Disk Image Validation Script

This script tests disk images to ensure they boot correctly and execute hypercalls in the intended order.

### Usage

```bash
build/ALL/gem5.opt util/disk-image-validator/disk-image-validate.py --isa <ISA> --workload <WORKLOAD_ID> --resource_version <RESOURCE_VERSION> [--validate-npb]
```

### Arguments

- `--isa`
  The instruction set architecture (ISA) to use for simulation.
  **Options:** `x86`, `arm`, `riscv` **(Required)**

- `--workload`
  The workload ID to run. **(Required)**

- `--resource_version`
  The version of the workload resource. **(Required)**

- `--validate-npb`
  Validate the NAS Parallel Benchmarks (NPB) output. *(Optional)*

### Example

```bash
build/ALL/gem5.opt util/disk-img-validator/disk-image-validate.py --isa x86 --workload x86-ubuntu-24.04-npb-is-s --resource_version 3.0.0 --validate-npb
```

### Requirements

- gem5 v25.0 or higher must be installed and properly configured.
- The necessary prebuilt demo board classes must be available.
- The gem5 resources should be able to fetch the specified workload.

### Functionality

- Selects the appropriate demo board based on the specified ISA (`x86`, `arm`, or `riscv`)
- Loads the specified workload and resource version
- Monitors key hypercalls during the simulation:
    - **Hypercall 1**: Kernel boot
    - **Hypercall 2**: In `after_boot.sh`
    - **Hypercall 3**: Done running `after_boot.sh`
    - **Hypercall 4**: Start of Region of Interest (ROI)
    - **Hypercall 5**: End of ROI
- Dumps and resets gem5 statistics at each hypercall.
- Checks whether the hypercalls are executed in the correct order.
- If `--validate-npb` is passed, it verifies the correctness of the NPB output.

### Exit Conditions

- If the hypercalls are executed in the expected order, a success message is printed.
- If the order is incorrect, an error message is printed.
- If `--validate-npb` is used and validation fails, an error message is printed. No additional message is printed if the NPB validation succeeds.

## gem5 Configuration Test Script

This script runs a specified workload across multiple gem5 configurations to verify that gem5 can successfully boot full-system workloads with different CPU models and cache hierarchies.

**Usage:**

```bash
python3 run_config_tests.py --workload <WORKLOAD_ID> --resource_version <RESOURCE_VERSION>
```

**Arguments:**

- `--workload`: The ID of the workload to be executed. (Required)
- `--resource_version`: The version of the workload resource. (Required)

**Example:**

```bash
python3 run_config_tests.py --workload x86-ubuntu-24.04-boot-no-systemd --resource_version 4.0.0
```

**Requirements:**

- The gem5 binary must be built and located at `./build/ALL/gem5.opt`.
- The configuration script must be located at `util/disk-image-validator/config_tester.py`.

**Functionality:**

- Runs gem5 with different configurations:
  - KVM_test
  - MESI_cache_test
  - O3_test
  - MINOR_test
  - ATOMIC_test
  - ATOMIC_2_core_test
  - ATOMIC_4_core_test
  - ATOMIC_8_core_test
- Results are saved in `m5out/<workload_id>-<test_name>`.
- At the end, it prints a summary of the test results and exits with the appropriate status code.

**Exit Codes:**

- `0`: All tests passed.
- `1`: One or more tests failed.
