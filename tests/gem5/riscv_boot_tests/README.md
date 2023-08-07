# RISCV Boot Tests

These tests run a series of Linux boots on the RISCVBoard.
It varies the CPU type, number of CPUs, and memory used for each run.
To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/riscv_boot_tests --length=[length]
```
