# Regression Tests

These tests check the performance of gem5 when running hello world binaries or
matrix multiply workloads on the Arm, RISC-V, and X86 ISAs. They check the
current run's IPCs against reference values taken on gem5 v25.0.0.0.

To run these tests independently, run:

```bash
./main.py run gem5/regression_tests -t <number of threads to run tests with>
```
