# CHI Protocol with different ISAs test

The purpose of this test is to ensure the CHI protocol functions across ARM, X86, and RISCV ISA targets with varying numbers of CPU cores. The tests can be run with the following command:

```shell
# In the "tests" directory
./main.py run --length=long -j`nproc` gem5/chi_protocol
```
