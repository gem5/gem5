# CHI Protocol with different ISAs test

The purpose of this test is to ensure the CHI protocol functions across ARM, X86, and RISCV ISA targets with varying numbers of CPU cores. The tests can be run with the following command:

```shell
# In the "tests" directory
./main.py run --length=long -j`nproc` gem5/chi_protocol
```

## CHI partial DMA write test

`test_chi_dma_partial_write.py` runs `MemTest` testers on the L1 caches and on
a Ruby DMA port of the CHI `PrivateL1CacheHierarchy`, all accessing the same
cache lines. It checks the home node's handling of `WriteUniquePtl` requests
to lines shared by the L1 caches. It is a quick test:

```shell
# In the "tests" directory
./main.py run -j`nproc` gem5/chi_protocol
```
