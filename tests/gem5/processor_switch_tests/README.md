# Processor Switch

These tests check processor switching in gem5.

To run the FS variant of these tests by themselves, you can run the
following command in the tests directory:

```bash
./main.py run gem5/processor_switch_tests --length=very-long --variant=fast
```

To run the SE variant of these tests, you can run the following:

```bash
./main.py run gem5/processor_switch_tests --length=long --variant=fast
```

To generate the checkpoints used in these tests, `cd` into `configs/checkpoint`
and run the following command:

```bash
path/to/gem5/build/ALL/gem5.fast cross-product-switch-afterboot-take-cpt-{isa}.py --num-cores={number_of_cores}
```

where `isa` is `arm`, `riscv`, or `x86`, and `number_of_cores` is 4, 8, or 16.
Note that the combination of `arm` and 16 cores won't succeed, however.

The checkpoint will be outputted into a subdirectory of `configs/checkpoint`
named `{isa}-ubuntu-24.04-boot-{number_of_cores}-core-checkpoint`.
