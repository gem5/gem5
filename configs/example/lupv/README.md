# README

This README details how to build a RISCV full system that utilizes the
lupIO devices. The LupIO devices were created by Professor Joël Porquet-Lupine,
and more information about the device can be found [here](
https://luplab.cs.ucdavis.edu/assets/lupio/wcae21-porquet-lupio-paper.pdf). The
specs for each individual device can be found [here](
https://gitlab.com/luplab/lupio/lupio-specs), and the Linux drivers for each of
these devices can be found [here](https://gitlab.com/luplab/lupio/linux).

To build the RISCV gem5 binary execute:

```sh
cmake -G Ninja --preset opt -DGEM5_BUILD_VARIANT=RISCV -B build
ninja -C build
```

Then, to run the LupIO example execute:


``` bash
./build/gem5 configs/example/lupv/run_lupv.py  [cpu type] [num cpus]
```

Note: valid cpu types are `atomic` and `timing` for now.

For example:

```bash
./build/gem5 configs/example/lupv/run_lupv.py atomic 1
```

You can observe the stdout of the simulated system in `m5out/system.terminal`.
Then, you can open up a separate terminal and use m5term to connect to the
simulated console. The port number will be specified in the gem5 simulation
as `0: system.remote_gdb: listening for remote gdb on port <port>`

example:

```bash
m5term localhost 3456
```

This should allow you to run busybox, in which you can see the LupIO device at
work!
