# Readfile Tests

These tests run a series of full-system simulations to verify whether a script can be executed inside the simulation via the `readfile` argument.

The tests boot an Ubuntu 24.04 image (without systemd) for the X86, RISC-V, and ARM ISAs.

To run these tests independently, execute the following command from the `tests` directory:

```bash
./main.py run gem5/readfile_tests --length=long
```
