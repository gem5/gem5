# X86 Boot Tests

These tests run a series of Linux boots on the X86Board.
It varies the CPU type, number of CPUs, and memory used for each run.
To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/x86_boot_tests --length=[length]
```
