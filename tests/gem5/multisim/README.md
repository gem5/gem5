# Multisim

These tests check if MultiSim works under certain conditions in gem5.

To run the longer tests, varying length tests, and processor switching tests,
use the following command:

```bash
./main.py run gem5/multisim --length=very-long
```

To run the tests for taking and restoring from checkpoints, use the following
command:

```bash
./main.py run gem5/multisim --length=quick
```
