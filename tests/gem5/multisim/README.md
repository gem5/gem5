# Multisim

These tests check multisim in gem5.

To run the longer tests and processor switching tests,
use the following command:

```bash
./main.py run gem5/multisim --length=very-long
```

To run the varying length tests, use the following command:

```bash
./main.py run gem5/multisim --length=very-long --variant=fast
```

To run the tests for taking and restoring from checkpoints, use the following
command:

```bash
./main.py run gem5/multisim --length=quick
```
