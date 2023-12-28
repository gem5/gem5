# Checkpoint tests

These tests run hello world binary for arm, x86, and power isa and ubuntuboot workload for x86 isa using checkpoints.
Each binary is run in two parts:
- Save checkpoint: A binary is run for a set amount of ticks and then a checkpoint is taken. This test checks if the checkpoint is taken.

- Resotre checkpoint: The same binary and board in the respective save test are used with the saved checkpoint (the checkpoint is uploaded to gem5 resources). This test checks if the binary ran properly.

```bash
./main.py run gem5/checkpoint_tests/
```
