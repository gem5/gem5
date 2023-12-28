# KVM Fork Tests

These tests check that gem5 can fork with the KVM cpu, then switch to a different CPU.
To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/kvm_fork_tests --length=[length]
```
