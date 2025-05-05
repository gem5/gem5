# GPU

These tests:

1. Carry out randomly generated checks to the Ruby GPU protocol within gem5.
2. Run a set SE mode GPU simulations.

To run the SE mode by themselves, you must run within a `gcn-gpu` Docker
virtual environment. E.g. (running the command at the root of the gem5 repo):

```bash
docker run -u $UID:$GID -v $(pwd):/gem5 -w /gem5/tests --rm -it ghcr.io/gem5/gcn-gpu:latest ./main.py run --host gcn_gpu gem5/gpu
```
