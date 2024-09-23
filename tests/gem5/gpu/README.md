# GPU

These tests do random checks to the Ruby GPU protocol within gem5. Additionally, these tests run SE mode GPU tests.
To run these tests by themselves, you can run the following command in the **gem5** directory:

```bash
docker run -u $UID:$GID -v $(pwd):/gem5 -w /gem5/tests --rm -it ghcr.io/gem5/gcn-gpu:latest ./main.py run --host gcn_gpu gem5/gpu
```
