# Traffic Generator

This tests the gem5 memory components with a simple traffic generator.
It also checks the correctness of the statistics outputted by gem5.

The processor is exported as a single statistics root. The trusted JSON files
check its simulation timing and the deterministic fields of each generator
under `cores.value[index].generator`, including single-core configurations.
The time at which the output was created and the undefined
`logs` field of empty distributions are omitted. Dictionary entries are
compared recursively, so gem5 may add unrelated statistics or metadata without
invalidating the references. Lists are compared in order and must have the same
length; this ensures that statistics from every requested traffic-generator
core are checked.

The MESITwoLevel and MIExample references use the corrected L1 cache indexing
from #3638: the first set-index bit is `log2(cache_line_size)` (6 for 64-byte
lines). Their timing, latency, and request counts reflect this indexing. These
references were captured by CI run 39322231942 at commit `55a47ed6dfdb` and
agree across both attempts of that run.

To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/traffic_gen --length=[length]
```
