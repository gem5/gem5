# Traffic Generator

This tests the gem5 memory components with a simple traffic generator.
It also checks the correctness of the statistics outputted by gem5.

The trusted JSON files contain the deterministic fields from the exported
statistics. The time at which the output was created and the undefined
`logs` field of empty distributions are omitted. Dictionary entries are
compared recursively, so gem5 may add unrelated statistics or metadata without
invalidating the references. Lists are compared in order and must have the same
length; this ensures that statistics from every requested traffic-generator
core are checked.

To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/traffic_gen --length=[length]
```
