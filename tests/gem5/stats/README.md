# Stats

These test ensure the stats are output correctly.

1. "test_hdf5" - Test hdf5 output. Runs a simulation and ensures the hdf5
   output exists.
2. "test_simstats_output" - Tests the SimStat python module is parsing and
   outputting the stats correctly.

```bash
./main.py run gem5/stats --length=[length]
```

## Single-root PyStats snapshots

`get_simstat(root)` accepts one `SimObject` and returns one `SimStat` for
that object and its descendants. Passing the simulation's `Root` continues
to export the full hierarchy, including any nested `SimObjectVector`s.

Lists and `SimObjectVector`s passed directly to `get_simstat` now raise
`TypeError`, including empty and singleton collections. To select multiple
objects, create and visit each snapshot separately:

```python
from m5.stats.gem5stats import JsonOutputVistor, get_simstat

snapshots = [get_simstat(core) for core in root.processor.cores]
visitor = JsonOutputVistor(None)
output = [visitor.visit_simstat(snapshot) for snapshot in snapshots]
```

Each snapshot has its own namespace, so equally named objects do not
overwrite one another. Callers can also visit individual snapshots with
different visitors or retain them as Python objects.

The existing `m5.stats.dump(roots=...)` and `global_dump_roots` interfaces
still support multiple selected subtrees. JSON and CSV dump visitors call
`get_simstat` separately for each selected object, preserving input order,
repeated objects, and duplicate local names. They accept only flat
lists/vectors of `SimObject`s, not nested collections.

Collection dumps use a JSON array and positional CSV columns such as
`0.count.value` and `1.count.value`; each entry also retains its local name.
Explicit singleton collections remain collections instead of being
unwrapped. Direct single-object exports and default full-root dumps retain
their existing formats. An empty collection passed directly to a visitor
produces an empty JSON array or an empty CSV row. As before, an empty
selection in `m5.stats.dump` falls back to the full simulation root when
`global_dump_roots` is also empty.

CSV dumps reject ambiguous flattened column names. When a collection is
involved, they also reject a changed column set before appending a row;
use a separate output file for a different selection schema.

`pystats_single_root_check.py` covers the single-root contract, vectors
inside a root, independent snapshots, and JSON/CSV dumps of selected roots.
It needs a binary built with `USE_TEST_OBJECTS=y`, as provided by TestLib:

```bash
scons build/NULL/gem5.opt USE_TEST_OBJECTS=y -j4
build/NULL/gem5.opt tests/gem5/stats/configs/pystats_single_root_check.py
```
