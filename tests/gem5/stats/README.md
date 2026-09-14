# Stats

These test ensure the stats are output correctly.

1. "test_hdf5" - Test hdf5 output. Runs a simulation and ensures the hdf5
   output exists.
2. "test_simstats_output" - Tests the SimStat python module is parsing and
   outputting the stats correctly.

```bash
./main.py run gem5/stats --length=[length]
```

## Selecting objects with PyStats

`get_simstat(object)` returns one `SimStat` with that object's statistics and
children. Its JSON has no additional object-name wrapper. Selecting `Root`
continues to export the full simulation hierarchy.

Passing a Python list or `SimObjectVector` returns a list in the supplied
order. Empty and one-element collections remain lists, repeated objects are
retained, and nested collections preserve their structure. Each entry carries
its local `name` and the same simulation timing metadata. Names do not become
keys, so equally named objects cannot overwrite one another.

For example, after instantiating a processor with two cores:

```python
stats = get_simstat(root.processor.cores)
output = JsonOutputVistor(None).visit_simstat(stats)
assert isinstance(output, list)
assert output[0]["name"] == "cores0"
assert output[1]["name"] == "cores1"
```

This intentionally changes the historical multi-root return value from a
name-keyed `SimStat` to a list of snapshots. Callers selecting several objects
should iterate over the returned list. Single-object exports and vectors
encountered inside a single object's hierarchy retain their existing format.
The JSON visitor accepts either result and writes an object or array to match.

CSV collection exports use positional columns such as `0.count.value` and
`1.count.value`, with `0.name` and `1.name` identifying the selected objects.
Reordering a selection reorders those values and names on the next row.
Nested root lists use prefixes such as `0.1.count.value`. Single-object CSV
columns are unchanged. A changed collection column set or ambiguous flattened
column names raise `ValueError` before modifying an existing file; use a new
file for a different selection schema.

The quick `pystats-ordered-roots` case exercises a processor/core hierarchy,
colliding names, collections, and JSON/CSV dispatch through `m5.stats.dump`
and `global_dump_roots`. `tests/pyunit/pyunit_gem5stats_ordered.py` checks the
return shapes, complete JSON entries, preparation, and CSV behavior.
