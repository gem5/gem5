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

`get_simstat` accepts a `SimObject`, `SimObjectVector`, or Python list.
Vectors always return lists in member order, including empty and one-element
vectors. Repeated objects are retained. Each vector snapshot carries its
local `name` and the same simulation timing metadata. Names do not become
keys, so equally named objects cannot overwrite one another.

For example, after instantiating a processor with two cores:

```python
stats = get_simstat(root.processor.cores)
output = JsonOutputVistor(None).visit_simstat(stats)
assert isinstance(output, list)
assert output[0]["name"] == "cores0"
assert output[1]["name"] == "cores1"
```

Python lists are handled by recursively calling `get_simstat` for each
element, forwarding `prepare_stats`. The returned list preserves input order,
repeated entries, nesting, and empty or singleton lists. Each recursive call
performs its own preparation (unless disabled) and records its own snapshot
metadata. For example:

```python
snapshots = get_simstat([first, second])
# Equivalent to [get_simstat(first), get_simstat(second)].
output = JsonOutputVistor(None).visit_simstat(snapshots)
```

Input shape determines output shape: a SimObject produces a JSON object,
and a vector or Python list produces a JSON array. This intentionally changes
historical singleton-collection output. Call `get_simstat(obj)` instead of
`get_simstat([obj])` or `get_simstat(singleton_vector)` to retain the
single-object shape (using the vector's member in the latter case).

The existing dump API still accepts flat lists: `m5.stats.dump(roots=[...])`,
`global_dump_roots`, and the JSON/CSV visitors convert each selected SimObject
separately. Every collection selection now produces a JSON array, including a
single selected object. Preparation stays in the dump caller; each independent
conversion records its creation time. The dump API continues to require flat
selections. Default full-root dumps, direct single-object exports, and vectors
encountered inside one object's hierarchy keep their existing format.

CSV vector and multi-object dump exports use positional columns such as
`0.count.value` and `1.count.value`, with `0.name` and `1.name` identifying
entries. Reordering a selection reorders those values and names on the next
row. Explicitly visiting nested snapshots uses prefixes such as
`0.1.count.value`. Direct single-object CSV columns are unchanged; singleton
collections now use indexed columns such as `0.count.value`. A changed
collection column set or ambiguous flattened column names raise `ValueError`
before modifying an existing file; use a new file for a different schema.

The quick `pystats-ordered-roots` case exercises a processor/core hierarchy,
recursive list conversion, and JSON/CSV dispatch through
`m5.stats.dump` and `global_dump_roots`.
`tests/pyunit/pyunit_gem5stats_ordered.py` checks the object/vector API,
recursive lists, complete JSON entries, preparation, and container shapes
through the existing dump selection interface.
