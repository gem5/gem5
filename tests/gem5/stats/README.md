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

`get_simstat` accepts only a `SimObject` or `SimObjectVector`. A vector with
one element returns that object's snapshot for compatibility. Larger vectors
return lists in member order, retaining repeated objects, while empty vectors
return empty lists. Each snapshot carries its local `name` and the same
simulation timing metadata. Names do not become keys, so equally named
objects cannot overwrite one another.

For example, after instantiating a processor with two cores:

```python
stats = get_simstat(root.processor.cores)
output = JsonOutputVistor(None).visit_simstat(stats)
assert isinstance(output, list)
assert output[0]["name"] == "cores0"
assert output[1]["name"] == "cores1"
```

Ordinary Python lists, including singleton lists, are rejected by
`get_simstat`. Replace `get_simstat([obj])` with `get_simstat(obj)`. For an
arbitrary selection, collect snapshots explicitly:

```python
snapshots = [get_simstat(obj) for obj in selected_objects]
output = JsonOutputVistor(None).visit_simstat(snapshots)
```

The existing dump API still accepts flat lists: `m5.stats.dump(roots=[...])`,
`global_dump_roots`, and the JSON/CSV visitors convert each selected SimObject
separately. A single selected object retains its old JSON object and CSV
column format. Larger selections produce JSON arrays. Preparation stays in
the dump caller; each independent conversion records its creation time.
Nested root selections are not supported. Single-object exports and vectors
encountered inside one object's hierarchy keep their existing format.

CSV vector and multi-object dump exports use positional columns such as
`0.count.value` and `1.count.value`, with `0.name` and `1.name` identifying
entries. Reordering a selection reorders those values and names on the next
row. Single-object and singleton-vector CSV columns are unchanged. A changed
collection column set or ambiguous flattened column names raise `ValueError`
before modifying an existing file; use a new file for a different schema.

The quick `pystats-ordered-roots` case exercises a processor/core hierarchy,
explicit per-object conversion, and JSON/CSV dispatch through
`m5.stats.dump` and `global_dump_roots`.
`tests/pyunit/pyunit_gem5stats_ordered.py` checks the object/vector API,
rejection of ordinary and nested lists, complete JSON entries, preparation,
and compatibility of the existing dump selection interface.
