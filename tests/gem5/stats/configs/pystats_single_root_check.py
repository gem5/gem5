# Copyright (c) 2026 The Regents of The University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Export a processor and selected cores through the public stats APIs."""

import csv
import json
from pathlib import Path

import m5
from m5.objects import (
    Root,
    ScalarStatTester,
    SubSystem,
)
from m5.params import SimObjectVector
from m5.stats.gem5stats import (
    CsvOutputVisitor,
    JsonOutputVistor,
    get_simstat,
)

root = Root(full_system=False)
root.processor = SubSystem()
root.processor.cores = [SubSystem(), SubSystem()]
root.processor.cores[0].generator = ScalarStatTester(name="count", value=11)
root.processor.cores[1].generator = ScalarStatTester(name="count", value=22)
first, second = root.processor.cores

m5.instantiate()
m5.simulate(1)

visitor = JsonOutputVistor(None)
# Single objects keep their existing structure, including nested vectors.
whole = visitor.visit_simstat(get_simstat(root))
processor = visitor.visit_simstat(get_simstat(root.processor))
for output in (whole["processor"], processor):
    assert [
        core["generator"]["count"]["value"]
        for core in output["cores"]["value"]
    ] == [11.0, 22.0]

# Each selected core owns its own snapshot and can be visited independently.
cores = [get_simstat(core) for core in root.processor.cores]
output = [visitor.visit_simstat(core) for core in cores]
assert [core["name"] for core in output] == ["cores0", "cores1"]
assert [core["generator"]["count"]["value"] for core in output] == [11.0, 22.0]

# Same-named roots and repeated selections stay in separate namespaces.
for objects, values in (
    ([], []),
    ([first.generator], [11.0]),
    ([first.generator, second.generator], [11.0, 22.0]),
    ([second.generator, first.generator], [22.0, 11.0]),
    ([first.generator, first.generator], [11.0, 11.0]),
):
    snapshots = [get_simstat(obj) for obj in objects]
    output = [visitor.visit_simstat(snapshot) for snapshot in snapshots]
    assert [entry["count"]["value"] for entry in output] == values
    assert [entry["name"] for entry in output] == ["generator"] * len(values)

# A collection is never accepted as a single root, even if it is empty or
# contains only one object. Vectors inside a root remain supported above.
for objects in (
    [],
    [first],
    [first, second],
    SimObjectVector([]),
    SimObjectVector([first]),
    root.processor.cores,
):
    try:
        get_simstat(objects)
    except TypeError as error:
        assert "single SimObject" in str(error)
    else:
        raise AssertionError("get_simstat accepted a collection of roots")

outdir = Path(m5.options.outdir)
json_path = outdir / "selected.json"
csv_path = outdir / "selected.csv"
json_visitor = JsonOutputVistor(json_path)
csv_visitor = CsvOutputVisitor(csv_path)
# Exercise the same dispatch used by --stats-root and global_dump_roots.
m5.stats.outputList = [json_visitor, csv_visitor]
m5.stats.dump(roots=[first.generator, second.generator])
output = json.loads(json_path.read_text())
assert [entry["count"]["value"] for entry in output] == [11.0, 22.0]
m5.stats.global_dump_roots = [second.generator, first.generator]
m5.stats.dump()
output = json.loads(json_path.read_text())
assert [entry["count"]["value"] for entry in output] == [22.0, 11.0]
with csv_path.open() as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 2
assert [float(row["0.count.value"]) for row in rows] == [11.0, 22.0]
assert [float(row["1.count.value"]) for row in rows] == [22.0, 11.0]
assert all(row["0.name"] == row["1.name"] == "generator" for row in rows)

# Singleton dump selections also retain their array and indexed columns.
single_csv_path = outdir / "singleton.csv"
m5.stats.global_dump_roots = []
m5.stats.outputList = [json_visitor, CsvOutputVisitor(single_csv_path)]
m5.stats.dump(roots=[first.generator])
output = json.loads(json_path.read_text())
assert isinstance(output, list) and len(output) == 1
assert output[0]["count"]["value"] == 11.0
assert output[0]["name"] == "generator"
m5.stats.global_dump_roots = [first.generator]
m5.stats.dump()
assert isinstance(json.loads(json_path.read_text()), list)
with single_csv_path.open() as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 2
assert all(float(row["0.count.value"]) == 11.0 for row in rows)
assert all("count.value" not in row for row in rows)
m5.stats.global_dump_roots = []

# A direct vector dump still exports its members as separate snapshots.
vector_csv_path = outdir / "vector.csv"
m5.stats.outputList = [json_visitor, CsvOutputVisitor(vector_csv_path)]
m5.stats.dump(roots=root.processor.cores)
output = json.loads(json_path.read_text())
assert [core["generator"]["count"]["value"] for core in output] == [11.0, 22.0]
with vector_csv_path.open() as stream:
    row = next(csv.DictReader(stream))
assert float(row["0.generator.count.value"]) == 11.0
assert float(row["1.generator.count.value"]) == 22.0

# The default whole-root dump keeps its existing object representation.
m5.stats.outputList = [json_visitor]
m5.simulate(1)
m5.stats.dump()
output = json.loads(json_path.read_text())
assert isinstance(output, dict)
assert (
    output["processor"]["cores"]["value"][0]["generator"]["count"]["value"]
    == 11.0
)
