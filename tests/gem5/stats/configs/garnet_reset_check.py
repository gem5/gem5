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

"""Exercise grouped Garnet counters across a statistics reset."""

import runpy
import sys
from pathlib import Path

import m5

from _m5.stats import processDumpQueue

script = (
    Path(__file__).resolve().parents[4]
    / "configs/example/garnet_synth_traffic.py"
)
sys.path.insert(0, str(script.parent))
sys.argv = [
    str(script),
    "--protocol=Garnet_standalone",
    "--network=garnet",
    "--topology=Mesh_XY",
    "--mesh-rows=2",
    "--num-cpus=4",
    "--num-dirs=4",
    # The traffic generator compares this limit to curTick().
    "--sim-cycles=1000000",
    "--injectionrate=0.1",
    "--inj-vnet=0",
]
namespace = runpy.run_path(str(script), run_name="__m5_main__")
network = namespace["system"].ruby.network

# Collate router and link activity as well as directly updated counters.
m5.stats.dump()

counter_names = {
    "packets_injected",
    "flits_injected",
    "buffer_reads",
    "buffer_writes",
    "crossbar_activity",
    "flits_per_vnet",
}


def collect_counters(group, prefix="network"):
    values = {}
    for stat in group.getStats():
        if stat.name in counter_names:
            value = stat.value
            values[f"{prefix}.{stat.name}"] = (
                sum(value) if isinstance(value, list) else value
            )
    for name, child in group.getStatGroups().items():
        values.update(collect_counters(child, f"{prefix}.{name}"))
    return values


before = collect_counters(network.getCCObject())
for name in counter_names:
    assert any(
        key.endswith(f".{name}") and value > 0 for key, value in before.items()
    ), f"No traffic exercised {name}: {before}"

m5.stats.reset()
# Re-collate raw router activity to check its reset as well as the stats.
processDumpQueue()
m5.stats.prepare()
after = collect_counters(network.getCCObject())
assert before.keys() == after.keys()
assert all(value == 0 for value in after.values()), after
print("Garnet grouped statistics reset correctly.")
