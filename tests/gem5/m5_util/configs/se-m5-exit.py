# Copyright (c) 2021-2026 The Regents of the University of California
# Copyright (c) 2022 Google Inc
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

"""
A script used to test the m5_exit magic instruction events function correctly
for a given ISA.

Usage
-----

```sh
<gem5> test/gem5/m5_util/configs/se-m5-exit.py <isa> --resource-directory <path>
```

`<isas>` can be one of `x86`, `arm`, or `riscv`.


The `--resource-directory` argument is optional, and can be passed with a
<path> in order to specify where the downloaded resources should be cached.
If not set, the downloaded resources are cached to the default location for
gem5 binaries (typically "~/.cache/gem5/resources").
"""

import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import (
    ISA,
    get_isa_from_str,
)
from gem5.resources.resource import (
    BinaryResource,
    obtain_resource,
)
from gem5.simulate.simulator import Simulator

isa_resource_id_map = {
    ISA.X86: "x86-m5-exit",
    ISA.ARM: "arm-m5-exit",
    ISA.RISCV: "riscv-m5-exit",
}

resource_id_version_map = {
    "x86-m5-exit": "2.0.0",
    "arm-m5-exit": "1.0.0",
    "riscv-m5-exit": "1.0.0",
}

assert (
    len(
        {isa for isa in isa_resource_id_map.values()}
        - {isa for isa in resource_id_version_map.keys()}
    )
    == 0
), (
    "The following resource IDs  are needed by a supported ISA but have no "
    "version specified in `resource_id_version_map`:\n"
    f"{isa_resource_id_map.values() - resource_id_version_map.keys()}"
)

parser = argparse.ArgumentParser(
    description=(
        "A gem5 script for checking the m5_exit magic instructions, functions "
        "correctly in SE mode for a given ISA"
    )
)

parser.add_argument(
    "isa",
    type=str,
    choices={isa.value for isa in isa_resource_id_map.keys()},
    help="The ISA to run m5_exit on.",
)

parser.add_argument(
    "--resource-directory",
    type=str,
    required=False,
    help="The directory where downloaded resources are to be cached.",
)

args = parser.parse_args()

isa = get_isa_from_str(args.isa)


cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()
processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=isa,
    num_cores=1,
)

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

binary = obtain_resource(
    resource_id=isa_resource_id_map[isa],
    resource_version=resource_id_version_map[isa_resource_id_map[isa]],
    resource_directory=args.resource_directory,
)

assert isinstance(
    binary, BinaryResource
), "binary is not of type BinaryResource."

motherboard.set_se_binary_workload(binary)

simulator = Simulator(board=motherboard)
simulator.run()

print(f"Exit cause: '{simulator.get_last_exit_event_cause()}'.")
