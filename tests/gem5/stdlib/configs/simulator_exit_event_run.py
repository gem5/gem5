# Copyright (c) 2022 The Regents of the University of California
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
This script is used for testing the event_event handler of the Simulator
module. If the handler is working correctly the following output will be
received:

```
The program has started!
About to exit the simulation for the 1 st/nd/rd/th time
Handled exit event.
build/X86/sim/simulate.cc:194: info: Entering event queue @ 780559326.  Starting simulation...
About to exit the simulation for the 2 st/nd/rd/th time
Handled exit event.
build/X86/sim/simulate.cc:194: info: Entering event queue @ 854152659.  Starting simulation...
About to exit the simulation for the 3 st/nd/rd/th time
Handling the final exit event. We'll exit now.
```

By default a generator is passed to define the exit_event behavior. A list of
functions or a lone function can also be passed. This can be specified by the
`--exit-event-type` parameter.
"""

import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(
    description="A gem5 script for running simple binaries in SE mode."
)

parser.add_argument(
    "-e",
    "--exit-event-type",
    type=str,
    choices=("generator", "function-list", "function"),
    default="generator",
    help="Used to specify what exit event format is to be passed.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)


args = parser.parse_args()

# Setup the system.
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()

processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=1,
)

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload
# Note: Here we're using the "x86-m5-exit-repeat" resource. This calls an
# `m5_exit(0)` command in an infinite while-loop.
binary = obtain_resource(
    "x86-m5-exit-repeat",
    resource_directory=args.resource_directory,
    resource_version="1.0.0",
)
motherboard.set_se_binary_workload(binary)

# Create the exit event handler. Here there are three kinds: either pass a
# generator, a list of functions, or a lone function. In this script they all
# do the same thing for testing purposes.


def event_handle() -> bool:
    print("Handled exit event.")
    return False


def event_handle_final() -> bool:
    print("Handling the final exit event. We'll exit now.")
    return True


def generator():
    yield event_handle()
    yield event_handle()
    yield event_handle_final()


func_list = [event_handle, event_handle, event_handle_final]

i = 0


def lone_function() -> bool:
    global i
    i += 1
    if i < 3:
        return event_handle()
    return event_handle_final()


exit_event_handler = None
if args.exit_event_type == "function-list":
    exit_event_handler = func_list
elif args.exit_event_type == "generator":
    exit_event_handler = generator()
elif args.exit_event_type == "function":
    exit_event_handler = lone_function

assert exit_event_handler is not None

# Run the simulation
simulator = Simulator(
    board=motherboard,
    on_exit_event={
        ExitEvent.EXIT: exit_event_handler,
    },
)
simulator.run()
