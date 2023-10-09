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
import argparse
from importlib.machinery import SourceFileLoader

import m5
from cache_hierarchies import ModMIExampleCacheHierarchy
from gem5.components.boards.test_board import TestBoard
from gem5.components.memory.simple import SingleChannelSimpleMemory
from gem5.components.processors.complex_generator import ComplexGenerator
from m5.debug import flags
from m5.objects import Root

argparser = argparse.ArgumentParser()

argparser.add_argument(
    "config_name",
    type=str,
    help="Name of the python file "
    "including the defintion of a python generator and "
    "importing the right replacement policy. The python "
    "generator should only assume one positional argument "
    "and be named python_generator. The replacement policy"
    " should be imported as rp.",
)
argparser.add_argument(
    "config_path",
    type=str,
    help="Path to the python file" "specified by config_name.",
)

args = argparser.parse_args()

module = SourceFileLoader(args.config_name, args.config_path).load_module()
python_generator = module.python_generator
rp_class = module.rp

flags["RubyHitMiss"].enable()

cache_hierarchy = ModMIExampleCacheHierarchy(rp_class)

memory = SingleChannelSimpleMemory(
    latency="30ns",
    latency_var="0ns",
    bandwidth="12.8GiB/s",
    size="512MiB",
)

generator = ComplexGenerator()
generator.set_traffic_from_python_generator(python_generator)

# We use the Test Board. This is a special board to run traffic generation
# tasks
motherboard = TestBoard(
    clk_freq="1GHz",
    generator=generator,  # We pass the traffic generator as the processor.
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)
root = Root(full_system=False, system=motherboard)

motherboard._pre_instantiate()
m5.instantiate()

generator.start_traffic()
print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}.")
