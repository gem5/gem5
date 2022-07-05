# Copyright (c) 2021 The Regents of the University of California
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
This tests the gem5 memory components with a simple traffic generator.

TODO: At present all the Single Channel memory components are tested. This
      should be expanded to included DRAMSIM3 memory systems.
"""

from testlib import *


def test_memory(
    generator: str,
    generator_cores: str,
    cache: str,
    module: str,
    memory: str,
    *args,
) -> None:
    protocol_map = {
        "NoCache": None,
        "PrivateL1": None,
        "PrivateL1PrivateL2": None,
        "MESITwoLevel": "MESI_Two_Level",
    }
    tag_map = {
        "NoCache": constants.quick_tag,
        "PrivateL1": constants.quick_tag,
        "PrivateL1PrivateL2": constants.quick_tag,
        "MESITwoLevel": constants.long_tag,
    }

    name = (
        "test-memory-"
        + f"{generator}-{generator_cores}-{cache}-{module}-{memory}"
    )
    for arg in args:
        name += "-" + arg

    gem5_verify_config(
        name=name,
        fixtures=(),
        verifiers=(),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "traffic_gen",
            "simple_traffic_run.py",
        ),
        config_args=[generator, generator_cores, cache, module, memory]
        + list(args),
        valid_isas=(constants.null_tag,),
        protocol=protocol_map[cache],
        valid_hosts=constants.supported_hosts,
        length=tag_map[cache],
    )


cache_classes = ["NoCache", "PrivateL1", "PrivateL1PrivateL2", "MESITwoLevel"]
memory_classes = [
    "SingleChannelDDR3_1600",
    "SingleChannelDDR3_2133",
    "SingleChannelDDR4_2400",
    "SingleChannelLPDDR3_1600",
    "SingleChannelHBM",
    "DualChannelDDR3_1600",
    "DualChannelDDR3_2133",
    "DualChannelDDR4_2400",
    "DualChannelLPDDR3_1600",
    "HBM2Stack",
]


def create_single_core_tests(module, memory_classes):
    generator_classes = ["LinearGenerator", "RandomGenerator", "GUPSGenerator"]
    for generator_class in generator_classes:
        for cache_class in cache_classes:
            for memory_class in memory_classes:
                test_memory(
                    generator_class,
                    "1",
                    cache_class,
                    module,
                    memory_class,
                    "512MiB",
                )


def create_dual_core_tests(module, memory_classes):
    generator_classes = ["GUPSGeneratorEP", "GUPSGeneratorPAR"]
    for generator_class in generator_classes:
        for cache_class in cache_classes:
            for memory_class in memory_classes:
                test_memory(
                    generator_class,
                    "2",
                    cache_class,
                    module,
                    memory_class,
                    "512MiB",
                )


create_single_core_tests("gem5.components.memory", memory_classes)
create_dual_core_tests("gem5.components.memory", memory_classes)
