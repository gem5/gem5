# Copyright (c) 2016 Georgia Institute of Technology
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
# LIMITED TO, THE IMPLIEpD WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class MyGarnetSyntheticTraffic(ClockedObject):
    type = "MyGarnetSyntheticTraffic"
    cxx_header = (
        "cpu/testers/garnet_synthetic_traffic/MyGarnetSyntheticTraffic.hh"
    )
    cxx_class = "gem5::MyGarnetSyntheticTraffic"

    block_offset = Param.Int(6, "block offset in bits")
    num_dest = Param.Int(1, "Number of Destinations")
    memory_size = Param.Int(65536, "memory size")
    sim_cycles = Param.Int(1000, "Number of simulation cycles")
    num_packets_max = Param.Int(
        -1,
        "Max number of packets to send. \
                        Default is to keep sending till simulation ends",
    )
    single_sender = Param.Int(
        -1,
        "Send only from this node. \
                                   By default every node sends",
    )
    single_dest = Param.Int(
        -1,
        "Send only to this dest. \
                                 Default depends on traffic_type",
    )
    traffic_type = Param.String("uniform_random", "Traffic type")
    inj_rate = Param.Float(0.1, "Packet injection rate")
    inj_vnet = Param.Int(
        -1,
        "Vnet to inject in. \
                              0 and 1 are 1-flit, 2 is 5-flit. \
                                Default is to inject in all three vnets",
    )
    precision = Param.Int(
        3,
        "Number of digits of precision \
                              after decimal point",
    )
    response_limit = Param.Cycles(
        5000000,
        "Cycles before exiting \
                                            due to lack of progress",
    )
    test = RequestPort("Port to the memory system to test")
    system = Param.System(Parent.any, "System we belong to")


class GarnetSyntheticTrafficV2(ClockedObject):
    """Here different seed is given to all cpus for random
    traffic generation"""

    type = "GarnetSyntheticTrafficV2"
    cxx_header = (
        "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTrafficV2.hh"
    )
    cxx_class = "gem5::GarnetSyntheticTrafficV2"

    block_offset = Param.Int(6, "block offset in bits")
    num_dest = Param.Int(1, "Number of Destinations")
    memory_size = Param.Int(65536, "memory size")
    sim_cycles = Param.Int(1000, "Number of simulation cycles")
    num_packets_max = Param.Int(
        -1,
        "Max number of packets to send. \
                        Default is to keep sending till simulation ends",
    )
    single_sender = Param.Int(
        -1,
        "Send only from this node. \
                                   By default every node sends",
    )
    single_dest = Param.Int(
        -1,
        "Send only to this dest. \
                                 Default depends on traffic_type",
    )
    traffic_type = Param.String("uniform_random", "Traffic type")
    inj_rate = Param.Float(0.1, "Packet injection rate")
    inj_vnet = Param.Int(
        -1,
        "Vnet to inject in. \
                              0 and 1 are 1-flit, 2 is 5-flit. \
                                Default is to inject in all three vnets",
    )
    precision = Param.Int(
        3,
        "Number of digits of precision \
                              after decimal point",
    )
    response_limit = Param.Cycles(
        5000000,
        "Cycles before exiting \
                                            due to lack of progress",
    )
    test = RequestPort("Port to the memory system to test")
    system = Param.System(Parent.any, "System we belong to")


class GarnetSyntheticTrafficV4(ClockedObject):
    """Here different seed is given to all cpus for random
    traffic generation"""

    type = "GarnetSyntheticTrafficV4"
    cxx_header = (
        "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTrafficV4.hh"
    )
    cxx_class = "gem5::GarnetSyntheticTrafficV4"

    block_offset = Param.Int(6, "block offset in bits")
    num_dest = Param.Int(1, "Number of Destinations")
    memory_size = Param.Int(65536, "memory size")
    sim_cycles = Param.Int(1000, "Number of simulation cycles")
    num_packets_max = Param.Int(
        -1,
        "Max number of packets to send. \
                        Default is to keep sending till simulation ends",
    )
    single_sender = Param.Int(
        -1,
        "Send only from this node. \
                                   By default every node sends",
    )
    single_dest = Param.Int(
        -1,
        "Send only to this dest. \
                                 Default depends on traffic_type",
    )
    traffic_type = Param.String("uniform_random", "Traffic type")
    # temp_traffic_type = Param.String("by_default", "Temporal Traffic type")
    temp_traffic_args = Param.String(
        "by_default", "Temporal Traffic arguments"
    )
    inj_rate = Param.Float(0.1, "Packet injection rate")
    inj_vnet = Param.Int(
        -1,
        "Vnet to inject in. \
                              0 and 1 are 1-flit, 2 is 5-flit. \
                                Default is to inject in all three vnets",
    )
    precision = Param.Int(
        3,
        "Number of digits of precision \
                              after decimal point",
    )
    response_limit = Param.Cycles(
        5000000,
        "Cycles before exiting \
                                            due to lack of progress",
    )
    test = RequestPort("Port to the memory system to test")
    system = Param.System(Parent.any, "System we belong to")
