# Copyright (c) 2015 The University of Illinois Urbana Champaign
# All rights reserved
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
# This is an example of an n port network switch to work in dist-gem5.
# Users can extend this to have different different topologies
import argparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath
from m5.util import fatal

addToPath("../")

from common import Simulation
from common import Options


def build_switch(args):
    # instantiate an EtherSwitch
    switch = EtherSwitch()
    # instantiate distEtherLinks to connect switch ports
    # to other gem5 instances
    switch.portlink = [
        DistEtherLink(
            speed=args.ethernet_linkspeed,
            delay=args.ethernet_linkdelay,
            dist_rank=args.dist_rank,
            dist_size=args.dist_size,
            server_name=args.dist_server_name,
            server_port=args.dist_server_port,
            sync_start=args.dist_sync_start,
            sync_repeat=args.dist_sync_repeat,
            is_switch=True,
            num_nodes=args.dist_size,
        )
        for i in range(args.dist_size)
    ]

    for (i, link) in enumerate(switch.portlink):
        link.int0 = switch.interface[i]

    return switch


def main():
    # Add options
    parser = argparse.ArgumentParser()
    Options.addCommonOptions(parser)
    Options.addFSOptions(parser)
    args = parser.parse_args()

    system = build_switch(args)
    root = Root(full_system=True, system=system)
    Simulation.run(args, root, None, None)


if __name__ == "__m5_main__":
    main()
