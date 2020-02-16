# Copyright (c) 2016-2017 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

# This configuration file extends the example ARM big.LITTLE(tm)
# configuration to enabe dist-gem5 siulations of big.LITTLE systems.

from __future__ import print_function
from __future__ import absolute_import

import argparse
import os

import m5
from m5.objects import *

import fs_bigLITTLE as bL
m5.util.addToPath("../../dist")
import sw


def addOptions(parser):
   # Options for distributed simulation (i.e. dist-gem5)
    parser.add_argument("--dist", action="store_true", help="Distributed gem5"\
                      " simulation.")
    parser.add_argument("--is-switch", action="store_true",
                        help="Select the network switch simulator process for"\
                      " a distributed gem5 run.")
    parser.add_argument("--dist-rank", default=0, action="store", type=int,
                      help="Rank of this system within the dist gem5 run.")
    parser.add_argument("--dist-size", default=0, action="store", type=int,
                      help="Number of gem5 processes within the dist gem5"\
                      " run.")
    parser.add_argument("--dist-server-name",
                      default="127.0.0.1",
                      action="store", type=str,
                      help="Name of the message server host\nDEFAULT:"\
                      " localhost")
    parser.add_argument("--dist-server-port",
                      default=2200,
                      action="store", type=int,
                      help="Message server listen port\nDEFAULT: 2200")
    parser.add_argument("--dist-sync-repeat",
                      default="0us",
                      action="store", type=str,
                      help="Repeat interval for synchronisation barriers"\
                      " among dist-gem5 processes\nDEFAULT:"\
                      " --ethernet-linkdelay")
    parser.add_argument("--dist-sync-start",
                      default="1000000000000t",
                      action="store", type=str,
                      help="Time to schedule the first dist synchronisation"\
                      " barrier\nDEFAULT:1000000000000t")
    parser.add_argument("--ethernet-linkspeed", default="10Gbps",
                        action="store", type=str,
                        help="Link speed in bps\nDEFAULT: 10Gbps")
    parser.add_argument("--ethernet-linkdelay", default="10us",
                      action="store", type=str,
                      help="Link delay in seconds\nDEFAULT: 10us")
    parser.add_argument("--etherdump", action="store", type=str, default="",
                        help="Specify the filename to dump a pcap capture of"\
                        " the ethernet traffic")
    # Used by util/dist/gem5-dist.sh
    parser.add_argument("--checkpoint-dir", type=str,
                        default=m5.options.outdir,
                        help="Directory to save/read checkpoints")


def addEthernet(system, options):
    # create NIC
    dev = IGbE_e1000()
    system.attach_pci(dev)
    system.ethernet = dev

    # create distributed ethernet link
    system.etherlink = DistEtherLink(speed = options.ethernet_linkspeed,
                                     delay = options.ethernet_linkdelay,
                                     dist_rank = options.dist_rank,
                                     dist_size = options.dist_size,
                                     server_name = options.dist_server_name,
                                     server_port = options.dist_server_port,
                                     sync_start = options.dist_sync_start,
                                     sync_repeat = options.dist_sync_repeat)
    system.etherlink.int0 = Parent.system.ethernet.interface
    if options.etherdump:
        system.etherdump = EtherDump(file=options.etherdump)
        system.etherlink.dump = system.etherdump


def main():
    parser = argparse.ArgumentParser(
        description="Generic ARM big.LITTLE configuration with "\
        "dist-gem5 support")
    bL.addOptions(parser)
    addOptions(parser)
    options = parser.parse_args()

    if options.is_switch:
        root = Root(full_system = True,
                    system = sw.build_switch(options))
    else:
        root = bL.build(options)
        addEthernet(root.system, options)

    bL.instantiate(options, checkpoint_dir=options.checkpoint_dir)
    bL.run(options.checkpoint_dir)


if __name__ == "__m5_main__":
    main()
