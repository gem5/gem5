#!/usr/bin/env python2

# Copyright (c) 2013-2014 ARM Limited
# All rights reserved
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
#
# Authors: Andreas Hansson

# This script is used to dump protobuf packet traces to ASCII
# format.

import os
import protolib
import subprocess
import sys

util_dir = os.path.dirname(os.path.realpath(__file__))
# Make sure the proto definitions are up to date.
subprocess.check_call(['make', '--quiet', '-C', util_dir, 'packet_pb2.py'])
import packet_pb2

def main():
    if len(sys.argv) != 3:
        print "Usage: ", sys.argv[0], " <protobuf input> <ASCII output>"
        exit(-1)

    # Open the file in read mode
    proto_in = protolib.openFileRd(sys.argv[1])

    try:
        ascii_out = open(sys.argv[2], 'w')
    except IOError:
        print "Failed to open ", sys.argv[2], " for writing"
        exit(-1)

    # Read the magic number in 4-byte Little Endian
    magic_number = proto_in.read(4)

    if magic_number != "gem5":
        print "Unrecognized file", sys.argv[1]
        exit(-1)

    print "Parsing packet header"

    # Add the packet header
    header = packet_pb2.PacketHeader()
    protolib.decodeMessage(proto_in, header)

    print "Object id:", header.obj_id
    print "Tick frequency:", header.tick_freq

    for id_string in header.id_strings:
        print 'Master id %d: %s' % (id_string.key, id_string.value)

    print "Parsing packets"

    num_packets = 0
    packet = packet_pb2.Packet()

    # Decode the packet messages until we hit the end of the file
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1
        # ReadReq is 1 and WriteReq is 4 in src/mem/packet.hh Command enum
        cmd = 'r' if packet.cmd == 1 else ('w' if packet.cmd == 4 else 'u')
        if packet.HasField('pkt_id'):
            ascii_out.write('%s,' % (packet.pkt_id))
        if packet.HasField('flags'):
            ascii_out.write('%s,%s,%s,%s,%s' % (cmd, packet.addr, packet.size,
                            packet.flags, packet.tick))
        else:
            ascii_out.write('%s,%s,%s,%s' % (cmd, packet.addr, packet.size,
                                           packet.tick))
        if packet.HasField('pc'):
            ascii_out.write(',%s\n' % (packet.pc))
        else:
            ascii_out.write('\n')

    print "Parsed packets:", num_packets

    # We're done
    ascii_out.close()
    proto_in.close()

if __name__ == "__main__":
    main()
