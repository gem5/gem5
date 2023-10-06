#!/usr/bin/env python3
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
# This script is used to migrate ASCII packet traces to the protobuf
# format currently used in gem5. It assumes that protoc has been
# executed and already generated the Python package for the packet
# messages. This can be done manually using:
# protoc --python_out=. --proto_path=src/proto src/proto/packet.proto
#
# The ASCII trace format uses one line per request on the format cmd,
# addr, size, tick. For example:
# r,128,64,4000
# w,232123,64,500000
# This trace reads 64 bytes from decimal address 128 at tick 4000,
# then writes 64 bytes to address 232123 at tick 500000.
#
# This script can of course also be used as a template to convert
# other trace formats into the gem5 protobuf format
import sys

import protolib

# Import the packet proto definitions. If they are not found, attempt
# to generate them automatically. This assumes that the script is
# executed from the gem5 root.
try:
    import packet_pb2
except:
    print("Did not find packet proto definitions, attempting to generate")
    from subprocess import call

    error = call(
        [
            "protoc",
            "--python_out=util",
            "--proto_path=src/proto",
            "src/proto/packet.proto",
        ],
    )
    if not error:
        print("Generated packet proto definitions")

        try:
            import google.protobuf
        except:
            print("Please install the Python protobuf module")
            exit(-1)

        import packet_pb2
    else:
        print("Failed to import packet proto definitions")
        exit(-1)


def main():
    if len(sys.argv) != 3:
        print("Usage: ", sys.argv[0], " <ASCII input> <protobuf output>")
        exit(-1)

    try:
        ascii_in = open(sys.argv[1], "r")
    except IOError:
        print("Failed to open ", sys.argv[1], " for reading")
        exit(-1)

    try:
        proto_out = open(sys.argv[2], "wb")
    except IOError:
        print("Failed to open ", sys.argv[2], " for writing")
        exit(-1)

    # Write the magic number in 4-byte Little Endian, similar to what
    # is done in src/proto/protoio.cc
    proto_out.write("gem5")

    # Add the packet header
    header = packet_pb2.PacketHeader()
    header.obj_id = "Converted ASCII trace " + sys.argv[1]
    # Assume the default tick rate
    header.tick_freq = 1000000000000
    protolib.encodeMessage(proto_out, header)

    # For each line in the ASCII trace, create a packet message and
    # write it to the encoded output
    for line in ascii_in:
        cmd, addr, size, tick = line.split(",")
        packet = packet_pb2.Packet()
        packet.tick = int(tick)
        # ReadReq is 1 and WriteReq is 4 in src/mem/packet.hh Command enum
        packet.cmd = 1 if cmd == "r" else 4
        packet.addr = int(addr)
        packet.size = int(size)
        protolib.encodeMessage(proto_out, packet)

    # We're done
    ascii_in.close()
    proto_out.close()


if __name__ == "__main__":
    main()
