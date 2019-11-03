#!/usr/bin/env python2.7

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
# Authors: Ali Saidi
#          Andreas Hansson

# This script is used to dump protobuf instruction traces to ASCII
# format. It assumes that protoc has been executed and already
# generated the Python package for the inst messages. This can
# be done manually using:
# protoc --python_out=. inst.proto
# The ASCII trace format uses one line per request.

import protolib
import sys

# Import the packet proto definitions
try:
    import inst_pb2
except:
    print "Did not find protobuf inst definitions, attempting to generate"
    from subprocess import call
    error = call(['protoc', '--python_out=util', '--proto_path=src/proto',
                  'src/proto/inst.proto'])
    if not error:
        print "Generated inst proto definitions"

        try:
            import google.protobuf
        except:
            print "Please install Python protobuf module"
            exit(-1)

        import inst_pb2
    else:
        print "Failed to import inst proto definitions"
        exit(-1)

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

    print "Parsing instruction header"

    # Add the packet header
    header = inst_pb2.InstHeader()
    protolib.decodeMessage(proto_in, header)

    print "Object id:", header.obj_id
    print "Tick frequency:", header.tick_freq
    print "Memory addresses included:", header.has_mem

    if header.ver != 0:
        print "Warning: file version newer than decoder:", header.ver
        print "This decoder may not understand how to decode this file"


    print "Parsing instructions"

    num_insts = 0
    inst = inst_pb2.Inst()

    # Decode the inst messages until we hit the end of the file
    optional_fields = ('tick', 'type', 'inst_flags', 'addr', 'size', 'mem_flags')
    while protolib.decodeMessage(proto_in,  inst):
        # If we have a tick use it, otherwise count instructions
        if inst.HasField('tick'):
            tick = inst.tick
        else:
            tick = num_insts

        if inst.HasField('nodeid'):
            node_id = inst.nodeid
        else:
            node_id = 0;
        if inst.HasField('cpuid'):
            cpu_id = inst.cpuid
        else:
            cpu_id = 0;

        ascii_out.write('%-20d: (%03d/%03d) %#010x @ %#016x ' % (tick, node_id, cpu_id,
                                                  inst.inst, inst.pc))

        if inst.HasField('type'):
            ascii_out.write(' : %10s' % inst_pb2._INST_INSTTYPE.values_by_number[inst.type].name)

        for mem_acc in inst.mem_access:
            ascii_out.write(" %#x-%#x;" % (mem_acc.addr, mem_acc.addr + mem_acc.size))

        ascii_out.write('\n')
        num_insts += 1

    print "Parsed instructions:", num_insts

    # We're done
    ascii_out.close()
    proto_in.close()

if __name__ == "__main__":
    main()
