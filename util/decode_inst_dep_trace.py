#!/usr/bin/env python2

# Copyright (c) 2013 - 2015 ARM Limited
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
# Copyright 2008 Google Inc.  All rights reserved.
# http://code.google.com/p/protobuf/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.
#     * Neither the name of Google Inc. nor the names of its
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
# Authors: Radhika Jagtap
#

# This script is used to dump protobuf traces of the instruction dependency
# graph to ASCII format.
#
# The ASCII trace format uses one line per instruction with the format
# instruction sequence number, (optional) pc, (optional) weight, type
# (optional) flags, (optional) phys addr, (optional) size, comp delay,
# (repeated) order dependencies comma-separated, and (repeated) register
# dependencies comma-separated.
#
# examples:
# seq_num,[pc],[weight,]type,[p_addr,size,flags,]comp_delay:[rob_dep]:
# [reg_dep]
# 1,35652,1,COMP,8500::
# 2,35656,1,COMP,0:,1:
# 3,35660,1,LOAD,1748752,4,74,500:,2:
# 4,35660,1,COMP,0:,3:
# 5,35664,1,COMP,3000::,4
# 6,35666,1,STORE,1748752,4,74,1000:,3:,4,5
# 7,35666,1,COMP,3000::,4
# 8,35670,1,STORE,1748748,4,74,0:,6,3:,7
# 9,35670,1,COMP,500::,7

import protolib
import sys

# Import the packet proto definitions. If they are not found, attempt
# to generate them automatically. This assumes that the script is
# executed from the gem5 root.
try:
    import inst_dep_record_pb2
except:
    print "Did not find proto definition, attempting to generate"
    from subprocess import call
    error = call(['protoc', '--python_out=util', '--proto_path=src/proto',
                  'src/proto/inst_dep_record.proto'])
    if not error:
        import inst_dep_record_pb2
        print "Generated proto definitions for instruction dependency record"
    else:
        print "Failed to import proto definitions"
        exit(-1)

def main():
    if len(sys.argv) != 3:
        print "Usage: ", sys.argv[0], " <protobuf input> <ASCII output>"
        exit(-1)

    # Open the file on read mode
    proto_in = protolib.openFileRd(sys.argv[1])

    try:
        ascii_out = open(sys.argv[2], 'w')
    except IOError:
        print "Failed to open ", sys.argv[2], " for writing"
        exit(-1)

    # Read the magic number in 4-byte Little Endian
    magic_number = proto_in.read(4)

    if magic_number != "gem5":
        print "Unrecognized file"
        exit(-1)

    print "Parsing packet header"

    # Add the packet header
    header = inst_dep_record_pb2.InstDepRecordHeader()
    protolib.decodeMessage(proto_in, header)

    print "Object id:", header.obj_id
    print "Tick frequency:", header.tick_freq

    print "Parsing packets"

    print "Creating enum value,name lookup from proto"
    enumNames = {}
    desc = inst_dep_record_pb2.InstDepRecord.DESCRIPTOR
    for namestr, valdesc in desc.enum_values_by_name.items():
        print '\t', valdesc.number, namestr
        enumNames[valdesc.number] = namestr

    num_packets = 0
    num_regdeps = 0
    num_robdeps = 0
    packet = inst_dep_record_pb2.InstDepRecord()

    # Decode the packet messages until we hit the end of the file
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1

        # Write to file the seq num
        ascii_out.write('%s' % (packet.seq_num))
        # Write to file the pc of the instruction, default is 0
        if packet.HasField('pc'):
            ascii_out.write(',%s' % (packet.pc))
        else:
            ascii_out.write(',0')
        # Write to file the weight, default is 1
        if packet.HasField('weight'):
            ascii_out.write(',%s' % (packet.weight))
        else:
            ascii_out.write(',1')
        # Write to file the type of the record
        try:
            ascii_out.write(',%s' % enumNames[packet.type])
        except KeyError:
            print "Seq. num", packet.seq_num, "has unsupported type", \
                packet.type
            exit(-1)


        # Write to file if it has the optional fields physical addr, size,
        # flags
        if packet.HasField('p_addr'):
            ascii_out.write(',%s' % (packet.p_addr))
        if packet.HasField('size'):
            ascii_out.write(',%s' % (packet.size))
        if packet.HasField('flags'):
            ascii_out.write(',%s' % (packet.flags))

        # Write to file the comp delay
        ascii_out.write(',%s' % (packet.comp_delay))

        # Write to file the repeated field order dependency
        ascii_out.write(':')
        if packet.rob_dep:
            num_robdeps += 1
            for dep in packet.rob_dep:
                ascii_out.write(',%s' % dep)
        # Write to file the repeated field register dependency
        ascii_out.write(':')
        if packet.reg_dep:
            num_regdeps += 1 # No. of packets with atleast 1 register dependency
            for dep in packet.reg_dep:
                ascii_out.write(',%s' % dep)
        # New line
        ascii_out.write('\n')

    print "Parsed packets:", num_packets
    print "Packets with at least 1 reg dep:", num_regdeps
    print "Packets with at least 1 rob dep:", num_robdeps

    # We're done
    ascii_out.close()
    proto_in.close()

if __name__ == "__main__":
    main()
