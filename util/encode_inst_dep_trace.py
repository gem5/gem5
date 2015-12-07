#!/usr/bin/env python

# Copyright (c) 2015 ARM Limited
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

# This script is used to dump ASCII traces of the instruction dependency
# graph to protobuf format.
#
# The ASCII trace format uses one line per instruction with the format
# instruction sequence number, (optional) pc, (optional) weight, type,
# (optional) flags, (optional) physical addr, (optional) size, comp delay,
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

DepRecord = inst_dep_record_pb2.InstDepRecord

def main():
    if len(sys.argv) != 3:
        print "Usage: ", sys.argv[0], " <ASCII input> <protobuf output>"
        exit(-1)

    # Open the file in write mode
    proto_out = open(sys.argv[2], 'wb')

    # Open the file in read mode
    try:
        ascii_in = open(sys.argv[1], 'r')
    except IOError:
        print "Failed to open ", sys.argv[1], " for reading"
        exit(-1)

    # Write the magic number in 4-byte Little Endian, similar to what
    # is done in src/proto/protoio.cc
    proto_out.write("gem5")

    # Add the packet header
    header = inst_dep_record_pb2.InstDepRecordHeader()
    header.obj_id = "Converted ASCII trace " + sys.argv[1]
    # Assume the default tick rate
    header.tick_freq = 1000000000
    header.window_size = 120
    protolib.encodeMessage(proto_out, header)

    print "Creating enum name,value lookup from proto"
    enumValues = {}
    for namestr, valdesc in DepRecord.DESCRIPTOR.enum_values_by_name.items():
        print '\t', namestr, valdesc.number
        enumValues[namestr] = valdesc.number

    num_records = 0
    # For each line in the ASCII trace, create a packet message and
    # write it to the encoded output
    for line in ascii_in:
        inst_info_str, rob_dep_str, reg_dep_str = (line.strip()).split(':')
        inst_info_list = inst_info_str.split(',')
        dep_record = DepRecord()

        dep_record.seq_num = long(inst_info_list[0])
        dep_record.pc = long(inst_info_list[1])
        dep_record.weight = long(inst_info_list[2])
        # If the type is not one of the enum values, it should be a key error
        try:
            dep_record.type = enumValues[inst_info_list[3]]
        except KeyError:
            print "Seq. num", dep_record.seq_num, "has unsupported type", \
                inst_info_list[3]
            exit(-1)

        if dep_record.type == DepRecord.INVALID:
            print "Seq. num", dep_record.seq_num, "is of INVALID type"
            exit(-1)

        # If the instruction is a load or store record the physical addr,
        # size flags in addition to recording the computation delay
        if dep_record.type in [DepRecord.LOAD, DepRecord.STORE]:
            p_addr, size, flags, comp_delay = inst_info_list[4:8]
            dep_record.p_addr = long(p_addr)
            dep_record.size = int(size)
            dep_record.flags = int(flags)
            dep_record.comp_delay = long(comp_delay)
        else:
            comp_delay = inst_info_list[4]
            dep_record.comp_delay = long(comp_delay)

        # Parse the register and order dependencies both of which are
        # repeated fields. An empty list is valid.
        rob_deps = rob_dep_str.strip().split(',')
        for a_dep in rob_deps:
            # if the string is empty, split(',') returns 1 item: ''
            # if the string is ",4", split(',') returns 2 items: '', '4'
            # long('') gives error, so check if the item is non-empty
            if a_dep:
                dep_record.rob_dep.append(long(a_dep))

        reg_deps = reg_dep_str.split(',')
        for a_dep in reg_deps:
            if a_dep:
                dep_record.reg_dep.append(long(a_dep))

        protolib.encodeMessage(proto_out, dep_record)
        num_records += 1

    print "Converted", num_records, "records."
    # We're done
    ascii_in.close()
    proto_out.close()

if __name__ == "__main__":
    main()
