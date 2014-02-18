#!/usr/bin/env python

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
# Authors: Andreas Hansson
#

# This script is used to dump protobuf packet traces to ASCII
# format. It assumes that protoc has been executed and already
# generated the Python package for the packet messages. This can
# be done manually using:
# protoc --python_out=. --proto_path=src/proto src/proto/packet.proto
#
# The ASCII trace format uses one line per request on the format cmd,
# addr, size, tick,flags. For example:
# r,128,64,4000,0
# w,232123,64,500000,0

import struct
import sys

# Import the packet proto definitions. If they are not found, attempt
# to generate them automatically. This assumes that the script is
# executed from the gem5 root.
try:
    import packet_pb2
except:
    print "Did not find packet proto definitions, attempting to generate"
    from subprocess import call
    error = call(['protoc', '--python_out=util', '--proto_path=src/proto',
                  'src/proto/packet.proto'])
    if not error:
        print "Generated packet proto definitions"

        try:
            import google.protobuf
        except:
            print "Please install Python protobuf module"
            exit(-1)

        import packet_pb2
    else:
        print "Failed to import packet proto definitions"
        exit(-1)

def DecodeVarint(in_file):
    """
    The decoding of the Varint32 is copied from
    google.protobuf.internal.decoder and is only repeated here to
    avoid depending on the internal functions in the library. If the
    end of file is reached, return (0, 0).
    """
    result = 0
    shift = 0
    pos = 0
    # Use a 32-bit mask
    mask = 0xffffffff
    while 1:
        c = in_file.read(1)
        if len(c) == 0:
            return (0, 0)
        b = struct.unpack('<B', c)[0]
        result |= ((b & 0x7f) << shift)
        pos += 1
        if not (b & 0x80):
            if result > 0x7fffffffffffffff:
                result -= (1 << 64)
                result |= ~mask
            else:
                result &= mask
                return (result, pos)
            shift += 7
            if shift >= 64:
                raise IOError('Too many bytes when decoding varint.')

def decodeMessage(in_file, message):
    """
    Attempt to read a message from the file and decode it. Return
    False if no message could be read.
    """
    try:
        size, pos = DecodeVarint(in_file)
        if size == 0:
            return False
        buf = in_file.read(size)
        message.ParseFromString(buf)
        return True
    except IOError:
        return False

def main():
    if len(sys.argv) != 3:
        print "Usage: ", sys.argv[0], " <protobuf input> <ASCII output>"
        exit(-1)

    try:
       proto_in = open(sys.argv[1], 'rb')
    except IOError:
        print "Failed to open ", sys.argv[1], " for reading"
        exit(-1)

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
    header = packet_pb2.PacketHeader()
    decodeMessage(proto_in, header)

    print "Object id:", header.obj_id
    print "Tick frequency:", header.tick_freq

    print "Parsing packets"

    num_packets = 0
    ignored_flags = False
    packet = packet_pb2.Packet()

    # Decode the packet messages until we hit the end of the file
    while decodeMessage(proto_in, packet):
        num_packets += 1
        # ReadReq is 1 and WriteReq is 4 in src/mem/packet.hh Command enum
        cmd = 'r' if packet.cmd == 1 else ('w' if packet.cmd == 4 else 'u')
        if packet.HasField('flags'):
            # Currently not printing flags
            ignored_flags = True
        ascii_out.write('%s,%s,%s,%s\n' % (cmd, packet.addr, packet.size,
                                           packet.tick))

    print "Parsed packets:", num_packets
    if ignored_flags:
        print "Encountered packet flags that were ignored"

    # We're done
    ascii_out.close()
    proto_in.close()

if __name__ == "__main__":
    main()
