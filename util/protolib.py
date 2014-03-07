#!/usr/bin/env python

# Copyright (c) 2013 ARM Limited
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
#          Radhika Jagtap

# This file is a library of commonly used functions used when interfacing
# with protobuf python messages. For eg, the decode scripts for different
# types of proto objects can use the same function to decode a single message

import struct

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

def EncodeVarint(out_file, value):
  """
  The encoding of the Varint32 is copied from
  google.protobuf.internal.encoder and is only repeated here to
  avoid depending on the internal functions in the library.
  """
  bits = value & 0x7f
  value >>= 7
  while value:
    out_file.write(struct.pack('<B', 0x80|bits))
    bits = value & 0x7f
    value >>= 7
  out_file.write(struct.pack('<B', bits))

def encodeMessage(out_file, message):
    """
    Encoded a message with the length prepended as a 32-bit varint.
    """
    out = message.SerializeToString()
    EncodeVarint(out_file, len(out))
    out_file.write(out)
