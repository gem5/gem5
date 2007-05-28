# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
# Authors: Nathan Binkert

from m5 import build_env
from m5.params import *
from m5.proxy import *
from MemObject import MemObject

if build_env['FULL_SYSTEM']:
    from Device import BadAddr

class Bus(MemObject):
    type = 'Bus'
    port = VectorPort("vector port for connecting devices")
    bus_id = Param.Int(0, "blah")
    clock = Param.Clock("1GHz", "bus clock speed")
    width = Param.Int(64, "bus width (bytes)")
    responder_set = Param.Bool(False, "Did the user specify a default responder.")
    block_size = Param.Int(64, "The default block size if one isn't set by a device attached to the bus.")
    if build_env['FULL_SYSTEM']:
        responder = BadAddr(pio_addr=0x0, pio_latency="1ps")
        default = Port(Self.responder.pio, "Default port for requests that aren't handled by a device.")
    else:
        default = Port("Default port for requests that aren't handled by a device.")
