# -*- mode:python -*-

# Copyright (c) 2014 ARM Limited
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
#
# Authors: Andreas Sandberg

from m5.params import *
from m5.proxy import *
from VirtIO import VirtIODeviceBase

class VirtIO9PBase(VirtIODeviceBase):
    type = 'VirtIO9PBase'
    abstract = True
    cxx_header = 'dev/virtio/fs9p.hh'

    queueSize = Param.Unsigned(32, "Output queue size (pages)")
    tag = Param.String("gem5", "Mount tag")


class VirtIO9PProxy(VirtIO9PBase):
    type = 'VirtIO9PProxy'
    abstract = True
    cxx_header = 'dev/virtio/fs9p.hh'

class VirtIO9PDiod(VirtIO9PProxy):
    type = 'VirtIO9PDiod'
    cxx_header = 'dev/virtio/fs9p.hh'

    diod = Param.String("/usr/sbin/diod", "Path to diod")
    root = Param.String("/tmp", "Path to export through diod")

class VirtIO9PSocket(VirtIO9PProxy):
    type = 'VirtIO9PSocket'
    cxx_header = 'dev/virtio/fs9p.hh'

    server = Param.String("127.0.0.1", "9P server address or host name")
    port = Param.String("564", "9P server port")
