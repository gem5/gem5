# Copyright (c) 2018 ARM Limited
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

from m5.defines import buildEnv
from m5.SimObject import *

from m5.objects.BaseTrafficGen import *

class PyTrafficGen(BaseTrafficGen):
    type = 'PyTrafficGen'
    cxx_header = "cpu/testers/traffic_gen/pygen.hh"

    @cxxMethod
    def start(self, meta_generator):
        """
        Start generating traffic using the provided meta-generator. The
        meta-generator is an iterable Python object that describes a
        list of traffic generator instances.
        """
        pass

    cxx_exports = [
        PyBindMethod("createIdle"),
        PyBindMethod("createExit"),
        PyBindMethod("createLinear"),
        PyBindMethod("createRandom"),
        PyBindMethod("createDram"),
        PyBindMethod("createDramRot"),
    ]

    @cxxMethod(override=True)
    def createTrace(self, duration, trace_file, addr_offset=0):
        if buildEnv['HAVE_PROTOBUF']:
            return self.getCCObject().createTrace(duration, trace_file,
                                                  addr_offset=addr_offset)
        else:
            raise NotImplementedError("Trace playback requires that gem5 "
                                      "was built with protobuf support.")
