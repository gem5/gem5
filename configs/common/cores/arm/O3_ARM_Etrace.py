# Copyright (c) 2012, 2017-2018, 2023 Arm Limited
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

from m5.objects import *
from .O3_ARM_v7a import O3_ARM_v7a_3

# O3_ARM_v7a_3 adapted to generate elastic traces
class O3_ARM_v7a_3_Etrace(O3_ARM_v7a_3):
    # Make the number of entries in the ROB, LQ and SQ very
    # large so that there are no stalls due to resource
    # limitation as such stalls will get captured in the trace
    # as compute delay. For replay, ROB, LQ and SQ sizes are
    # modelled in the Trace CPU.
    numROBEntries = 512
    LQEntries = 128
    SQEntries = 128

    def attach_probe_listener(self, inst_trace_file, data_trace_file):
        # Attach the elastic trace probe listener. Set the protobuf trace
        # file names. Set the dependency window size equal to the cpu it
        # is attached to.
        self.traceListener = m5.objects.ElasticTrace(
            instFetchTraceFile=inst_trace_file,
            dataDepTraceFile=data_trace_file,
            depWindowSize=3 * self.numROBEntries,
        )
