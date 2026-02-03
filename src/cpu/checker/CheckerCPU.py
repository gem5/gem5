# Copyright (c) 2007 The Regents of The University of Michigan
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

from m5.objects.BaseCPU import BaseCPU
from m5.params import *
from m5.SimObject import SimObject


class CheckerCPU(BaseCPU):
    type = "CheckerCPU"
    abstract = True
    cxx_header = "cpu/checker/cpu.hh"
    cxx_class = "gem5::CheckerCPU"

    exitOnError = Param.Bool(False, "Exit on an error")
    updateOnError = Param.Bool(
        False, "Update the checker with the main CPU's state on an error"
    )
    warnOnlyOnLoadError = Param.Bool(
        True,
        "If a load result is incorrect, only print a warning and do not exit",
    )

    def generateDeviceTree(self, state):
        # The CheckerCPU is not a real CPU and shouldn't generate a DTB
        # node. This is why we are skipping the BaseCPU implementation
        # and we are calling the base SimObject one.
        return SimObject.generateDeviceTree(self, state)
