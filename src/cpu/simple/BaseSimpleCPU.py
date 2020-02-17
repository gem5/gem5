# Copyright (c) 2008 The Hewlett-Packard Development Company
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

from __future__ import print_function

from m5.defines import buildEnv
from m5.params import *

from m5.objects.BaseCPU import BaseCPU
from m5.objects.DummyChecker import DummyChecker
from m5.objects.BranchPredictor import *

class BaseSimpleCPU(BaseCPU):
    type = 'BaseSimpleCPU'
    abstract = True
    cxx_header = "cpu/simple/base.hh"

    def addCheckerCpu(self):
        if buildEnv['TARGET_ISA'] in ['arm']:
            from m5.objects.ArmTLB import ArmITB, ArmDTB

            self.checker = DummyChecker(workload = self.workload)
            self.checker.itb = ArmITB(size = self.itb.size)
            self.checker.dtb = ArmDTB(size = self.dtb.size)
        else:
            print("ERROR: Checker only supported under ARM ISA!")
            exit(1)

    branchPred = Param.BranchPredictor(NULL, "Branch Predictor")
