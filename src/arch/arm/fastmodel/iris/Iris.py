# Copyright 2019 Google, Inc.
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

from m5.params import *
from m5.proxy import *

from m5.objects.BaseCPU import BaseCPU
from m5.objects.BaseTLB import BaseTLB

class IrisTLB(BaseTLB):
    type = 'IrisTLB'
    cxx_class = 'Iris::TLB'
    cxx_header = 'arch/arm/fastmodel/iris/tlb.hh'

class IrisBaseCPU(BaseCPU):
    type = 'IrisBaseCPU'
    abstract = True
    cxx_class = 'Iris::BaseCPU'
    cxx_header = 'arch/arm/fastmodel/iris/cpu.hh'

    @classmethod
    def memory_mode(cls):
        return 'atomic_noncaching'

    @classmethod
    def require_caches(cls):
        return False

    @classmethod
    def support_take_over(cls):
        #TODO Make this work.
        return False

    evs = Param.SystemC_ScModule(
            "Fast model exported virtual subsystem holding cores")
    thread_paths = VectorParam.String(
            "Sub-paths to elements in the EVS which support a thread context")

    dtb = IrisTLB()
    itb = IrisTLB()
