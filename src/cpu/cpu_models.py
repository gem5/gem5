# Copyright (c) 2003-2006 The Regents of The University of Michigan
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

################
# CpuModel class
#
# The CpuModel class encapsulates everything the ISA parser needs to
# know about a particular CPU model.

class CpuModel:
    # Dict of available CPU model objects.  Accessible as CpuModel.dict.
    dict = {}

    # Constructor.  Automatically adds models to CpuModel.dict.
    def __init__(self, name, filename, includes, strings):
        self.name = name
        self.filename = filename   # filename for output exec code
        self.includes = includes   # include files needed in exec file
        # The 'strings' dict holds all the per-CPU symbols we can
        # substitute into templates etc.
        self.strings = strings
        # Add self to dict
        CpuModel.dict[name] = self


#
# Define CPU models.
#
# Parameters are:
#   - name of model
#   - filename for generated ISA execution file
#   - includes needed for generated ISA execution file
#   - substitution strings for ISA description templates
#

CpuModel('AtomicSimpleCPU', 'atomic_simple_cpu_exec.cc',
         '#include "cpu/simple/atomic.hh"',
         { 'CPU_exec_context': 'AtomicSimpleCPU' })
CpuModel('TimingSimpleCPU', 'timing_simple_cpu_exec.cc',
         '#include "cpu/simple/timing.hh"',
         { 'CPU_exec_context': 'TimingSimpleCPU' })
CpuModel('FullCPU', 'full_cpu_exec.cc',
         '#include "encumbered/cpu/full/dyn_inst.hh"',
         { 'CPU_exec_context': 'DynInst' })
CpuModel('AlphaFullCPU', 'alpha_o3_exec.cc',
         '#include "cpu/o3/alpha_dyn_inst.hh"',
         { 'CPU_exec_context': 'AlphaDynInst<AlphaSimpleImpl>' })
CpuModel('OzoneSimpleCPU', 'ozone_simple_exec.cc',
         '#include "cpu/ozone/dyn_inst.hh"',
         { 'CPU_exec_context': 'OzoneDynInst<SimpleImpl>' })
CpuModel('OzoneCPU', 'ozone_exec.cc',
         '#include "cpu/ozone/dyn_inst.hh"',
         { 'CPU_exec_context': 'OzoneDynInst<OzoneImpl>' })
CpuModel('CheckerCPU', 'checker_cpu_exec.cc',
         '#include "cpu/checker/cpu.hh"',
         { 'CPU_exec_context': 'CheckerCPU' })

