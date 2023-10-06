# Copyright 2021 Google, Inc.
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
import m5.defines

arch_vars = [
    "USE_ARM_ISA",
    "USE_MIPS_ISA",
    "USE_POWER_ISA",
    "USE_RISCV_ISA",
    "USE_SPARC_ISA",
    "USE_X86_ISA",
]

enabled = list(filter(lambda var: m5.defines.buildEnv[var], arch_vars))

if len(enabled) == 1:
    arch = enabled[0]
    if arch == "USE_ARM_ISA":
        from m5.objects.ArmCPU import ArmAtomicSimpleCPU as AtomicSimpleCPU
    elif arch == "USE_MIPS_ISA":
        from m5.objects.MipsCPU import MipsAtomicSimpleCPU as AtomicSimpleCPU
    elif arch == "USE_POWER_ISA":
        from m5.objects.PowerCPU import PowerAtomicSimpleCPU as AtomicSimpleCPU
    elif arch == "USE_RISCV_ISA":
        from m5.objects.RiscvCPU import RiscvAtomicSimpleCPU as AtomicSimpleCPU
    elif arch == "USE_SPARC_ISA":
        from m5.objects.SparcCPU import SparcAtomicSimpleCPU as AtomicSimpleCPU
    elif arch == "USE_X86_ISA":
        from m5.objects.X86CPU import X86AtomicSimpleCPU as AtomicSimpleCPU
