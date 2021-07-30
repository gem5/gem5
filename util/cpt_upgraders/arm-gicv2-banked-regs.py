# Copyright (c) 2016 ARM Limited
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

# duplicate banked registers into new per-cpu arrays.
def upgrader(cpt):
    if cpt.get('root', 'isa', fallback='') == 'arm':
        for sec in cpt.sections():
            import re

            if not re.search('\.gic$', sec):
                continue
            cpuEnabled  = cpt.get(sec, 'cpuEnabled' ).split()

            intEnabled  = cpt.get(sec, 'intEnabled' ).split()
            pendingInt  = cpt.get(sec, 'pendingInt' ).split()
            activeInt   = cpt.get(sec, 'activeInt'  ).split()
            intPriority = cpt.get(sec, 'intPriority').split()
            cpuTarget   = cpt.get(sec, 'cpuTarget'  ).split()

            b_intEnabled = intEnabled[0]
            b_pendingInt = pendingInt[0]
            b_activeInt  = activeInt[0]

            del intEnabled[0]
            del pendingInt[0]
            del activeInt[0]
            del intPriority[0:32] # unused; overlapped with bankedIntPriority
            del cpuTarget[0:32]

            cpt.set(sec, 'intEnabled', ' '.join(intEnabled))
            cpt.set(sec, 'pendingInt', ' '.join(pendingInt))
            cpt.set(sec, 'activeInt',  ' '.join(activeInt))
            cpt.set(sec, 'intPriority',' '.join(intPriority))
            cpt.set(sec, 'cpuTarget',  ' '.join(cpuTarget))

            b_intPriority = cpt.get(sec, '*bankedIntPriority').split()
            cpt.remove_option(sec, '*bankedIntPriority')

            for cpu in range(255):
                if cpuEnabled[cpu] == 'true':
                    intPriority = b_intPriority[cpu*32 : (cpu+1)*32]
                    new_sec = "%s.bankedRegs%u" % (sec, cpu)
                    cpt.add_section(new_sec)
                    cpt.set(new_sec, 'intEnabled', b_intEnabled)
                    cpt.set(new_sec, 'pendingInt', b_pendingInt)
                    cpt.set(new_sec, 'activeInt',  b_activeInt)
                    cpt.set(new_sec, 'intPriority',' '.join(intPriority))
