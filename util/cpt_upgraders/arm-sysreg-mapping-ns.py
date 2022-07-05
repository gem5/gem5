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

# reflect updated register mappings for ARM ISA
def upgrader(cpt):
    if cpt.get("root", "isa", fallback="") == "arm":
        for sec in cpt.sections():
            import re

            # Search for all ISA sections
            if re.search(".*sys.*\.cpu.*\.isa\d*$", sec):
                mr = cpt.get(sec, "miscRegs").split()
                if int(mr[0]) & 16 == 0:  # CPSR reg width; 0 for AArch64
                    mr[112] = mr[111]  # ACTLR_NS = ACTLR
                    mr[146] = mr[145]  # ADFSR_NS = ADFSR
                    mr[149] = mr[148]  # AIFSR_NS = AIFSR
                    mr[253] = mr[252]  # AMAIR0_NS = AMAIR0
                    mr[289] = mr[288]  # CNTP_CTL_NS = CNTP_CTL
                    mr[313] = mr[312]  # CNTP_CVAL_NS = CNTP_CVAL
                    mr[286] = mr[285]  # CNTP_TVAL_NS = CNTP_TVAL
                    mr[271] = mr[270]  # CONTEXTIDR_NS = CONTEXTIDR
                    mr[104] = mr[103]  # CSSELR_NS = CSSELR
                    mr[137] = mr[136]  # DACR_NS = DACR
                    mr[155] = mr[154]  # DFAR_NS = DFAR
                    mr[158] = mr[157]  # IFAR_NS = IFAR
                    mr[143] = mr[142]  # IFSR_NS = IFSR
                    mr[247] = mr[246]  # NMRR_NS = NMRR
                    mr[166] = mr[165]  # PAR_NS = PAR
                    mr[241] = mr[240]  # PRRR_NS = PRRR
                    mr[4] = mr[424]  # SPSR_SVC = SPSR_EL1
                    mr[7] = mr[435]  # SPSR_HYP = SPSR_EL2
                    mr[5] = mr[442]  # SPSR_MON = SPSR_EL3
                    mr[277] = mr[276]  # TPIDRURO_NS = TPIDRURO
                    mr[280] = mr[279]  # TPIDRPRW_NS = TPIDRPRW
                    mr[274] = mr[273]  # TPIDRURW_NS = TPIDRURW
                    mr[132] = mr[131]  # TTBCR_NS = TTBCR
                    mr[126] = mr[125]  # TTBR0_NS = TTBR0
                    mr[129] = mr[128]  # TTBR1_NS = TTBR1
                    mr[263] = mr[262]  # VBAR_NS = VBAR

                    cpt.set(sec, "miscRegs", " ".join(str(x) for x in mr))
