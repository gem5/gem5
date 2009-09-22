# -*- mode:python -*-

# Copyright (c) 2007 MIPS Technologies, Inc.
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
#
# Authors: Jaidev Patwardhan
#          Korey Sewell

from m5.defines import buildEnv
from m5.params import *

from BaseCPU import BaseCPU

class BaseMipsCPU(BaseCPU)
    if buildEnv['TARGET_ISA'] == 'mips':
        CP0_IntCtl_IPTI = Param.Unsigned(0,"No Description")
        CP0_IntCtl_IPPCI = Param.Unsigned(0,"No Description")
        CP0_SrsCtl_HSS = Param.Unsigned(0,"No Description")
        CP0_EBase_CPUNum = Param.Unsigned(0,"No Description")
        CP0_PRId_CompanyOptions = Param.Unsigned(0,"Company Options in Processor ID Register")
        CP0_PRId_CompanyID = Param.Unsigned(0,"Company Identifier in Processor ID Register")
        CP0_PRId_ProcessorID = Param.Unsigned(1,"Processor ID (0=>Not MIPS32/64 Processor, 1=>MIPS, 2-255 => Other Company")
        CP0_PRId_Revision = Param.Unsigned(0,"Processor Revision Number in Processor ID Register")
        CP0_Config_BE = Param.Unsigned(0,"Big Endian?")
        CP0_Config_AT = Param.Unsigned(0,"No Description")
        CP0_Config_AR = Param.Unsigned(0,"No Description")
        CP0_Config_MT = Param.Unsigned(0,"No Description")
        CP0_Config_VI = Param.Unsigned(0,"No Description")
        CP0_Config1_M = Param.Unsigned(0,"Config2 Implemented?")
        CP0_Config1_MMU = Param.Unsigned(0,"MMU Type")
        CP0_Config1_IS = Param.Unsigned(0,"No Description")
        CP0_Config1_IL = Param.Unsigned(0,"No Description")
        CP0_Config1_IA = Param.Unsigned(0,"No Description")
        CP0_Config1_DS = Param.Unsigned(0,"No Description")
        CP0_Config1_DL = Param.Unsigned(0,"No Description")
        CP0_Config1_DA = Param.Unsigned(0,"No Description")
        CP0_Config1_C2 = Param.Bool(False,"No Description")
        CP0_Config1_MD = Param.Bool(False,"No Description")
        CP0_Config1_PC = Param.Bool(False,"No Description")
        CP0_Config1_WR = Param.Bool(False,"No Description")
        CP0_Config1_CA = Param.Bool(False,"No Description")
        CP0_Config1_EP = Param.Bool(False,"No Description")
        CP0_Config1_FP = Param.Bool(False,"FPU Implemented?")
        CP0_Config2_M = Param.Bool(False,"Config3 Implemented?")
        CP0_Config2_TU = Param.Unsigned(0,"No Description")
        CP0_Config2_TS = Param.Unsigned(0,"No Description")
        CP0_Config2_TL = Param.Unsigned(0,"No Description")
        CP0_Config2_TA = Param.Unsigned(0,"No Description")
        CP0_Config2_SU = Param.Unsigned(0,"No Description")
        CP0_Config2_SS = Param.Unsigned(0,"No Description")
        CP0_Config2_SL = Param.Unsigned(0,"No Description")
        CP0_Config2_SA = Param.Unsigned(0,"No Description")
        CP0_Config3_M = Param.Bool(False,"Config4 Implemented?")
        CP0_Config3_DSPP = Param.Bool(False,"DSP Extensions Present?")
        CP0_Config3_LPA = Param.Bool(False,"No Description")
        CP0_Config3_VEIC = Param.Bool(False,"No Description")
        CP0_Config3_VInt = Param.Bool(False,"No Description")
        CP0_Config3_SP = Param.Bool(False,"No Description")
        CP0_Config3_MT = Param.Bool(False,"Multithreading Extensions Present?")
        CP0_Config3_SM = Param.Bool(False,"No Description")
        CP0_Config3_TL = Param.Bool(False,"No Description")
        CP0_WatchHi_M = Param.Bool(False,"No Description")
        CP0_PerfCtr_M = Param.Bool(False,"No Description")
        CP0_PerfCtr_W = Param.Bool(False,"No Description")
        CP0_PRId = Param.Unsigned(0,"CP0 Status Register")
        CP0_Config = Param.Unsigned(0,"CP0 Config Register")
        CP0_Config1 = Param.Unsigned(0,"CP0 Config1 Register")
        CP0_Config2 = Param.Unsigned(0,"CP0 Config2 Register")
        CP0_Config3 = Param.Unsigned(0,"CP0 Config3 Register")
