# Copyright (c) 2013 Andreas Sandberg
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
# Authors: Andreas Sandberg

# Register usage:
#  t1, t2 == temporaries
#  t7 == base address (RIP or SIB)


loadX87RegTemplate =  '''
    ld t1, seg, %(mode)s, "DISPLACEMENT + 32 + 16 * %(idx)i", dataSize=8
    ld t2, seg, %(mode)s, "DISPLACEMENT + 32 + 16 * %(idx)i + 8", dataSize=2
    cvtint_fp80 st(%(idx)i), t1, t2
'''

storeX87RegTemplate = '''
    cvtfp80h_int t1, st(%(idx)i)
    cvtfp80l_int t2, st(%(idx)i)
    st t1, seg, %(mode)s, "DISPLACEMENT + 32 + 16 * %(idx)i", dataSize=8
    st t2, seg, %(mode)s, "DISPLACEMENT + 32 + 16 * %(idx)i + 8", dataSize=2
'''

loadXMMRegTemplate =  '''
    ldfp "InstRegIndex(FLOATREG_XMM_LOW(%(idx)i))", seg, %(mode)s, \
         "DISPLACEMENT + 160 + 16 * %(idx)i", dataSize=8
    ldfp "InstRegIndex(FLOATREG_XMM_HIGH(%(idx)i))", seg, %(mode)s, \
         "DISPLACEMENT + 160 + 16 * %(idx)i + 8", dataSize=8
'''

storeXMMRegTemplate =  '''
    stfp "InstRegIndex(FLOATREG_XMM_LOW(%(idx)i))", seg, %(mode)s, \
         "DISPLACEMENT + 160 + 16 * %(idx)i", dataSize=8
    stfp "InstRegIndex(FLOATREG_XMM_HIGH(%(idx)i))", seg, %(mode)s, \
         "DISPLACEMENT + 160 + 16 * %(idx)i + 8", dataSize=8
'''

loadAllDataRegs = \
    "".join([loadX87RegTemplate % { "idx" : i, "mode" : "%(mode)s" }
             for i in range(8)]) + \
    "".join([loadXMMRegTemplate % { "idx" : i, "mode" : "%(mode)s" }
             for i in range(16)])

storeAllDataRegs = \
    "".join([storeX87RegTemplate % { "idx" : i, "mode" : "%(mode)s" }
             for i in range(8)]) + \
    "".join([storeXMMRegTemplate % { "idx" : i, "mode" : "%(mode)s" }
             for i in range(16)])

fxsaveCommonTemplate = """
    rdval t1, fcw
    st t1, seg, %(mode)s, "DISPLACEMENT + 0", dataSize=2

    # FSW includes TOP when read
    rdval t1, fsw
    st t1, seg, %(mode)s, "DISPLACEMENT + 2", dataSize=2

    # FTW
    rdxftw t1
    st t1, seg, %(mode)s, "DISPLACEMENT + 4", dataSize=1

    rdval t1, "InstRegIndex(MISCREG_FOP)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 6", dataSize=2

    rdval t1, "InstRegIndex(MISCREG_MXCSR)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 8", dataSize=4

    # MXCSR_MASK, software assumes the default (0xFFBF) if 0.
    limm t1, 0xFFFF
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 12", dataSize=4
""" + storeAllDataRegs

fxsave32Template = """
    rdval t1, "InstRegIndex(MISCREG_FIOFF)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=4

    rdval t1, "InstRegIndex(MISCREG_FISEG)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 12", dataSize=2

    rdval t1, "InstRegIndex(MISCREG_FOOFF)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=4

    rdval t1, "InstRegIndex(MISCREG_FOSEG)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 4", dataSize=2
""" + fxsaveCommonTemplate

fxsave64Template = """
    rdval t1, "InstRegIndex(MISCREG_FIOFF)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=8

    rdval t1, "InstRegIndex(MISCREG_FOOFF)"
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=8
""" + fxsaveCommonTemplate

fxrstorCommonTemplate = """
    ld t1, seg, %(mode)s, "DISPLACEMENT + 0", dataSize=2
    wrval fcw, t1

    # FSW includes TOP when read
    ld t1, seg, %(mode)s, "DISPLACEMENT + 2", dataSize=2
    wrval fsw, t1

    # FTW
    ld t1, seg, %(mode)s, "DISPLACEMENT + 4", dataSize=1
    wrxftw t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 6", dataSize=2
    wrval "InstRegIndex(MISCREG_FOP)", t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 8", dataSize=4
    wrval "InstRegIndex(MISCREG_MXCSR)", t1
""" + loadAllDataRegs

fxrstor32Template = """
    ld t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=4
    wrval "InstRegIndex(MISCREG_FIOFF)", t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 12", dataSize=2
    wrval "InstRegIndex(MISCREG_FISEG)", t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=4
    wrval "InstRegIndex(MISCREG_FOOFF)", t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 4", dataSize=2
    wrval "InstRegIndex(MISCREG_FOSEG)", t1
""" + fxrstorCommonTemplate

fxrstor64Template = """
    limm t2, 0, dataSize=8

    ld t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=8
    wrval "InstRegIndex(MISCREG_FIOFF)", t1
    wrval "InstRegIndex(MISCREG_FISEG)", t2

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=8
    wrval "InstRegIndex(MISCREG_FOOFF)", t1
    wrval "InstRegIndex(MISCREG_FOSEG)", t2
""" + fxrstorCommonTemplate

microcode = '''
def macroop FXSAVE_M {
''' + fxsave32Template % { "mode" : "sib" } + '''
};

def macroop FXSAVE_P {
    rdip t7
''' + fxsave32Template % { "mode" : "riprel" } + '''
};

def macroop FXSAVE64_M {
''' + fxsave64Template % { "mode" : "sib" } + '''
};

def macroop FXSAVE64_P {
    rdip t7
''' + fxsave64Template % { "mode" : "riprel" } + '''
};

def macroop FXRSTOR_M {
''' + fxrstor32Template % { "mode" : "sib" } + '''
};

def macroop FXRSTOR_P {
    rdip t7
''' + fxrstor32Template % { "mode" : "riprel" } + '''
};

def macroop FXRSTOR64_M {
''' + fxrstor64Template % { "mode" : "sib" } + '''
};

def macroop FXRSTOR64_P {
    rdip t7
''' + fxrstor64Template % { "mode" : "riprel" } + '''
};
'''
