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


# Register usage:
#  t1, t2 == temporaries

fldenvTemplate = """
    ld t1, seg, %(mode)s, "DISPLACEMENT + 0", dataSize=2
    wrval fcw, t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 4", dataSize=2
    wrval fsw, t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=2
    wrval ftw, t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 12", dataSize=4
    wrval ctrlRegIdx("misc_reg::Fioff"), t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=2
    wrval ctrlRegIdx("misc_reg::Fiseg"), t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 16 + 2", dataSize=2
    wrval ctrlRegIdx("misc_reg::Fop"), t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 20", dataSize=4
    wrval ctrlRegIdx("misc_reg::Fooff"), t1

    ld t1, seg, %(mode)s, "DISPLACEMENT + 24", dataSize=2
    wrval ctrlRegIdx("misc_reg::Foseg"), t1
"""

fnstenvTemplate = """
    rdval t2, fcw
    st t2, seg, %(mode)s, "DISPLACEMENT + 0", dataSize=2

    # FSW includes TOP when read
    rdval t1, fsw
    st t1, seg, %(mode)s, "DISPLACEMENT + 4", dataSize=2
    srli t1, t1, 11, dataSize=2
    andi t1, t1, 0x7, dataSize=2
    wrval ctrlRegIdx("misc_reg::X87Top"), t1

    rdval t1, ftw
    st t1, seg, %(mode)s, "DISPLACEMENT + 8", dataSize=2

    rdval t1, ctrlRegIdx("misc_reg::Fioff")
    st t1, seg, %(mode)s, "DISPLACEMENT + 12", dataSize=4

    rdval t1, ctrlRegIdx("misc_reg::Fiseg")
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 0", dataSize=2

    rdval t1, ctrlRegIdx("misc_reg::Fop")
    st t1, seg, %(mode)s, "DISPLACEMENT + 16 + 2", dataSize=2

    rdval t1, ctrlRegIdx("misc_reg::Fooff")
    st t1, seg, %(mode)s, "DISPLACEMENT + 20", dataSize=4

    rdval t1, ctrlRegIdx("misc_reg::Foseg")
    st t1, seg, %(mode)s, "DISPLACEMENT + 24", dataSize=2

    # Mask exceptions
    ori t2, t2, 0x3F
    wrval fcw, t2
"""

microcode = (
    """
def macroop FLDENV_M {
"""
    + fldenvTemplate % {"mode": "sib"}
    + """
};

def macroop FLDENV_P {
    rdip t7
"""
    + fldenvTemplate % {"mode": "riprel"}
    + """
};

def macroop FNSTENV_M {
"""
    + fnstenvTemplate % {"mode": "sib"}
    + """
};

def macroop FNSTENV_P {
    rdip t7
"""
    + fnstenvTemplate % {"mode": "riprel"}
    + """
};
"""
)
