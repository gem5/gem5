# Copyright (c) 2007 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# The software must be used only for Non-Commercial Use which means any
# use which is NOT directed to receiving any direct monetary
# compensation for, or commercial advantage from such use.  Illustrative
# examples of non-commercial use are academic research, personal study,
# teaching, education and corporate research & development.
# Illustrative examples of commercial use are distributing products for
# commercial advantage and providing services using the software for
# commercial advantage.
#
# If you wish to use this software or functionality therein that may be
# covered by patents for commercial use, please contact:
#     Director of Intellectual Property Licensing
#     Office of Strategy and Technology
#     Hewlett-Packard Company
#     1501 Page Mill Road
#     Palo Alto, California  94304
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  Neither the name of
# the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.  No right of
# sublicense is granted herewith.  Derivatives of the software and
# output created using the software may be prepared, but only for
# Non-Commercial Uses.  Derivatives of the software may be shared with
# others provided: (i) the others agree to abide by the list of
# conditions herein which includes the Non-Commercial Use restrictions;
# and (ii) such Derivatives of the software include the above copyright
# notice to acknowledge the contribution from this software where
# applicable, this list of conditions and the disclaimer below.
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
# Authors: Gabe Black

microcode = '''
def macroop CPUID_R {

#
# Find which type of cpuid function it is by checking bit 31. Also clear that
# bit to form an offset into the functions of that type.
#
    limm t1, 0x80000000, dataSize=4
    and t2, t1, rax, flags=(EZF,)
    # clear the bit
    xor t1, t2, rax

#
# Do range checking on the offset
#
    # If EZF is set, the function is standard and the max is 0x1.
    movi t2, t2, 0x1, flags=(CEZF,)
    # If EZF is cleared, the function is extended and the max is 0x18.
    movi t2, t2, 0x18, flags=(nCEZF,)
    subi t0, t1, t2, flags=(ECF,)
    # ECF will be set if the offset is too large.
    bri t0, label("end"), flags=(CECF,)


#
# Jump to the right portion
#
    movi t2, t2, label("standardStart"), flags=(CEZF,)
    movi t2, t2, label("extendedStart"), flags=(nCEZF,)
    # This gives each function 8 microops to use. It's wasteful because only
    # 5 will be needed, but a multiply would be expensive. In the system
    # described in the RISC86 patent, the fifth instruction would really be
    # the sequencing field on an op quad, so each function would be implemented
    # by -exactly- one op quad. Since we're approximating, this should be ok.
    slli t1, t1, 3
    br t2, t1

#############################################################################
#############################################################################

#
# Standard functions.
#

standardStart:

# 0x00000000 -- Processor Vendor and Largest Standard Function Number
    limm rax, 0x00000001, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x00000001 -- Family, Model, Stepping Identifiers
    limm rax, 0x00020f51, dataSize=4
    limm rbx, 0x00000405, dataSize=4
    limm rdx, 0xe3d3fbff, dataSize=4
    limm rcx, 0x00000001, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

#
# Extended functions.
#

extendedStart:

# 0x80000000 -- Processor Vendor and Largest Extended Function Number
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000001 -- EAX: AMD Family, Model, Stepping
#               EBX: BrandId Identifier
#               ECX: Feature Identifiers
#               EDX: Feature Identifiers
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000002 -- Processor Name String Identifier
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000003 -- Processor Name String Identifier
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000004 -- Processor Name String Identifier
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000005 -- L1 Cache and TLB Identifiers
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000006 -- L2/L3 Cache and L2 TLB Identifiers
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000007 -- Advanced Power Management Information
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000008 -- Long Mode Address Size Identification
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000009 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000A -- SVM Revision and Feature Identification
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000B -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000C -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000D -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000E -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x8000000F -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000010 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000011 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000012 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000013 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000014 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000015 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000016 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000017 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

# 0x80000018 -- Reserved
    # JUNK VALUES
    limm rax, 0x80000018, dataSize=4
    limm rbx, 0x68747541, dataSize=4
    limm rdx, 0x69746e65, dataSize=4
    limm rcx, 0x444d4163, dataSize=4
    bri t0, label("end")
    fault "NoFault"
    fault "NoFault"
    fault "NoFault"

end:
    fault "NoFault"
};
'''
