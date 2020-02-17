# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
# All rights reserved.
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

microcode = '''
def macroop IRET_REAL {
    .serialize_after
    panic "Real mode iret isn't implemented!"
};

def macroop IRET_PROT {
    .serialize_after
    .adjust_env oszIn64Override

    # Check for a nested task. This isn't supported at the moment.
    rflag t1, 14; #NT bit
    panic "Task switching with iret is unimplemented!", flags=(nCEZF,)

    #t1 = temp_RIP
    #t2 = temp_CS
    #t3 = temp_RFLAGS
    #t4 = handy m5 register

    # Pop temp_RIP, temp_CS, and temp_RFLAGS
    ld t1, ss, [1, t0, rsp], "0 * env.stackSize", dataSize=ssz
    ld t2, ss, [1, t0, rsp], "1 * env.stackSize", dataSize=ssz
    ld t3, ss, [1, t0, rsp], "2 * env.stackSize", dataSize=ssz

    # Read the handy m5 register for use later
    rdm5reg t4


###
### Handle if we're returning to virtual 8086 mode.
###

    #IF ((temp_RFLAGS.VM=1) && (CPL=0) && (LEGACY_MODE))
    #    IRET_FROM_PROTECTED_TO_VIRTUAL

    #temp_RFLAGS.VM != 1
    rcri t0, t3, 18, flags=(ECF,)
    br label("protToVirtFallThrough"), flags=(nCECF,)

    #CPL=0
    andi t0, t4, 0x30, flags=(EZF,)
    br label("protToVirtFallThrough"), flags=(nCEZF,)

    #(LEGACY_MODE)
    rcri t0, t4, 1, flags=(ECF,)
    br label("protToVirtFallThrough"), flags=(nCECF,)

    panic "iret to virtual mode not supported"

protToVirtFallThrough:



    #temp_CPL = temp_CS.rpl
    andi t5, t2, 0x3


###
### Read in the info for the new CS segment.
###

    #CS = READ_DESCRIPTOR (temp_CS, iret_chk)
    andi t0, t2, 0xFC, flags=(EZF,), dataSize=2
    br label("processCSDescriptor"), flags=(CEZF,)
    andi t6, t2, 0xF8, dataSize=8
    andi t0, t2, 0x4, flags=(EZF,), dataSize=2
    br label("globalCSDescriptor"), flags=(CEZF,)
    ld t8, tsl, [1, t0, t6], dataSize=8, atCPL0=True
    br label("processCSDescriptor")
globalCSDescriptor:
    ld t8, tsg, [1, t0, t6], dataSize=8, atCPL0=True
processCSDescriptor:
    chks t2, t6, dataSize=8


###
### Get the new stack pointer and stack segment off the old stack if necessary,
### and piggyback on the logic to check the new RIP value.
###
    #IF ((64BIT_MODE) || (temp_CPL!=CPL))
    #{

    #(64BIT_MODE)
    andi t0, t4, 0xE, flags=(EZF,)
    # Since we just found out we're in 64 bit mode, take advantage and
    # do the appropriate RIP checks.
    br label("doPopStackStuffAndCheckRIP"), flags=(CEZF,)

    # Here, we know we're -not- in 64 bit mode, so we should do the
    # appropriate/other RIP checks.
    # if temp_RIP > CS.limit throw #GP(0)
    rdlimit t6, cs, dataSize=8
    sub t0, t1, t6, flags=(ECF,)
    fault "std::make_shared<GeneralProtection>(0)", flags=(CECF,)

    #(temp_CPL!=CPL)
    srli t7, t4, 4
    xor t7, t7, t5
    andi t0, t7, 0x3, flags=(EZF,)
    br label("doPopStackStuff"), flags=(nCEZF,)
    # We can modify user visible state here because we're know
    # we're done with things that can fault.
    addi rsp, rsp, "3 * env.stackSize"
    br label("fallThroughPopStackStuff")

doPopStackStuffAndCheckRIP:
    # Check if the RIP is canonical.
    srai t7, t1, 47, flags=(EZF,), dataSize=ssz
    # if t7 isn't 0 or -1, it wasn't canonical.
    br label("doPopStackStuff"), flags=(CEZF,)
    addi t0, t7, 1, flags=(EZF,), dataSize=ssz
    fault "std::make_shared<GeneralProtection>(0)", flags=(nCEZF,)

doPopStackStuff:
    #    POP.v temp_RSP
    ld t6, ss, [1, t0, rsp], "3 * env.dataSize", dataSize=ssz
    #    POP.v temp_SS
    ld t9, ss, [1, t0, rsp], "4 * env.dataSize", dataSize=ssz
    #    SS = READ_DESCRIPTOR (temp_SS, ss_chk)
    andi t0, t9, 0xFC, flags=(EZF,), dataSize=2
    br label("processSSDescriptor"), flags=(CEZF,)
    andi t7, t9, 0xF8, dataSize=8
    andi t0, t9, 0x4, flags=(EZF,), dataSize=2
    br label("globalSSDescriptor"), flags=(CEZF,)
    ld t7, tsl, [1, t0, t7], dataSize=8, atCPL0=True
    br label("processSSDescriptor")
globalSSDescriptor:
    ld t7, tsg, [1, t0, t7], dataSize=8, atCPL0=True
processSSDescriptor:
    chks t9, t7, dataSize=8

    # This actually updates state which is wrong. It should wait until we know
    # we're not going to fault. Unfortunately, that's hard to do.
    wrdl ss, t7, t9
    wrsel ss, t9

###
### From this point downwards, we can't fault. We can update user visible state.
###
    #    RSP.s = temp_RSP
    mov rsp, rsp, t6, dataSize=ssz

    #}

fallThroughPopStackStuff:

    # Update CS
    wrdl cs, t8, t2
    wrsel cs, t2

    #CPL = temp_CPL

    #IF (changing CPL)
    #{
    srli t7, t4, 4
    xor t7, t7, t5
    andi t0, t7, 0x3, flags=(EZF,)
    br label("skipSegmentSquashing"), flags=(CEZF,)

    # The attribute register needs to keep track of more info before this will
    # work the way it needs to.
    #    FOR (seg = ES, DS, FS, GS)
    #        IF ((seg.attr.dpl < cpl && ((seg.attr.type = 'data')
    #            || (seg.attr.type = 'non-conforming-code')))
    #        {
    #            seg = NULL
    #        }
    #}

skipSegmentSquashing:

    # Ignore this for now.
    #RFLAGS.v = temp_RFLAGS
    wrflags t0, t3
    #  VIF,VIP,IOPL only changed if (old_CPL = 0)
    #  IF only changed if (old_CPL <= old_RFLAGS.IOPL)
    #  VM unchanged
    #  RF cleared

    #RIP = temp_RIP
    wrip t0, t1, dataSize=ssz
};

def macroop IRET_VIRT {
    panic "Virtual mode iret isn't implemented!"
};

def macroop INT3 {

    limm t1, 0x03, dataSize=8

    rdip t7

    # Are we in long mode?
    rdm5reg t5
    andi t0, t5, 0x1, flags=(EZF,)
    br rom_label("longModeSoftInterrupt"), flags=(CEZF,)
    br rom_label("legacyModeInterrupt")
};

def macroop INT_I {

    #load the byte-sized interrupt vector specified in the instruction
    .adjust_imm trimImm(8)
    limm t1, imm, dataSize=8

    rdip t7

    # Are we in long mode?
    rdm5reg t5
    andi t0, t5, 0x1, flags=(EZF,)
    br rom_label("longModeSoftInterrupt"), flags=(CEZF,)
    br rom_label("legacyModeInterrupt")
};
'''
#let {{
#    class INT(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class INTO(Inst):
#       "GenFault ${new UnimpInstFault}"
#}};
