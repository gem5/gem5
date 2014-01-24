/*
 * Copyright (c) 2010, 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          William Wang
 */

/*
 * Copyright (c) 1990, 1993 The Regents of the University of California
 * All rights reserved
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Lawrence Berkeley Laboratories.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      @(#)kgdb_stub.c 8.4 (Berkeley) 1/12/94
 */

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the NetBSD
 *      Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * $NetBSD: kgdb_stub.c,v 1.8 2001/07/07 22:58:00 wdk Exp $
 *
 * Taken from NetBSD
 *
 * "Stub" to allow remote cpu to debug over a serial line using gdb.
 */

#include <sys/signal.h>
#include <unistd.h>

#include <string>

#include "arch/arm/decoder.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/remote_gdb.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBMisc.hh"
#include "mem/page_table.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

RemoteGDB::RemoteGDB(System *_system, ThreadContext *tc)
    : BaseRemoteGDB(_system, tc, MAX_NUMREGS)
{
}

/*
 * Determine if the mapping at va..(va+len) is valid.
 */
bool
RemoteGDB::acc(Addr va, size_t len)
{
    if (FullSystem) {
        Addr last_va;
        va       = truncPage(va);
        last_va  = roundPage(va + len);

        do  {
            if (virtvalid(context, va)) {
                return true;
            }
            va += PageBytes;
        } while (va < last_va);

        DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
        return true;
    } else {
        TlbEntry entry;
        //Check to make sure the first byte is mapped into the processes address
        //space.
        if (context->getProcessPtr()->pTable->lookup(va, entry))
            return true;
        return false;
    }
}

/*
 * Translate the kernel debugger register format into the GDB register
 * format.
 */
void
RemoteGDB::getregs()
{
    DPRINTF(GDBAcc, "getregs in remotegdb \n");

    memset(gdbregs.regs, 0, gdbregs.bytes());

    if (inAArch64(context)) {  // AArch64
        // x0-x31
        for (int i = 0; i < 32; ++i) {
            gdbregs.regs[REG_X0 + i] = context->readIntReg(INTREG_X0 + i);
        }
        // pc
        gdbregs.regs[REG_PC_64] = context->pcState().pc();
        // cpsr
        gdbregs.regs[REG_CPSR_64] = context->readMiscRegNoEffect(MISCREG_CPSR);
        // v0-v31
        for (int i = 0; i < 32; ++i) {
            gdbregs.regs[REG_V0 + 2 * i] = static_cast<uint64_t>(
                context->readFloatRegBits(i * 4 + 3)) << 32 |
                    context->readFloatRegBits(i * 4 + 2);
            gdbregs.regs[REG_V0 + 2 * i + 1] = static_cast<uint64_t>(
                context->readFloatRegBits(i * 4 + 1)) << 32 |
                    context->readFloatRegBits(i * 4 + 0);
        }
    } else {  // AArch32
        // R0-R15 supervisor mode
        // arm registers are 32 bits wide, gdb registers are 64 bits wide two
        // arm registers are packed into one gdb register (little endian)
        gdbregs.regs[REG_R0 + 0] = context->readIntReg(INTREG_R1) << 32 |
            context->readIntReg(INTREG_R0);
        gdbregs.regs[REG_R0 + 1] = context->readIntReg(INTREG_R3) << 32 |
            context->readIntReg(INTREG_R2);
        gdbregs.regs[REG_R0 + 2] = context->readIntReg(INTREG_R5) << 32 |
            context->readIntReg(INTREG_R4);
        gdbregs.regs[REG_R0 + 3] = context->readIntReg(INTREG_R7) << 32 |
            context->readIntReg(INTREG_R6);
        gdbregs.regs[REG_R0 + 4] = context->readIntReg(INTREG_R9) << 32 |
            context->readIntReg(INTREG_R8);
        gdbregs.regs[REG_R0 + 5] = context->readIntReg(INTREG_R11) << 32|
            context->readIntReg(INTREG_R10);
        gdbregs.regs[REG_R0 + 6] = context->readIntReg(INTREG_SP) << 32 |
            context->readIntReg(INTREG_R12);
        gdbregs.regs[REG_R0 + 7] = context->pcState().pc() << 32        |
            context->readIntReg(INTREG_LR);

        // CPSR
        gdbregs.regs[REG_CPSR]  = context->readMiscRegNoEffect(MISCREG_CPSR);

        // vfpv3/neon floating point registers (32 double or 64 float)

        gdbregs.regs[REG_F0] =
            static_cast<uint64_t>(context->readFloatRegBits(0)) << 32 |
            gdbregs.regs[REG_CPSR];

        for (int i = 1; i < (NumFloatV7ArchRegs>>1); ++i) {
            gdbregs.regs[i + REG_F0] =
                static_cast<uint64_t>(context->readFloatRegBits(2*i)) << 32 |
                context->readFloatRegBits(2*i-1);
        }

        // FPSCR
        gdbregs.regs[REG_FPSCR] = static_cast<uint64_t>(
            context->readMiscRegNoEffect(MISCREG_FPSCR)) << 32 |
                context->readFloatRegBits(NumFloatV7ArchRegs - 1);
    }
}

/*
 * Translate the GDB register format into the kernel debugger register
 * format.
 */
void
RemoteGDB::setregs()
{

    DPRINTF(GDBAcc, "setregs in remotegdb \n");
    if (inAArch64(context)) {  // AArch64
        // x0-x31
        for (int i = 0; i < 32; ++i) {
            context->setIntReg(INTREG_X0 + i, gdbregs.regs[REG_X0 + i]);
        }
        // pc
        context->pcState(gdbregs.regs[REG_PC_64]);
        // cpsr
        context->setMiscRegNoEffect(MISCREG_CPSR, gdbregs.regs[REG_CPSR_64]);
        // v0-v31
        for (int i = 0; i < 32; ++i) {
          context->setFloatRegBits(i * 4 + 3,
                                   gdbregs.regs[REG_V0 + 2 * i] >> 32);
          context->setFloatRegBits(i * 4 + 2,
                                   gdbregs.regs[REG_V0 + 2 * i]);
          context->setFloatRegBits(i * 4 + 1,
                                   gdbregs.regs[REG_V0 + 2 * i + 1] >> 32);
          context->setFloatRegBits(i * 4 + 0,
                                   gdbregs.regs[REG_V0 + 2 * i + 1]);
        }
    } else {  // AArch32
        // R0-R15 supervisor mode
        // arm registers are 32 bits wide, gdb registers are 64 bits wide
        // two arm registers are packed into one gdb register (little endian)
        context->setIntReg(INTREG_R0 , bits(gdbregs.regs[REG_R0 + 0], 31, 0));
        context->setIntReg(INTREG_R1 , bits(gdbregs.regs[REG_R0 + 0], 63, 32));
        context->setIntReg(INTREG_R2 , bits(gdbregs.regs[REG_R0 + 1], 31, 0));
        context->setIntReg(INTREG_R3 , bits(gdbregs.regs[REG_R0 + 1], 63, 32));
        context->setIntReg(INTREG_R4 , bits(gdbregs.regs[REG_R0 + 2], 31, 0));
        context->setIntReg(INTREG_R5 , bits(gdbregs.regs[REG_R0 + 2], 63, 32));
        context->setIntReg(INTREG_R6 , bits(gdbregs.regs[REG_R0 + 3], 31, 0));
        context->setIntReg(INTREG_R7 , bits(gdbregs.regs[REG_R0 + 3], 63, 32));
        context->setIntReg(INTREG_R8 , bits(gdbregs.regs[REG_R0 + 4], 31, 0));
        context->setIntReg(INTREG_R9 , bits(gdbregs.regs[REG_R0 + 4], 63, 32));
        context->setIntReg(INTREG_R10, bits(gdbregs.regs[REG_R0 + 5], 31, 0));
        context->setIntReg(INTREG_R11, bits(gdbregs.regs[REG_R0 + 5], 63, 32));
        context->setIntReg(INTREG_R12, bits(gdbregs.regs[REG_R0 + 6], 31, 0));
        context->setIntReg(INTREG_SP , bits(gdbregs.regs[REG_R0 + 6], 63, 32));
        context->setIntReg(INTREG_LR , bits(gdbregs.regs[REG_R0 + 7], 31, 0));
        context->pcState(bits(gdbregs.regs[REG_R0 + 7], 63, 32));

        //CPSR
        context->setMiscRegNoEffect(MISCREG_CPSR, gdbregs.regs[REG_CPSR]);

        //vfpv3/neon floating point registers (32 double or 64 float)
        context->setFloatRegBits(0, gdbregs.regs[REG_F0]>>32);

        for (int i = 1; i < NumFloatV7ArchRegs; ++i) {
            if (i%2) {
                int j = (i+1)/2;
                context->setFloatRegBits(i, bits(gdbregs.regs[j + REG_F0], 31, 0));
            } else {
                int j = i/2;
                context->setFloatRegBits(i, gdbregs.regs[j + REG_F0]>>32);
            }
        }

        //FPSCR
        context->setMiscReg(MISCREG_FPSCR, gdbregs.regs[REG_FPSCR]>>32);
    }
}

void
RemoteGDB::clearSingleStep()
{
    DPRINTF(GDBMisc, "clearSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt, notTakenBkpt);

    if (takenBkpt != 0)
        clearTempBreakpoint(takenBkpt);

    if (notTakenBkpt != 0)
        clearTempBreakpoint(notTakenBkpt);
}

void
RemoteGDB::setSingleStep()
{
    PCState pc = context->pcState();
    PCState bpc;
    bool set_bt = false;

    // User was stopped at pc, e.g. the instruction at pc was not
    // executed.
    MachInst inst = read<MachInst>(pc.pc());
    StaticInstPtr si = context->getDecoderPtr()->decode(inst, pc.pc());
    if (si->hasBranchTarget(pc, context, bpc)) {
        // Don't bother setting a breakpoint on the taken branch if it
        // is the same as the next pc
        if (bpc.pc() != pc.npc())
            set_bt = true;
    }

    DPRINTF(GDBMisc, "setSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt, notTakenBkpt);

    setTempBreakpoint(notTakenBkpt = pc.npc());

    if (set_bt)
        setTempBreakpoint(takenBkpt = bpc.pc());
}

// Write bytes to kernel address space for debugger.
bool
RemoteGDB::write(Addr vaddr, size_t size, const char *data)
{
    return BaseRemoteGDB::write(vaddr, size, data);
}

