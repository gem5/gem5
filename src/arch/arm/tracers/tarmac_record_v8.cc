/*
 * Copyright (c) 2017-2018 ARM Limited
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
 * Authors: Giacomo Travaglini
 */

#include "arch/arm/tracers/tarmac_record_v8.hh"

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/tracers/tarmac_tracer.hh"

namespace Trace {

TarmacTracerRecordV8::TraceInstEntryV8::TraceInstEntryV8(
    const TarmacContext& tarmCtx,
    bool predicate)
      : TraceInstEntry(tarmCtx, predicate),
        TraceEntryV8(tarmCtx.tarmacCpuName()),
        paddr(0),
        paddrValid(false)
{
    const auto thread = tarmCtx.thread;

    // Evaluate physical address
    TheISA::TLB* dtb = static_cast<TLB*>(thread->getDTBPtr());
    paddrValid = dtb->translateFunctional(thread, addr, paddr);
}

TarmacTracerRecordV8::TraceMemEntryV8::TraceMemEntryV8(
    const TarmacContext& tarmCtx,
    uint8_t _size, Addr _addr, uint64_t _data)
      : TraceMemEntry(tarmCtx, _size, _addr, _data),
        TraceEntryV8(tarmCtx.tarmacCpuName()),
        paddr(_addr)
{
    const auto thread = tarmCtx.thread;

    // Evaluate physical address
    TheISA::TLB* dtb = static_cast<TLB*>(thread->getDTBPtr());
    dtb->translateFunctional(thread, addr, paddr);
}

TarmacTracerRecordV8::TraceRegEntryV8::TraceRegEntryV8(
    const TarmacContext& tarmCtx,
    const RegId& reg)
      : TraceRegEntry(tarmCtx, reg),
        TraceEntryV8(tarmCtx.tarmacCpuName()),
        regWidth(64)
{
}

void
TarmacTracerRecordV8::TraceRegEntryV8::updateInt(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    // Do not trace pseudo register accesses: invalid
    // register entry.
    if (regRelIdx > NUM_ARCH_INTREGS) {
        regValid = false;
        return;
    }

    TraceRegEntry::updateInt(tarmCtx, regRelIdx);

    if ((regRelIdx != PCReg) || (regRelIdx != StackPointerReg) ||
        (regRelIdx != FramePointerReg) || (regRelIdx != ReturnAddressReg)) {

        const auto* arm_inst = static_cast<const ArmStaticInst*>(
            tarmCtx.staticInst.get()
        );

        regWidth = (arm_inst->getIntWidth());
        if (regWidth == 32) {
            regName = "W" + std::to_string(regRelIdx);
        } else {
            regName = "X" + std::to_string(regRelIdx);
        }
    }
}

void
TarmacTracerRecordV8::TraceRegEntryV8::updateMisc(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    TraceRegEntry::updateMisc(tarmCtx, regRelIdx);
    // System registers are 32bit wide
    regWidth = 32;
}

void
TarmacTracerRecordV8::addInstEntry(std::vector<InstPtr>& queue,
                                   const TarmacContext& tarmCtx)
{
    // Generate an instruction entry in the record and
    // add it to the Instruction Queue
    queue.push_back(
        m5::make_unique<TraceInstEntryV8>(tarmCtx, predicate)
    );
}

void
TarmacTracerRecordV8::addMemEntry(std::vector<MemPtr>& queue,
                                  const TarmacContext& tarmCtx)
{
    // Generate a memory entry in the record if the record
    // implies a valid memory access, and add it to the
    // Memory Queue
    if (getMemValid()) {
        queue.push_back(
            m5::make_unique<TraceMemEntryV8>(tarmCtx,
                                             static_cast<uint8_t>(getSize()),
                                             getAddr(), getIntData())
        );
    }
}

void
TarmacTracerRecordV8::addRegEntry(std::vector<RegPtr>& queue,
                                  const TarmacContext& tarmCtx)
{
    // Generate an entry for every ARM register being
    // written by the current instruction
    for (auto reg = 0; reg < staticInst->numDestRegs(); ++reg) {

        RegId reg_id = staticInst->destRegIdx(reg);

        // Creating a single register change entry
        auto single_reg = genRegister<TraceRegEntryV8>(tarmCtx, reg_id);

        // Copying the entry and adding it to the "list"
        // of entries to be dumped to trace.
        queue.push_back(
            m5::make_unique<TraceRegEntryV8>(single_reg)
        );
    }

    // Gem5 is treating CPSR flags as separate registers (CC registers),
    // in contrast with Tarmac specification: we need to merge the gem5 CC
    // entries altogether with the CPSR register and produce a single entry.
    mergeCCEntry<TraceRegEntryV8>(queue, tarmCtx);
}

void
TarmacTracerRecordV8::TraceInstEntryV8::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // If there is a valid vaddr->paddr translation, print the
    // physical address, otherwise print the virtual address only.
    std::string paddr_str = paddrValid? csprintf(":%012x",paddr) :
                                        std::string();

    // Pad the opcode.
    std::string opcode_str = csprintf("%0*x", instSize >> 2, opcode);

    // Print the instruction record formatted according
    // to the Tarmac specification
    ccprintf(outs, "%s clk %s %s (%u) %08x%s %s %s %s_%s : %s\n",
             curTick(),                     /* Tick time */
             cpuName,                       /* Cpu name */
             taken? "IT" : "IS",            /* Instruction taken/skipped */
             instCount,                     /* Instruction count */
             addr,                          /* Instruction virt address */
             paddr_str,                     /* Instruction phys address */
             opcode_str,                    /* Instruction opcode */
             iSetStateToStr(isetstate),     /* Instruction Set */
             opModeToStr(mode),             /* Exception level */
             secureMode? "s" : "ns",        /* Security */
             disassemble);                  /* Instruction disass */
}

void
TarmacTracerRecordV8::TraceMemEntryV8::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // Print the memory record formatted according
    // to the Tarmac specification
    ccprintf(outs, "%s clk %s M%s%d %08x:%012x %0*x\n",
             curTick(),                 /* Tick time */
             cpuName,                   /* Cpu name */
             loadAccess? "R" : "W",     /* Access type */
             size,                      /* Access size */
             addr,                      /* Virt Memory address */
             paddr,                     /* Phys Memory address */
             size*2,                    /* Padding with access size */
             data);                     /* Memory data */
}

void
TarmacTracerRecordV8::TraceRegEntryV8::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // Print the register record formatted according
    // to the Tarmac specification
    if (regValid) {
        ccprintf(outs, "%s clk %s R %s %0*x\n",
                 curTick(),            /* Tick time */
                 cpuName,              /* Cpu name */
                 regName,              /* Register name */
                 regWidth >> 2,        /* Register value padding */
                 valueLo);             /* Register value */
    }
}

} // namespace Trace
