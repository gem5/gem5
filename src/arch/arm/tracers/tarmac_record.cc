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

#include "arch/arm/tracers/tarmac_record.hh"

#include "arch/arm/insts/static_inst.hh"
#include "tarmac_tracer.hh"

namespace Trace {

// TARMAC Instruction Record static variables
uint64_t TarmacTracerRecord::TraceInstEntry::instCount = 0;

std::string
iSetStateToStr(TarmacBaseRecord::ISetState isetstate)
{
    switch (isetstate) {
      case TarmacBaseRecord::ISET_ARM:
        return "A";
      case TarmacBaseRecord::ISET_THUMB:
        return "T";
      case TarmacBaseRecord::ISET_A64:
        return "O";
      default:
        return "Unsupported";
    }
}

std::string
opModeToStr(OperatingMode opMode)
{
    switch (opMode) {
      case MODE_EL0T:
        return "EL0t";
      case MODE_EL1T:
        return "EL1t";
      case MODE_EL1H:
        return "EL1h";
      case MODE_EL2T:
        return "EL2t";
      case MODE_EL2H:
        return "EL2h";
      case MODE_EL3T:
        return "EL3t";
      case MODE_EL3H:
        return "EL3h";
      case MODE_USER:
        return "usr";
      case MODE_FIQ:
        return "fiq";
      case MODE_IRQ:
        return "irq";
      case MODE_SVC:
        return "svc";
      case MODE_MON:
        return "mon";
      case MODE_ABORT:
        return "abt";
      case MODE_HYP:
        return "hyp";
      case MODE_UNDEFINED:
        return "und";
      case MODE_SYSTEM:
        return "sys";
      default:
        return "Unsupported";
    }
}

// TarmacTracerRecord ctor
TarmacTracerRecord::TarmacTracerRecord(Tick _when, ThreadContext *_thread,
                                     const StaticInstPtr _staticInst,
                                     PCState _pc,
                                     TarmacTracer& _tracer,
                                     const StaticInstPtr _macroStaticInst)
    : TarmacBaseRecord(_when, _thread, _staticInst,
                       _pc,  _macroStaticInst),
      tracer(_tracer)
{
}

TarmacTracerRecord::TraceInstEntry::TraceInstEntry(
    const TarmacContext& tarmCtx,
    bool predicate)
      : InstEntry(tarmCtx.thread, tarmCtx.pc, tarmCtx.staticInst, predicate)
{
    secureMode = inSecureState(tarmCtx.thread);

    auto arm_inst = static_cast<const ArmStaticInst*>(
        tarmCtx.staticInst.get()
    );

    // Get the instruction size as a number of bits:
    // (multiply byte size by 8)
    instSize = (arm_inst->instSize() << 3);

    // Mask the opcode using the instruction size: the
    // opcode field will otherwise be 32 bit wide even
    // for 16bit (Thumb) instruction.
    opcode = arm_inst->encoding();

    // Update the instruction count: number of executed
    // instructions.
    instCount++;
}

TarmacTracerRecord::TraceMemEntry::TraceMemEntry(
    const TarmacContext& tarmCtx,
    uint8_t _size, Addr _addr, uint64_t _data)
      :  MemEntry(_size, _addr, _data),
         loadAccess(tarmCtx.staticInst->isLoad())
{
}

TarmacTracerRecord::TraceRegEntry::TraceRegEntry(
    const TarmacContext& tarmCtx,
    const RegId& reg)
      : RegEntry(tarmCtx.pc),
        regValid(false),
        regClass(reg.classValue()),
        regRel(reg.index())
{
}

void
TarmacTracerRecord::TraceRegEntry::update(
    const TarmacContext& tarmCtx
)
{
    // Fill the register entry data, according to register
    // class.
    switch (regClass) {
      case CCRegClass:
        updateCC(tarmCtx, regRel);
        break;
      case FloatRegClass:
        updateFloat(tarmCtx, regRel);
        break;
      case IntRegClass:
        updateInt(tarmCtx, regRel);
        break;
      case MiscRegClass:
        updateMisc(tarmCtx, regRel);
        break;
      default:
        // If unsupported format, do nothing: non updating
        // the register will prevent it to be printed.
        break;
    }
}

void
TarmacTracerRecord::TraceRegEntry::updateMisc(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    auto thread = tarmCtx.thread;

    regValid = true;
    regName = miscRegName[regRelIdx];
    valueLo = thread->readMiscRegNoEffect(regRelIdx);

    // If it is the CPSR:
    // update the value of the CPSR register and add
    // the CC flags on top of the value
    if (regRelIdx == MISCREG_CPSR) {
        CPSR cpsr = thread->readMiscRegNoEffect(MISCREG_CPSR);
        cpsr.nz = thread->readCCReg(CCREG_NZ);
        cpsr.c = thread->readCCReg(CCREG_C);
        cpsr.v = thread->readCCReg(CCREG_V);
        cpsr.ge = thread->readCCReg(CCREG_GE);

        // update the entry value
        valueLo = cpsr;
    }
}

void
TarmacTracerRecord::TraceRegEntry::updateCC(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    auto thread = tarmCtx.thread;

    regValid = true;
    regName = ccRegName[regRelIdx];
    valueLo = thread->readCCReg(regRelIdx);
}

void
TarmacTracerRecord::TraceRegEntry::updateFloat(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    auto thread = tarmCtx.thread;

    regValid = true;
    regName  = "f" + std::to_string(regRelIdx);
    valueLo = bitsToFloat32(thread->readFloatReg(regRelIdx));
}

void
TarmacTracerRecord::TraceRegEntry::updateInt(
    const TarmacContext& tarmCtx,
    RegIndex regRelIdx
)
{
    auto thread = tarmCtx.thread;

    // Reading operating mode from CPSR.
    // This is needed when printing the register name in case
    // of banked register (e.g. lr_svc)
    CPSR cpsr = thread->readMiscRegNoEffect(MISCREG_CPSR);
    OperatingMode mode = (OperatingMode)(uint8_t)cpsr.mode;

    std::string reg_suffix;
    if (mode != MODE_USER) {
        reg_suffix = "_"  + opModeToStr(mode);
    }

    regValid = true;
    switch (regRelIdx) {
      case PCReg:
        regName = "pc";
        break;
      case StackPointerReg:
        regName = "sp" + reg_suffix ;
        break;
      case FramePointerReg:
        regName = "fp" + reg_suffix;
        break;
      case ReturnAddressReg:
        regName = "lr" + reg_suffix;
        break;
      default:
        regName  = "r" + std::to_string(regRelIdx);
        break;
    }
    valueLo = thread->readIntReg(regRelIdx);
}

void
TarmacTracerRecord::addInstEntry(std::vector<InstPtr>& queue,
                                 const TarmacContext& tarmCtx)
{
    // Generate an instruction entry in the record and
    // add it to the Instruction Queue
    queue.push_back(
        m5::make_unique<TraceInstEntry>(tarmCtx, predicate)
    );
}

void
TarmacTracerRecord::addMemEntry(std::vector<MemPtr>& queue,
                                const TarmacContext& tarmCtx)
{
    // Generate a memory entry in the record if the record
    // implies a valid memory access, and add it to the
    // Memory Queue
    if (getMemValid()) {
        queue.push_back(
            m5::make_unique<TraceMemEntry>(tarmCtx,
                                           static_cast<uint8_t>(getSize()),
                                           getAddr(), getIntData())
        );
    }
}

void
TarmacTracerRecord::addRegEntry(std::vector<RegPtr>& queue,
                                const TarmacContext& tarmCtx)
{
    // Generate an entry for every ARM register being
    // written by the current instruction
    for (auto reg = 0; reg < staticInst->numDestRegs(); ++reg) {

        RegId reg_id = staticInst->destRegIdx(reg);

        // Creating a single register change entry
        auto single_reg = genRegister<TraceRegEntry>(tarmCtx, reg_id);

        // Copying the entry and adding it to the "list"
        // of entries to be dumped to trace.
        queue.push_back(
            m5::make_unique<TraceRegEntry>(single_reg)
        );
    }

    // Gem5 is treating CPSR flags as separate registers (CC registers),
    // in contrast with Tarmac specification: we need to merge the gem5 CC
    // entries altogether with the CPSR register and produce a single entry.
    mergeCCEntry<TraceRegEntry>(queue, tarmCtx);
}

void
TarmacTracerRecord::dump()
{
    // Generate and print all the record's entries.
    auto &instQueue = tracer.instQueue;
    auto &memQueue = tracer.memQueue;
    auto &regQueue = tracer.regQueue;

    const TarmacContext tarmCtx(
        thread,
        staticInst->isMicroop()? macroStaticInst : staticInst,
        pc
    );

    if (!staticInst->isMicroop()) {
        // Current instruction is NOT a micro-instruction:
        // Generate Tarmac entries and dump them immediately

        // Generate Tarmac entries and add them to the respective
        // queues.
        addInstEntry(instQueue, tarmCtx);
        addMemEntry(memQueue, tarmCtx);
        addRegEntry(regQueue, tarmCtx);

        // Flush (print) any queued entry.
        flushQueues(instQueue, memQueue, regQueue);

    } else {
        // Current instruction is a micro-instruction:
        // save micro entries into dedicated queues and flush them
        // into the tracefile only when the MACRO-instruction
        // has completed.

        if (staticInst->isFirstMicroop()) {
            addInstEntry(instQueue, tarmCtx);
        }

        addRegEntry(regQueue, tarmCtx);
        addMemEntry(memQueue, tarmCtx);

        if (staticInst->isLastMicroop()) {
            // Flush (print) any queued entry.
            flushQueues(instQueue, memQueue, regQueue);
        }
    }
}

template<typename Queue>
void
TarmacTracerRecord::flushQueues(Queue& queue)
{
    std::ostream &outs = Trace::output();

    for (const auto &single_entry : queue) {
        single_entry->print(outs);
    }

    queue.clear();
}

template<typename Queue, typename... Args>
void
TarmacTracerRecord::flushQueues(Queue& queue, Args & ... args)
{
    flushQueues(queue);
    flushQueues(args...);
}

void
TarmacTracerRecord::TraceInstEntry::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // Pad the opcode
    std::string opcode_str = csprintf("%0*x", instSize >> 2, opcode);

    // Print the instruction record formatted according
    // to the Tarmac specification
    ccprintf(outs, "%s clk %s (%u) %08x %s %s %s_%s : %s\n",
             curTick(),                   /* Tick time */
             taken? "IT" : "IS",          /* Instruction taken/skipped */
             instCount,                   /* Instruction count */
             addr,                        /* Instruction address */
             opcode_str,                  /* Instruction opcode */
             iSetStateToStr(isetstate),   /* Instruction Set */
             opModeToStr(mode),           /* Exception level */
             secureMode? "s" : "ns",      /* Security */
             disassemble);                /* Instruction disass */
}

void
TarmacTracerRecord::TraceMemEntry::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // Print the memory record formatted according
    // to the Tarmac specification
    ccprintf(outs, "%s clk M%s%d %08x %0*x\n",
             curTick(),                 /* Tick time */
             loadAccess? "R" : "W",     /* Access type */
             size,                      /* Access size */
             addr,                      /* Memory address */
             size*2,                    /* Padding with access size */
             data);                     /* Memory data */
}

void
TarmacTracerRecord::TraceRegEntry::print(
    std::ostream& outs,
    int verbosity,
    const std::string &prefix) const
{
    // Print the register record formatted according
    // to the Tarmac specification
    if (regValid)
        ccprintf(outs, "%s clk R %s %08x\n",
                 curTick(),                 /* Tick time */
                 regName,                   /* Register name */
                 valueLo);                  /* Register value */
}

} // namespace Trace
