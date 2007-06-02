/*
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
 * Authors: Steve Reinhardt
 */

#include "arch/utility.hh"
#include "arch/faults.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/range.hh"
#include "base/stats/events.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/profile.hh"
#include "cpu/simple/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "mem/packet.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

#if FULL_SYSTEM
#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "arch/tlb.hh"
#include "arch/vtophys.hh"
#include "base/remote_gdb.hh"
#else // !FULL_SYSTEM
#include "mem/mem_object.hh"
#endif // FULL_SYSTEM

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(Params *p)
    : BaseCPU(p), traceData(NULL), thread(NULL), predecoder(NULL)
{
#if FULL_SYSTEM
    thread = new SimpleThread(this, 0, p->system, p->itb, p->dtb);
#else
    thread = new SimpleThread(this, /* thread_num */ 0, p->process,
            /* asid */ 0);
#endif // !FULL_SYSTEM

    thread->setStatus(ThreadContext::Unallocated);

    tc = thread->getTC();

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    threadContexts.push_back(tc);

    fetchOffset = 0;
    stayAtPC = false;
}

BaseSimpleCPU::~BaseSimpleCPU()
{
}

void
BaseSimpleCPU::deallocateContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
BaseSimpleCPU::haltContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
BaseSimpleCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    numInsts
        .name(name() + ".num_insts")
        .desc("Number of instructions executed")
        ;

    numMemRefs
        .name(name() + ".num_refs")
        .desc("Number of memory references")
        ;

    notIdleFraction
        .name(name() + ".not_idle_fraction")
        .desc("Percentage of non-idle cycles")
        ;

    idleFraction
        .name(name() + ".idle_fraction")
        .desc("Percentage of idle cycles")
        ;

    icacheStallCycles
        .name(name() + ".icache_stall_cycles")
        .desc("ICache total stall cycles")
        .prereq(icacheStallCycles)
        ;

    dcacheStallCycles
        .name(name() + ".dcache_stall_cycles")
        .desc("DCache total stall cycles")
        .prereq(dcacheStallCycles)
        ;

    icacheRetryCycles
        .name(name() + ".icache_retry_cycles")
        .desc("ICache total retry cycles")
        .prereq(icacheRetryCycles)
        ;

    dcacheRetryCycles
        .name(name() + ".dcache_retry_cycles")
        .desc("DCache total retry cycles")
        .prereq(dcacheRetryCycles)
        ;

    idleFraction = constant(1.0) - notIdleFraction;
}

void
BaseSimpleCPU::resetStats()
{
//    startNumInst = numInst;
    // notIdleFraction = (_status != Idle);
}

void
BaseSimpleCPU::serialize(ostream &os)
{
    BaseCPU::serialize(os);
//    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc.0", name()));
    thread->serialize(os);
}

void
BaseSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    BaseCPU::unserialize(cp, section);
//    UNSERIALIZE_SCALAR(inst);
    thread->unserialize(cp, csprintf("%s.xc.0", section));
}

void
change_thread_state(int thread_number, int activate, int priority)
{
}

Fault
BaseSimpleCPU::copySrcTranslate(Addr src)
{
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    int offset = src & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (src & PageMask) != ((src + blk_size) & PageMask) &&
        (src >> 40) != 0xfffffc) {
        warn("Copied block source spans pages %x.", src);
        no_warn = false;
    }

    memReq->reset(src & ~(blk_size - 1), blk_size);

    // translate to physical address
    Fault fault = thread->translateDataReadReq(req);

    if (fault == NoFault) {
        thread->copySrcAddr = src;
        thread->copySrcPhysAddr = memReq->paddr + offset;
    } else {
        assert(!fault->isAlignmentFault());

        thread->copySrcAddr = 0;
        thread->copySrcPhysAddr = 0;
    }
    return fault;
#else
    return NoFault;
#endif
}

Fault
BaseSimpleCPU::copy(Addr dest)
{
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    uint8_t data[blk_size];
    //assert(thread->copySrcAddr);
    int offset = dest & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (dest & PageMask) != ((dest + blk_size) & PageMask) &&
        (dest >> 40) != 0xfffffc) {
        no_warn = false;
        warn("Copied block destination spans pages %x. ", dest);
    }

    memReq->reset(dest & ~(blk_size -1), blk_size);
    // translate to physical address
    Fault fault = thread->translateDataWriteReq(req);

    if (fault == NoFault) {
        Addr dest_addr = memReq->paddr + offset;
        // Need to read straight from memory since we have more than 8 bytes.
        memReq->paddr = thread->copySrcPhysAddr;
        thread->mem->read(memReq, data);
        memReq->paddr = dest_addr;
        thread->mem->write(memReq, data);
        if (dcacheInterface) {
            memReq->cmd = Copy;
            memReq->completionEvent = NULL;
            memReq->paddr = thread->copySrcPhysAddr;
            memReq->dest = dest_addr;
            memReq->size = 64;
            memReq->time = curTick;
            memReq->flags &= ~INST_READ;
            dcacheInterface->access(memReq);
        }
    }
    else
        assert(!fault->isAlignmentFault());

    return fault;
#else
    panic("copy not implemented");
    return NoFault;
#endif
}

#if FULL_SYSTEM
Addr
BaseSimpleCPU::dbg_vtophys(Addr addr)
{
    return vtophys(tc, addr);
}
#endif // FULL_SYSTEM

#if FULL_SYSTEM
void
BaseSimpleCPU::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (thread->status() == ThreadContext::Suspended) {
                DPRINTF(Quiesce,"Suspended Processor awoke\n");
        thread->activate();
    }
}
#endif // FULL_SYSTEM

void
BaseSimpleCPU::checkForInterrupts()
{
#if FULL_SYSTEM
    if (check_interrupts(tc)) {
        Fault interrupt = interrupts.getInterrupt(tc);

        if (interrupt != NoFault) {
            interrupts.updateIntrInfo(tc);
            interrupt->invoke(tc);
        }
    }
#endif
}


Fault
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    Addr threadPC = thread->readPC();

    // set up memory request for instruction fetch
#if ISA_HAS_DELAY_SLOT
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p NNPC:%08p\n",threadPC,
            thread->readNextPC(),thread->readNextNPC());
#else
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p",threadPC,
            thread->readNextPC());
#endif

    const Addr PCMask = ~((Addr)sizeof(MachInst) - 1);
    Addr fetchPC = threadPC + fetchOffset;
    req->setVirt(0, fetchPC & PCMask, sizeof(MachInst), 0, threadPC);

    Fault fault = thread->translateInstReq(req);

    return fault;
}


void
BaseSimpleCPU::preExecute()
{
    // maintain $r0 semantics
    thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    thread->setFloatReg(ZeroReg, 0.0);
#endif // ALPHA_ISA

    // keep an instruction count
    numInst++;
    numInsts++;

    thread->funcExeInst++;

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);

    // decode the instruction
    inst = gtoh(inst);

    //If we're not in the middle of a macro instruction
    if (!curMacroStaticInst) {

        StaticInstPtr instPtr = NULL;

        //Predecode, ie bundle up an ExtMachInst
        //This should go away once the constructor can be set up properly
        predecoder.setTC(thread->getTC());
        //If more fetch data is needed, pass it in.
        if(predecoder.needMoreBytes())
            predecoder.moreBytes(thread->readPC() + fetchOffset, 0, inst);
        else
            predecoder.process();

        //If an instruction is ready, decode it. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        if (predecoder.extMachInstReady()) {
#if THE_ISA == X86_ISA
            thread->setNextPC(thread->readPC() + predecoder.getInstSize());
#endif // X86_ISA
            stayAtPC = false;
            instPtr = StaticInst::decode(predecoder.getExtMachInst());
        } else {
            stayAtPC = true;
            fetchOffset += sizeof(MachInst);
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroOp()) {
            curMacroStaticInst = instPtr;
            curStaticInst = curMacroStaticInst->
                fetchMicroOp(thread->readMicroPC());
        } else {
            curStaticInst = instPtr;
        }
    } else {
        //Read the next micro op from the macro op
        curStaticInst = curMacroStaticInst->
            fetchMicroOp(thread->readMicroPC());
    }

#if TRACING_ON
    //If we decoded an instruction this "tick", record information about it.
    if(curStaticInst)
    {
        traceData = Trace::getInstRecord(curTick, tc, curStaticInst,
                                         thread->readPC());

        DPRINTF(Decode,"Decode: Decoded %s instruction: 0x%x\n",
                curStaticInst->getName(), curStaticInst->machInst);

#if FULL_SYSTEM
        thread->setInst(inst);
#endif // FULL_SYSTEM
    }
#endif // TRACING_ON
}

void
BaseSimpleCPU::postExecute()
{
#if FULL_SYSTEM
    if (thread->profile) {
        bool usermode = TheISA::inUserMode(tc);
        thread->profilePC = usermode ? 1 : thread->readPC();
        StaticInstPtr si(inst);
        ProfileNode *node = thread->profile->consume(tc, si);
        if (node)
            thread->profileNode = node;
    }
#endif

    if (curStaticInst->isMemRef()) {
        numMemRefs++;
    }

    if (curStaticInst->isLoad()) {
        ++numLoad;
        comLoadEventQueue[0]->serviceEvents(numLoad);
    }

    traceFunctions(thread->readPC());

    if (traceData) {
        traceData->dump();
        delete traceData;
        traceData = NULL;
    }
}


void
BaseSimpleCPU::advancePC(Fault fault)
{
    //Since we're moving to a new pc, zero out the offset
    fetchOffset = 0;
    if (fault != NoFault) {
        curMacroStaticInst = StaticInst::nullStaticInstPtr;
        fault->invoke(tc);
        thread->setMicroPC(0);
        thread->setNextMicroPC(1);
    } else {
        //If we're at the last micro op for this instruction
        if (curStaticInst && curStaticInst->isLastMicroOp()) {
            //We should be working with a macro op
            assert(curMacroStaticInst);
            //Close out this macro op, and clean up the
            //microcode state
            curMacroStaticInst = StaticInst::nullStaticInstPtr;
            thread->setMicroPC(0);
            thread->setNextMicroPC(1);
        }
        //If we're still in a macro op
        if (curMacroStaticInst) {
            //Advance the micro pc
            thread->setMicroPC(thread->readNextMicroPC());
            //Advance the "next" micro pc. Note that there are no delay
            //slots, and micro ops are "word" addressed.
            thread->setNextMicroPC(thread->readNextMicroPC() + 1);
        } else {
            // go to the next instruction
            thread->setPC(thread->readNextPC());
            thread->setNextPC(thread->readNextNPC());
            thread->setNextNPC(thread->readNextNPC() + sizeof(MachInst));
            assert(thread->readNextPC() != thread->readNextNPC());
        }
    }

#if FULL_SYSTEM
    Addr oldpc;
    do {
        oldpc = thread->readPC();
        system->pcEventQueue.service(tc);
    } while (oldpc != thread->readPC());
#endif
}

