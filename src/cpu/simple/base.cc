/*
 * Copyright (c) 2010 ARM Limited
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
 * Authors: Steve Reinhardt
 */

#include "arch/faults.hh"
#include "arch/utility.hh"
#include "base/cp_annotate.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/range.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/profile.hh"
#include "cpu/simple/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/BaseSimpleCPU.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

#if FULL_SYSTEM
#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "arch/tlb.hh"
#include "arch/vtophys.hh"
#else // !FULL_SYSTEM
#include "mem/mem_object.hh"
#endif // FULL_SYSTEM

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(BaseSimpleCPUParams *p)
    : BaseCPU(p), traceData(NULL), thread(NULL), predecoder(NULL)
{
#if FULL_SYSTEM
    thread = new SimpleThread(this, 0, p->system, p->itb, p->dtb);
#else
    thread = new SimpleThread(this, /* thread_num */ 0, p->workload[0],
            p->itb, p->dtb);
#endif // !FULL_SYSTEM

    thread->setStatus(ThreadContext::Halted);

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
     notIdleFraction = (_status != Idle);
}

void
BaseSimpleCPU::serialize(ostream &os)
{
    SERIALIZE_ENUM(_status);
    BaseCPU::serialize(os);
//    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc.0", name()));
    thread->serialize(os);
}

void
BaseSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_ENUM(_status);
    BaseCPU::unserialize(cp, section);
//    UNSERIALIZE_SCALAR(inst);
    thread->unserialize(cp, csprintf("%s.xc.0", section));
}

void
change_thread_state(ThreadID tid, int activate, int priority)
{
}

void
BaseSimpleCPU::prefetch(Addr addr, unsigned flags)
{
    if (traceData) {
        traceData->setAddr(addr);
    }

    // need to do this...
}

void
BaseSimpleCPU::writeHint(Addr addr, int size, unsigned flags)
{
    if (traceData) {
        traceData->setAddr(addr);
    }

    // need to do this...
}


Fault
BaseSimpleCPU::copySrcTranslate(Addr src)
{
#if 0
    static bool no_warn = true;
    unsigned blk_size =
        (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
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
    unsigned blk_size =
        (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
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
BaseSimpleCPU::wakeup()
{
    if (thread->status() != ThreadContext::Suspended)
        return;

    DPRINTF(Quiesce,"Suspended Processor awoke\n");
    thread->activate();
}
#endif // FULL_SYSTEM

void
BaseSimpleCPU::checkForInterrupts()
{
#if FULL_SYSTEM
    if (checkInterrupts(tc)) {
        Fault interrupt = interrupts->getInterrupt(tc);

        if (interrupt != NoFault) {
            fetchOffset = 0;
            interrupts->updateIntrInfo(tc);
            interrupt->invoke(tc);
            predecoder.reset();
        }
    }
#endif
}


void
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    Addr threadPC = thread->readPC();

    // set up memory request for instruction fetch
#if ISA_HAS_DELAY_SLOT
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p NNPC:%08p\n",threadPC,
            thread->readNextPC(),thread->readNextNPC());
#else
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p\n",threadPC,
            thread->readNextPC());
#endif

    Addr fetchPC = (threadPC & PCMask) + fetchOffset;
    req->setVirt(0, fetchPC, sizeof(MachInst), Request::INST_FETCH, threadPC);
}


void
BaseSimpleCPU::preExecute()
{
    // maintain $r0 semantics
    thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    thread->setFloatReg(ZeroReg, 0.0);
#endif // ALPHA_ISA

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);

    // decode the instruction
    inst = gtoh(inst);

    MicroPC upc = thread->readMicroPC();

    if (isRomMicroPC(upc)) {
        stayAtPC = false;
        curStaticInst = microcodeRom.fetchMicroop(upc, curMacroStaticInst);
    } else if (!curMacroStaticInst) {
        //We're not in the middle of a macro instruction
        StaticInstPtr instPtr = NULL;

        //Predecode, ie bundle up an ExtMachInst
        //This should go away once the constructor can be set up properly
        predecoder.setTC(thread->getTC());
        //If more fetch data is needed, pass it in.
        Addr fetchPC = (thread->readPC() & PCMask) + fetchOffset;
        //if(predecoder.needMoreBytes())
            predecoder.moreBytes(thread->readPC(), fetchPC, inst);
        //else
        //    predecoder.process();

        //If an instruction is ready, decode it. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        if (predecoder.extMachInstReady()) {
#if THE_ISA == X86_ISA || THE_ISA == ARM_ISA
            thread->setNextPC(thread->readPC() + predecoder.getInstSize());
#endif // X86_ISA
            stayAtPC = false;
            instPtr = StaticInst::decode(predecoder.getExtMachInst(),
                                         thread->readPC());
        } else {
            stayAtPC = true;
            fetchOffset += sizeof(MachInst);
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroop()) {
            curMacroStaticInst = instPtr;
            curStaticInst = curMacroStaticInst->fetchMicroop(upc);
        } else {
            curStaticInst = instPtr;
        }
    } else {
        //Read the next micro op from the macro op
        curStaticInst = curMacroStaticInst->fetchMicroop(upc);
    }

    //If we decoded an instruction this "tick", record information about it.
    if(curStaticInst)
    {
#if TRACING_ON
        traceData = tracer->getInstRecord(curTick, tc,
                curStaticInst, thread->readPC(),
                curMacroStaticInst, thread->readMicroPC());

        DPRINTF(Decode,"Decode: Decoded %s instruction: 0x%x\n",
                curStaticInst->getName(), curStaticInst->machInst);
#endif // TRACING_ON

#if FULL_SYSTEM
        thread->setInst(inst);
#endif // FULL_SYSTEM
    }
}

void
BaseSimpleCPU::postExecute()
{
#if FULL_SYSTEM
    if (thread->profile && curStaticInst) {
        bool usermode = TheISA::inUserMode(tc);
        thread->profilePC = usermode ? 1 : thread->readPC();
        ProfileNode *node = thread->profile->consume(tc, curStaticInst);
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

    if (CPA::available()) {
        CPA::cpa()->swAutoBegin(tc, thread->readNextPC());
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
        predecoder.reset();
    } else {
        //If we're at the last micro op for this instruction
        if (curStaticInst && curStaticInst->isLastMicroop()) {
            //We should be working with a macro op or be in the ROM
            assert(curMacroStaticInst ||
                    isRomMicroPC(thread->readMicroPC()));
            //Close out this macro op, and clean up the
            //microcode state
            curMacroStaticInst = StaticInst::nullStaticInstPtr;
            thread->setMicroPC(normalMicroPC(0));
            thread->setNextMicroPC(normalMicroPC(1));
        }
        //If we're still in a macro op
        if (curMacroStaticInst || isRomMicroPC(thread->readMicroPC())) {
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
}

/*Fault
BaseSimpleCPU::CacheOp(uint8_t Op, Addr EffAddr)
{
    // translate to physical address
    Fault fault = NoFault;
    int CacheID = Op & 0x3; // Lower 3 bits identify Cache
    int CacheOP = Op >> 2; // Upper 3 bits identify Cache Operation
    if(CacheID > 1)
      {
        warn("CacheOps not implemented for secondary/tertiary caches\n");
      }
    else
      {
        switch(CacheOP)
          { // Fill Packet Type
          case 0: warn("Invalidate Cache Op\n");
            break;
          case 1: warn("Index Load Tag Cache Op\n");
            break;
          case 2: warn("Index Store Tag Cache Op\n");
            break;
          case 4: warn("Hit Invalidate Cache Op\n");
            break;
          case 5: warn("Fill/Hit Writeback Invalidate Cache Op\n");
            break;
          case 6: warn("Hit Writeback\n");
            break;
          case 7: warn("Fetch & Lock Cache Op\n");
            break;
          default: warn("Unimplemented Cache Op\n");
          }
      }
    return fault;
}*/
