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
#include "kern/kernel_stats.hh"
#include "mem/packet_impl.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

#if FULL_SYSTEM
#include "base/remote_gdb.hh"
#include "arch/tlb.hh"
#include "arch/stacktrace.hh"
#include "arch/vtophys.hh"
#else // !FULL_SYSTEM
#include "mem/mem_object.hh"
#endif // FULL_SYSTEM

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(Params *p)
    : BaseCPU(p), mem(p->mem), thread(NULL)
{
#if FULL_SYSTEM
    thread = new SimpleThread(this, 0, p->system, p->itb, p->dtb);
#else
    thread = new SimpleThread(this, /* thread_num */ 0, p->process,
            /* asid */ 0, mem);
#endif // !FULL_SYSTEM

    thread->setStatus(ThreadContext::Suspended);

    tc = thread->getTC();

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    threadContexts.push_back(tc);
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
    startNumInst = numInst;
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
                DPRINTF(IPI,"Suspended Processor awoke\n");
        thread->activate();
    }
}
#endif // FULL_SYSTEM

void
BaseSimpleCPU::checkForInterrupts()
{
#if FULL_SYSTEM
    if (checkInterrupts && check_interrupts() && !thread->inPalMode()) {
        int ipl = 0;
        int summary = 0;
        checkInterrupts = false;

        if (thread->readMiscReg(IPR_SIRR)) {
            for (int i = INTLEVEL_SOFTWARE_MIN;
                 i < INTLEVEL_SOFTWARE_MAX; i++) {
                if (thread->readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
                    // See table 4-19 of 21164 hardware reference
                    ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                    summary |= (ULL(1) << i);
                }
            }
        }

        uint64_t interrupts = thread->cpu->intr_status();
        for (int i = INTLEVEL_EXTERNAL_MIN;
            i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }

        if (thread->readMiscReg(IPR_ASTRR))
            panic("asynchronous traps not implemented\n");

        if (ipl && ipl > thread->readMiscReg(IPR_IPLR)) {
            thread->setMiscReg(IPR_ISR, summary);
            thread->setMiscReg(IPR_INTID, ipl);

            Fault(new InterruptFault)->invoke(tc);

            DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                    thread->readMiscReg(IPR_IPLR), ipl, summary);
        }
    }
#endif
}


Fault
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    // set up memory request for instruction fetch
#if ISA_HAS_DELAY_SLOT
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p NNPC:%08p\n",thread->readPC(),
            thread->readNextPC(),thread->readNextNPC());
#else
    DPRINTF(Fetch,"Fetch: PC:%08p NPC:%08p",thread->readPC(),
            thread->readNextPC());
#endif

    req->setVirt(0, thread->readPC() & ~3, sizeof(MachInst),
                 (FULL_SYSTEM && (thread->readPC() & 1)) ? PHYSICAL : 0,
                 thread->readPC());

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
    curStaticInst = StaticInst::decode(makeExtMI(inst, thread->readPC()));

    traceData = Trace::getInstRecord(curTick, tc, this, curStaticInst,
                                     thread->readPC());

    DPRINTF(Decode,"Decode: Decoded %s instruction (opcode: 0x%x): 0x%x\n",
            curStaticInst->getName(), curStaticInst->getOpcode(),
            curStaticInst->machInst);

#if FULL_SYSTEM
    thread->setInst(inst);
#endif // FULL_SYSTEM
}

void
BaseSimpleCPU::postExecute()
{
#if FULL_SYSTEM
    if (thread->profile) {
        bool usermode =
            (thread->readMiscReg(AlphaISA::IPR_DTB_CM) & 0x18) != 0;
        thread->profilePC = usermode ? 1 : thread->readPC();
        ProfileNode *node = thread->profile->consume(tc, inst);
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
        traceData->finalize();
    }
}


void
BaseSimpleCPU::advancePC(Fault fault)
{
    if (fault != NoFault) {
        fault->invoke(tc);
    }
    else {
        // go to the next instruction
        thread->setPC(thread->readNextPC());
#if ISA_HAS_DELAY_SLOT
        thread->setNextPC(thread->readNextNPC());
        thread->setNextNPC(thread->readNextNPC() + sizeof(MachInst));
        assert(thread->readNextPC() != thread->readNextNPC());
#else
        thread->setNextPC(thread->readNextPC() + sizeof(MachInst));
#endif

    }

#if FULL_SYSTEM
    Addr oldpc;
    do {
        oldpc = thread->readPC();
        system->pcEventQueue.service(tc);
    } while (oldpc != thread->readPC());
#endif
}

