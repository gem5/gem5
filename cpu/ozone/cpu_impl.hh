/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 */

#include <cstdio>
#include <cstdlib>

#include "arch/isa_traits.hh" // For MachInst
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/ozone/cpu.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/static_inst.hh"
#include "mem/base_mem.hh"
#include "mem/mem_interface.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

#if FULL_SYSTEM
#include "arch/faults.hh"
#include "arch/alpha/osfpal.hh"
#include "arch/alpha/tlb.hh"
#include "arch/vtophys.hh"
#include "base/callback.hh"
#include "base/remote_gdb.hh"
#include "cpu/profile.hh"
#include "kern/kernel_stats.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/faults.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"
#else // !FULL_SYSTEM
#include "mem/functional/functional.hh"
#include "sim/process.hh"
#endif // FULL_SYSTEM

using namespace TheISA;

template <class Impl>
template<typename T>
void
OzoneCPU<Impl>::trace_data(T data) {
    if (traceData) {
        traceData->setData(data);
    }
}

template <class Impl>
OzoneCPU<Impl>::TickEvent::TickEvent(OzoneCPU *c, int w)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c), width(w)
{
}

template <class Impl>
void
OzoneCPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
OzoneCPU<Impl>::TickEvent::description()
{
    return "OzoneCPU tick event";
}
/*
template <class Impl>
OzoneCPU<Impl>::ICacheCompletionEvent::ICacheCompletionEvent(OzoneCPU *_cpu)
    : Event(&mainEventQueue),
      cpu(_cpu)
{
}

template <class Impl>
void
OzoneCPU<Impl>::ICacheCompletionEvent::process()
{
    cpu->processICacheCompletion();
}

template <class Impl>
const char *
OzoneCPU<Impl>::ICacheCompletionEvent::description()
{
    return "OzoneCPU I-cache completion event";
}

template <class Impl>
OzoneCPU<Impl>::DCacheCompletionEvent::
DCacheCompletionEvent(OzoneCPU *_cpu,
                      DynInstPtr &_inst,
                      DCacheCompEventIt &_dcceIt)
    : Event(&mainEventQueue),
      cpu(_cpu),
      inst(_inst),
      dcceIt(_dcceIt)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
OzoneCPU<Impl>::DCacheCompletionEvent::process()
{
    inst->setCompleted();

    // Maybe remove the EA from the list of addrs?
    cpu->eaList.clearAddr(inst->seqNum, inst->getEA());
    cpu->dCacheCompList.erase(this->dcceIt);
}

template <class Impl>
const char *
OzoneCPU<Impl>::DCacheCompletionEvent::description()
{
    return "OzoneCPU D-cache completion event";
}
*/
template <class Impl>
OzoneCPU<Impl>::OzoneCPU(Params *p)
#if FULL_SYSTEM
    : BaseCPU(p), thread(this, 0, p->mem), tickEvent(this, p->width),
#else
    : BaseCPU(p), thread(this, 0, p->workload[0], 0), tickEvent(this, p->width),
#endif
      comm(5, 5)
{
    frontEnd = new FrontEnd(p);
    backEnd = new BackEnd(p);

    _status = Idle;
    thread.xcProxy = &xcProxy;

    thread.inSyscall = false;

    xcProxy.cpu = this;
    xcProxy.thread = &thread;

    thread.setStatus(ExecContext::Suspended);
#if FULL_SYSTEM
//    xc = new ExecContext(this, 0, p->system, p->itb, p->dtb, p->mem);

    /***** All thread state stuff *****/
    thread.cpu = this;
    thread.tid = 0;
    thread.mem = p->mem;

    thread.quiesceEvent = new EndQuiesceEvent(&xcProxy);

    system = p->system;
    itb = p->itb;
    dtb = p->dtb;
    memctrl = p->system->memctrl;
    physmem = p->system->physmem;

    if (p->profile) {
        thread.profile = new FunctionProfile(p->system->kernelSymtab);
        Callback *cb =
            new MakeCallback<OzoneXC,
            &OzoneXC::dumpFuncProfile>(&xcProxy);
        registerExitCallback(cb);
    }

    // let's fill with a dummy node for now so we don't get a segfault
    // on the first cycle when there's no node available.
    static ProfileNode dummyNode;
    thread.profileNode = &dummyNode;
    thread.profilePC = 3;

#else
//    xc = new ExecContext(this, /* thread_num */ 0, p->workload[0], /* asid */ 0);
    thread.cpu = this;
    thread.tid = 0;
    thread.process = p->workload[0];
//    thread.mem = thread.process->getMemory();
    thread.asid = 0;
#endif // !FULL_SYSTEM
/*
    icacheInterface = p->icache_interface;
    dcacheInterface = p->dcache_interface;

    cacheMemReq = new MemReq();
    cacheMemReq->xc = xc;
    cacheMemReq->asid = 0;
    cacheMemReq->data = new uint8_t[64];
*/
    numInst = 0;
    startNumInst = 0;
/*    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    issueWidth = p->issueWidth;
*/
    execContexts.push_back(&xcProxy);

    frontEnd->setCPU(this);
    backEnd->setCPU(this);

    frontEnd->setXC(&xcProxy);
    backEnd->setXC(&xcProxy);

    frontEnd->setThreadState(&thread);
    backEnd->setThreadState(&thread);

    frontEnd->setCommBuffer(&comm);
    backEnd->setCommBuffer(&comm);

    frontEnd->setBackEnd(backEnd);
    backEnd->setFrontEnd(frontEnd);

    decoupledFrontEnd = p->decoupledFrontEnd;

    globalSeqNum = 1;

    checkInterrupts = false;
/*
    fetchRedirBranch = true;
    fetchRedirExcp = true;

    // Need to initialize the rename maps, and the head and tail pointers.
    robHeadPtr = new DynInst(this);
    robTailPtr = new DynInst(this);

    robHeadPtr->setNextInst(robTailPtr);
//    robHeadPtr->setPrevInst(NULL);
//    robTailPtr->setNextInst(NULL);
    robTailPtr->setPrevInst(robHeadPtr);

    robHeadPtr->setCompleted();
    robTailPtr->setCompleted();

    for (int i = 0; i < ISA::TotalNumRegs; ++i) {
        renameTable[i] = new DynInst(this);
        commitTable[i] = new DynInst(this);

        renameTable[i]->setCompleted();
        commitTable[i]->setCompleted();
    }

#if FULL_SYSTEM
    for (int i = 0; i < ISA::NumIntRegs; ++i) {
        palShadowTable[i] = new DynInst(this);
        palShadowTable[i]->setCompleted();
    }
#endif

    // Size of cache block.
    cacheBlkSize = icacheInterface ? icacheInterface->getBlockSize() : 64;

    // Create mask to get rid of offset bits.
    cacheBlkMask = (cacheBlkSize - 1);

    // Get the size of an instruction.
    instSize = sizeof(MachInst);

    // Create space to store a cache line.
    cacheData = new uint8_t[cacheBlkSize];

    cacheBlkValid = false;
*/
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        thread.renameTable[i] = new DynInst(this);
        thread.renameTable[i]->setCompleted();
    }

    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);

#if !FULL_SYSTEM
    pTable = p->pTable;
#endif

    DPRINTF(OzoneCPU, "OzoneCPU: Created Ozone cpu object.\n");
}

template <class Impl>
OzoneCPU<Impl>::~OzoneCPU()
{
}
/*
template <class Impl>
void
OzoneCPU<Impl>::copyFromXC()
{
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        if (i < TheISA::NumIntRegs) {
            renameTable[i]->setIntResult(xc->readIntReg(i));
        } else if (i < TheISA::NumFloatRegs) {
            renameTable[i]->setDoubleResult(xc->readFloatRegDouble(i));
        }
    }

    DPRINTF(OzoneCPU, "Func Exe inst is: %i\n", xc->func_exe_inst);
    backEnd->funcExeInst = xc->func_exe_inst;
//    PC = xc->readPC();
//    nextPC = xc->regs.npc;
}

template <class Impl>
void
OzoneCPU<Impl>::copyToXC()
{
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        if (i < TheISA::NumIntRegs) {
            xc->setIntReg(i, renameTable[i]->readIntResult());
        } else if (i < TheISA::NumFloatRegs) {
            xc->setFloatRegDouble(i, renameTable[i]->readDoubleResult());
        }
    }

    this->xc->regs.miscRegs.fpcr = this->regFile.miscRegs[tid].fpcr;
    this->xc->regs.miscRegs.uniq = this->regFile.miscRegs[tid].uniq;
    this->xc->regs.miscRegs.lock_flag = this->regFile.miscRegs[tid].lock_flag;
    this->xc->regs.miscRegs.lock_addr = this->regFile.miscRegs[tid].lock_addr;

    xc->func_exe_inst = backEnd->funcExeInst;
    xc->regs.pc = PC;
    xc->regs.npc = nextPC;
}
*/
template <class Impl>
void
OzoneCPU<Impl>::switchOut()
{
    _status = SwitchedOut;
    if (tickEvent.scheduled())
        tickEvent.squash();
}

template <class Impl>
void
OzoneCPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // if any of this CPU's ExecContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        if (xc->status() == ExecContext::Active &&
            _status != Running) {
            _status = Running;
            tickEvent.schedule(curTick);
        }
    }
}

template <class Impl>
void
OzoneCPU<Impl>::activateContext(int thread_num, int delay)
{
    // Eventually change this in SMT.
    assert(thread_num == 0);
//    assert(xcProxy);

    assert(_status == Idle);
    notIdleFraction++;
    scheduleTickEvent(delay);
    _status = Running;
    thread._status = ExecContext::Active;
}

template <class Impl>
void
OzoneCPU<Impl>::suspendContext(int thread_num)
{
    // Eventually change this in SMT.
    assert(thread_num == 0);
//    assert(xcProxy);

    assert(_status == Running);
    notIdleFraction--;
    unscheduleTickEvent();
    _status = Idle;
}

template <class Impl>
void
OzoneCPU<Impl>::deallocateContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

template <class Impl>
void
OzoneCPU<Impl>::haltContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}

template <class Impl>
void
OzoneCPU<Impl>::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    thread.numInsts
        .name(name() + ".num_insts")
        .desc("Number of instructions executed")
        ;

    thread.numMemRefs
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

    idleFraction = constant(1.0) - notIdleFraction;

    frontEnd->regStats();
    backEnd->regStats();
}

template <class Impl>
void
OzoneCPU<Impl>::resetStats()
{
    startNumInst = numInst;
    notIdleFraction = (_status != Idle);
}

template <class Impl>
void
OzoneCPU<Impl>::init()
{
    BaseCPU::init();
/*
    copyFromXC();

    // ALso copy over PC/nextPC.  This isn't normally copied in "copyFromXC()"
    // so that the XC doesn't mess up the PC when returning from a syscall.
    PC = xc->readPC();
    nextPC = xc->regs.npc;
*/
    // Mark this as in syscall so it won't need to squash
    thread.inSyscall = true;
#if FULL_SYSTEM
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];

        // initialize CPU, including PC
        TheISA::initCPU(xc, xc->readCpuId());
    }
#endif
    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);

    thread.inSyscall = false;
}

template <class Impl>
void
OzoneCPU<Impl>::serialize(std::ostream &os)
{
    // At this point, all DCacheCompEvents should be processed.

    BaseCPU::serialize(os);
    SERIALIZE_ENUM(_status);
    nameOut(os, csprintf("%s.xc", name()));
    xcProxy.serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
}

template <class Impl>
void
OzoneCPU<Impl>::unserialize(Checkpoint *cp, const std::string &section)
{
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_ENUM(_status);
    xcProxy.unserialize(cp, csprintf("%s.xc", section));
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

template <class Impl>
Fault
OzoneCPU<Impl>::copySrcTranslate(Addr src)
{
    panic("Copy not implemented!\n");
    return NoFault;
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    int offset = src & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (src & TheISA::PageMask) != ((src + blk_size) & TheISA::PageMask) &&
        (src >> 40) != 0xfffffc) {
        warn("Copied block source spans pages %x.", src);
        no_warn = false;
    }

    memReq->reset(src & ~(blk_size - 1), blk_size);

    // translate to physical address
    Fault fault = xc->translateDataReadReq(memReq);

    assert(fault != Alignment_Fault);

    if (fault == NoFault) {
        xc->copySrcAddr = src;
        xc->copySrcPhysAddr = memReq->paddr + offset;
    } else {
        xc->copySrcAddr = 0;
        xc->copySrcPhysAddr = 0;
    }
    return fault;
#endif
}

template <class Impl>
Fault
OzoneCPU<Impl>::copy(Addr dest)
{
    panic("Copy not implemented!\n");
    return NoFault;
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    uint8_t data[blk_size];
    //assert(xc->copySrcAddr);
    int offset = dest & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (dest & TheISA::PageMask) != ((dest + blk_size) & TheISA::PageMask) &&
        (dest >> 40) != 0xfffffc) {
        no_warn = false;
        warn("Copied block destination spans pages %x. ", dest);
    }

    memReq->reset(dest & ~(blk_size -1), blk_size);
    // translate to physical address
    Fault fault = xc->translateDataWriteReq(memReq);

    assert(fault != Alignment_Fault);

    if (fault == NoFault) {
        Addr dest_addr = memReq->paddr + offset;
        // Need to read straight from memory since we have more than 8 bytes.
        memReq->paddr = xc->copySrcPhysAddr;
        xc->mem->read(memReq, data);
        memReq->paddr = dest_addr;
        xc->mem->write(memReq, data);
        if (dcacheInterface) {
            memReq->cmd = Copy;
            memReq->completionEvent = NULL;
            memReq->paddr = xc->copySrcPhysAddr;
            memReq->dest = dest_addr;
            memReq->size = 64;
            memReq->time = curTick;
            dcacheInterface->access(memReq);
        }
    }
    return fault;
#endif
}

#if FULL_SYSTEM
template <class Impl>
Addr
OzoneCPU<Impl>::dbg_vtophys(Addr addr)
{
    return vtophys(&xcProxy, addr);
}
#endif // FULL_SYSTEM
/*
template <class Impl>
void
OzoneCPU<Impl>::processICacheCompletion()
{
    switch (status()) {
      case IcacheMiss:
        DPRINTF(OzoneCPU, "OzoneCPU: Finished Icache miss.\n");

        icacheStallCycles += curTick - lastIcacheStall;
        _status = IcacheMissComplete;
        cacheBlkValid = true;
//	scheduleTickEvent(1);
        break;
      case SwitchedOut:
        // If this CPU has been switched out due to sampling/warm-up,
        // ignore any further status changes (e.g., due to cache
        // misses outstanding at the time of the switch).
        return;
      default:
        panic("OzoneCPU::processICacheCompletion: bad state");
        break;
    }
}
*/
#if FULL_SYSTEM
template <class Impl>
void
OzoneCPU<Impl>::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (thread._status == ExecContext::Suspended) {
        DPRINTF(IPI,"Suspended Processor awoke\n");
//	thread.activate();
        // Hack for now.  Otherwise might have to go through the xcProxy, or
        // I need to figure out what's the right thing to call.
        activateContext(thread.tid, 1);
    }
}
#endif // FULL_SYSTEM

/* start simulation, program loaded, processor precise state initialized */
template <class Impl>
void
OzoneCPU<Impl>::tick()
{
    DPRINTF(OzoneCPU, "\n\nOzoneCPU: Ticking cpu.\n");

    thread.renameTable[ZeroReg]->setIntResult(0);
    thread.renameTable[ZeroReg+TheISA::FP_Base_DepTag]->
        setDoubleResult(0.0);

    // General code flow:
    // Check for any interrupts.  Handle them if I do have one.
    // Check if I have a need to fetch a new cache block.  Either a bit could be
    // set by functions indicating that I need to fetch a new block, or I could
    // hang onto the last PC of the last cache block I fetched and compare the
    // current PC to that.  Setting a bit seems nicer but may be more error
    // prone.
    // Scan through the IQ to figure out if there's anything I can issue/execute
    // Might need something close to the FU Pools to tell what instructions
    // I can issue.  How to handle loads and stores vs other insts?
    // Extremely slow way: find first inst that can possibly issue; if it's a
    // load or a store, then iterate through load/store queue.
    // If I can't find instructions to execute and I've got room in the IQ
    // (which is just a counter), then grab a few instructions out of the cache
    // line buffer until I either run out or can execute up until my limit.

    numCycles++;

    traceData = NULL;

//    Fault fault = NoFault;

#if 0 // FULL_SYSTEM
    if (checkInterrupts && check_interrupts() && !inPalMode() &&
        status() != IcacheMissComplete) {
        int ipl = 0;
        int summary = 0;
        checkInterrupts = false;

        if (readMiscReg(IPR_SIRR)) {
            for (int i = INTLEVEL_SOFTWARE_MIN;
                 i < INTLEVEL_SOFTWARE_MAX; i++) {
                if (readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
                    // See table 4-19 of 21164 hardware reference
                    ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                    summary |= (ULL(1) << i);
                }
            }
        }

        // Is this method so that if the interrupts are switched over from
        // another CPU they'll still be handled?
//	uint64_t interrupts = cpuXC->cpu->intr_status();
        uint64_t interrupts = intr_status();
        for (int i = INTLEVEL_EXTERNAL_MIN;
            i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }

        if (readMiscReg(IPR_ASTRR))
            panic("asynchronous traps not implemented\n");

        if (ipl && ipl > readMiscReg(IPR_IPLR)) {
            setMiscReg(IPR_ISR, summary);
            setMiscReg(IPR_INTID, ipl);

            Fault(new InterruptFault)->invoke(xc);

            DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                    readMiscReg(IPR_IPLR), ipl, summary);
        }
    }
#endif

    // Make call to ISA to ensure 0 register semantics...actually because the
    // DynInsts will generally be the register file, this should only have to
    // happen when the xc is actually written to (during a syscall or something)
    // maintain $r0 semantics
//    assert(renameTable[ZeroReg]->readIntResult() == 0);
#ifdef TARGET_ALPHA
//    assert(renameTable[ZeroReg]->readDoubleResult() == 0);
#endif // TARGET_ALPHA

    comm.advance();
    frontEnd->tick();
    backEnd->tick();

    // Do this here?  For now the front end will control the PC.
//    PC = nextPC;

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);

    if (!tickEvent.scheduled())
        tickEvent.schedule(curTick + 1);
}

template <class Impl>
void
OzoneCPU<Impl>::squashFromXC()
{
    thread.inSyscall = true;
    backEnd->squashFromXC();
}

#if !FULL_SYSTEM
template <class Impl>
void
OzoneCPU<Impl>::syscall()
{
    // Not sure this copy is needed, depending on how the XC proxy is made.
    thread.renameTable.copyFrom(backEnd->renameTable);

    thread.inSyscall = true;

    thread.funcExeInst++;

    DPRINTF(OzoneCPU, "FuncExeInst: %i\n", thread.funcExeInst);

    thread.process->syscall(&xcProxy);

    thread.funcExeInst--;

    thread.inSyscall = false;

    frontEnd->renameTable.copyFrom(thread.renameTable);
    backEnd->renameTable.copyFrom(thread.renameTable);
}

template <class Impl>
void
OzoneCPU<Impl>::setSyscallReturn(SyscallReturn return_value, int tid)
{
    // check for error condition.  Alpha syscall convention is to
    // indicate success/failure in reg a3 (r19) and put the
    // return value itself in the standard return value reg (v0).
    if (return_value.successful()) {
        // no error
        thread.renameTable[SyscallSuccessReg]->setIntResult(0);
        thread.renameTable[ReturnValueReg]->setIntResult(return_value.value());
    } else {
        // got an error, return details
        thread.renameTable[SyscallSuccessReg]->setIntResult((IntReg) -1);
        thread.renameTable[ReturnValueReg]->setIntResult(-return_value.value());
    }
}
#else
template <class Impl>
Fault
OzoneCPU<Impl>::hwrei()
{
    // Need to move this to ISA code
    // May also need to make this per thread
    if (!inPalMode())
        return new UnimplementedOpcodeFault;

    thread.setNextPC(thread.readMiscReg(AlphaISA::IPR_EXC_ADDR));

    // Not sure how to make a similar check in the Ozone model
//    if (!misspeculating()) {
        kernelStats->hwrei();

        checkInterrupts = true;
//    }

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
bool
OzoneCPU<Impl>::simPalCheck(int palFunc)
{
    // Need to move this to ISA code
    // May also need to make this per thread
    this->kernelStats->callpal(palFunc, &xcProxy);

    switch (palFunc) {
      case PAL::halt:
        haltContext(thread.tid);
        if (--System::numSystemsRunning == 0)
            new SimExitEvent("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (system->breakpoint())
            return false;
        break;
    }

    return true;
}
#endif

template <class Impl>
BaseCPU *
OzoneCPU<Impl>::OzoneXC::getCpuPtr()
{
    return cpu;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setCpuId(int id)
{
    cpu->cpuId = id;
    thread->cpuId = id;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setStatus(Status new_status)
{
//    cpu->_status = new_status;
    thread->_status = new_status;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::activate(int delay)
{
    cpu->activateContext(thread->tid, delay);
}

/// Set the status to Suspended.
template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::suspend()
{
    cpu->suspendContext(thread->tid);
}

/// Set the status to Unallocated.
template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::deallocate()
{
    cpu->deallocateContext(thread->tid);
}

/// Set the status to Halted.
template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::halt()
{
    cpu->haltContext(thread->tid);
}

#if FULL_SYSTEM
template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::dumpFuncProfile()
{ }
#endif

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::takeOverFrom(ExecContext *old_context)
{ }

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::regStats(const std::string &name)
{ }

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::serialize(std::ostream &os)
{ }

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::unserialize(Checkpoint *cp, const std::string &section)
{ }

#if FULL_SYSTEM
template <class Impl>
Event *
OzoneCPU<Impl>::OzoneXC::getQuiesceEvent()
{
    return thread->quiesceEvent;
}

template <class Impl>
Tick
OzoneCPU<Impl>::OzoneXC::readLastActivate()
{
    return thread->lastActivate;
}

template <class Impl>
Tick
OzoneCPU<Impl>::OzoneXC::readLastSuspend()
{
    return thread->lastSuspend;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::profileClear()
{
    if (thread->profile)
        thread->profile->clear();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::profileSample()
{
    if (thread->profile)
        thread->profile->sample(thread->profileNode, thread->profilePC);
}
#endif

template <class Impl>
int
OzoneCPU<Impl>::OzoneXC::getThreadNum()
{
    return thread->tid;
}

// Also somewhat obnoxious.  Really only used for the TLB fault.
template <class Impl>
TheISA::MachInst
OzoneCPU<Impl>::OzoneXC::getInst()
{
    return thread->inst;
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::copyArchRegs(ExecContext *xc)
{
    thread->PC = xc->readPC();
    thread->nextPC = xc->readNextPC();

    cpu->frontEnd->setPC(thread->PC);
    cpu->frontEnd->setNextPC(thread->nextPC);

    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        if (i < TheISA::FP_Base_DepTag) {
            thread->renameTable[i]->setIntResult(xc->readIntReg(i));
        } else if (i < (TheISA::FP_Base_DepTag + TheISA::NumFloatRegs)) {
            int fp_idx = i - TheISA::FP_Base_DepTag;
            thread->renameTable[i]->setDoubleResult(
                xc->readFloatRegDouble(fp_idx));
        }
    }

#if !FULL_SYSTEM
    thread->funcExeInst = xc->readFuncExeInst();
#endif

    // Need to copy the XC values into the current rename table,
    // copy the misc regs.
    thread->regs.miscRegs.copyMiscRegs(xc);
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::clearArchRegs()
{
    panic("Unimplemented!");
}

template <class Impl>
uint64_t
OzoneCPU<Impl>::OzoneXC::readIntReg(int reg_idx)
{
    return thread->renameTable[reg_idx]->readIntResult();
}

template <class Impl>
float
OzoneCPU<Impl>::OzoneXC::readFloatRegSingle(int reg_idx)
{
    return thread->renameTable[reg_idx]->readFloatResult();
}

template <class Impl>
double
OzoneCPU<Impl>::OzoneXC::readFloatRegDouble(int reg_idx)
{
    return thread->renameTable[reg_idx]->readDoubleResult();
}

template <class Impl>
uint64_t
OzoneCPU<Impl>::OzoneXC::readFloatRegInt(int reg_idx)
{
    return thread->renameTable[reg_idx]->readIntResult();
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setIntReg(int reg_idx, uint64_t val)
{
    thread->renameTable[reg_idx]->setIntResult(val);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setFloatRegSingle(int reg_idx, float val)
{
    panic("Unimplemented!");
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setFloatRegDouble(int reg_idx, double val)
{
    thread->renameTable[reg_idx]->setDoubleResult(val);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setFloatRegInt(int reg_idx, uint64_t val)
{
    panic("Unimplemented!");
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setPC(Addr val)
{
    thread->PC = val;
    cpu->frontEnd->setPC(val);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }
}

template <class Impl>
void
OzoneCPU<Impl>::OzoneXC::setNextPC(Addr val)
{
    thread->nextPC = val;
    cpu->frontEnd->setNextPC(val);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }
}

template <class Impl>
TheISA::MiscReg
OzoneCPU<Impl>::OzoneXC::readMiscReg(int misc_reg)
{
    return thread->regs.miscRegs.readReg(misc_reg);
}

template <class Impl>
TheISA::MiscReg
OzoneCPU<Impl>::OzoneXC::readMiscRegWithEffect(int misc_reg, Fault &fault)
{
    return thread->regs.miscRegs.readRegWithEffect(misc_reg,
                                                   fault, this);
}

template <class Impl>
Fault
OzoneCPU<Impl>::OzoneXC::setMiscReg(int misc_reg, const MiscReg &val)
{
    // Needs to setup a squash event unless we're in syscall mode
    Fault ret_fault = thread->regs.miscRegs.setReg(misc_reg, val);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }

    return ret_fault;
}

template <class Impl>
Fault
OzoneCPU<Impl>::OzoneXC::setMiscRegWithEffect(int misc_reg, const MiscReg &val)
{
    // Needs to setup a squash event unless we're in syscall mode
    Fault ret_fault = thread->regs.miscRegs.setRegWithEffect(misc_reg, val,
                                                             this);

    if (!thread->inSyscall) {
        cpu->squashFromXC();
    }

    return ret_fault;
}
