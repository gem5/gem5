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
 */

//#include <cmath>
#include <cstdio>
//#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <list>
//#include <sstream>
#include <string>

//#include "base/cprintf.hh"
//#include "base/inifile.hh"
//#include "base/loader/symtab.hh"
#include "base/misc.hh"
//#include "base/pollevent.hh"
//#include "base/range.hh"
#include "base/refcnt.hh"
//#include "base/stats/events.hh"
#include "cpu/base.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/exec_context.hh"
//#include "cpu/exetrace.hh"
//#include "cpu/profile.hh"
#include "cpu/sampler/sampler.hh"
//#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
//#include "kern/kernel_stats.hh"
#include "mem/base_mem.hh"
#include "mem/mem_interface.hh"
#include "sim/byteswap.hh"
#include "sim/builder.hh"
//#include "sim/debug.hh"
//#include "sim/host.hh"
//#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

#include "cpu/o3/alpha_dyn_inst.hh"
#include "cpu/o3/alpha_impl.hh"

#include "cpu/ozone/dyn_inst.hh"
#include "cpu/ozone/ozone_impl.hh"
#include "cpu/ozone/simple_impl.hh"

#if FULL_SYSTEM
#include "base/remote_gdb.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/system.hh"
#include "arch/tlb.hh"
#include "arch/stacktrace.hh"
#include "arch/vtophys.hh"
#else // !FULL_SYSTEM
#include "mem/functional/functional.hh"
#endif // FULL_SYSTEM

using namespace std;
//The CheckerCPU does alpha only
using namespace AlphaISA;

void
CheckerCPU::init()
{
/*
    BaseCPU::init();
#if FULL_SYSTEM
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];

        // initialize CPU, including PC
        TheISA::initCPU(xc, xc->readCpuId());
    }
#endif
*/
}

CheckerCPU::CheckerCPU(Params *p)
    : BaseCPU(p), cpuXC(NULL), xcProxy(NULL)
{
    memReq = new MemReq();
    memReq->xc = xcProxy;
    memReq->asid = 0;
    memReq->data = new uint8_t[64];

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    youngestSN = 0;

    changedPC = willChangePC = changedNextPC = false;

    exitOnError = p->exitOnError;
#if FULL_SYSTEM
    itb = p->itb;
    dtb = p->dtb;
    systemPtr = NULL;
    memPtr = NULL;
#endif
}

CheckerCPU::~CheckerCPU()
{
}

void
CheckerCPU::setMemory(FunctionalMemory *mem)
{
    memPtr = mem;
#if !FULL_SYSTEM
    cpuXC = new CPUExecContext(this, /* thread_num */ 0, mem,
                               /* asid */ 0);

    cpuXC->setStatus(ExecContext::Suspended);
    xcProxy = cpuXC->getProxy();
    execContexts.push_back(xcProxy);
#else
    if (systemPtr) {
        cpuXC = new CPUExecContext(this, 0, systemPtr, itb, dtb, memPtr);

        cpuXC->setStatus(ExecContext::Suspended);
        xcProxy = cpuXC->getProxy();
        execContexts.push_back(xcProxy);
        memReq->xc = xcProxy;
    }
#endif
}

#if FULL_SYSTEM
void
CheckerCPU::setSystem(System *system)
{
    systemPtr = system;

    if (memPtr) {
        cpuXC = new CPUExecContext(this, 0, systemPtr, itb, dtb, memPtr);

        cpuXC->setStatus(ExecContext::Suspended);
        xcProxy = cpuXC->getProxy();
        execContexts.push_back(xcProxy);
        memReq->xc = xcProxy;
    }
}
#endif

void
CheckerCPU::serialize(ostream &os)
{
/*
    BaseCPU::serialize(os);
    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc", name()));
    cpuXC->serialize(os);
    cacheCompletionEvent.serialize(os);
*/
}

void
CheckerCPU::unserialize(Checkpoint *cp, const string &section)
{
/*
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_SCALAR(inst);
    cpuXC->unserialize(cp, csprintf("%s.xc", section));
*/
}

Fault
CheckerCPU::copySrcTranslate(Addr src)
{
    static bool no_warn = true;
    int blk_size = 64;
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
    Fault fault = cpuXC->translateDataReadReq(memReq);

    if (fault == NoFault) {
        cpuXC->copySrcAddr = src;
        cpuXC->copySrcPhysAddr = memReq->paddr + offset;
    } else {
        assert(!fault->isAlignmentFault());

        cpuXC->copySrcAddr = 0;
        cpuXC->copySrcPhysAddr = 0;
    }
    return fault;
}

Fault
CheckerCPU::copy(Addr dest)
{
    static bool no_warn = true;
    int blk_size = 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    uint8_t data[blk_size];
    //assert(cpuXC->copySrcAddr);
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
    Fault fault = cpuXC->translateDataWriteReq(memReq);

    if (fault == NoFault) {
        Addr dest_addr = memReq->paddr + offset;
        // Need to read straight from memory since we have more than 8 bytes.
        memReq->paddr = cpuXC->copySrcPhysAddr;
        cpuXC->mem->read(memReq, data);
        memReq->paddr = dest_addr;
        cpuXC->mem->write(memReq, data);
        memReq->cmd = Copy;
        memReq->completionEvent = NULL;
        memReq->paddr = cpuXC->copySrcPhysAddr;
        memReq->dest = dest_addr;
        memReq->size = 64;
        memReq->time = curTick;
        memReq->flags &= ~INST_READ;
    }
    else
        assert(!fault->isAlignmentFault());

    return fault;
}

// precise architected memory state accessor macros
template <class T>
Fault
CheckerCPU::read(Addr addr, T &data, unsigned flags)
{
    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    // Should I probe the DTB?  Or should I just take the physical address
    // and assume correct translation?
    translateDataReadReq(memReq);

    // if we have a cache, do cache access too
    memReq->cmd = Read;
    memReq->completionEvent = NULL;
    memReq->time = curTick;
    memReq->flags &= ~INST_READ;

    if (!(memReq->flags & UNCACHEABLE)) {
        cpuXC->read(memReq, data);
    } else {
        // Assume the data is correct if it's an uncached access
        memcpy(&data, &unverifiedResult.integer, sizeof(T));
    }

    return NoFault;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
CheckerCPU::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
CheckerCPU::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
CheckerCPU::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}

template<>
Fault
CheckerCPU::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}

template <class T>
Fault
CheckerCPU::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    cpuXC->translateDataWriteReq(memReq);

    if ((!(unverifiedReq->flags & LOCKED) ||
        ((unverifiedReq->flags & LOCKED) &&
         unverifiedReq->result == 1)) &&
        !(unverifiedReq->flags & UNCACHEABLE)) {
        // do functional access
//        cpuXC->read(memReq, data);

        memReq->cmd = Write;
//    memcpy(memReq->data,(uint8_t *)&data,memReq->size);
        T inst_data;
        memcpy(&inst_data, unverifiedReq->data, sizeof(T));
        memReq->completionEvent = NULL;
        memReq->time = curTick;
        memReq->flags &= ~INST_READ;

        // Hard to verify this as the data writes back after the
        // instruction commits.  May only be able to check that the
        // value produced from execute() matches the value produced
        // from the instruction's first execution.
        if (data != inst_data) {
            warn("Store value does not match value in memory! "
                 "Instruction: %#x, memory: %#x",
                 inst_data, data);
            handleError();
        }
    }

    // Assume the result was the same as the one passed in.  This checker
    // doesn't check if the SC should succeed or fail, it just checks the
    // value.
    if (res)
        *res = unverifiedReq->result;

    return NoFault;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
CheckerCPU::write(uint64_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint32_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint16_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint8_t data, Addr addr, unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
CheckerCPU::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
CheckerCPU::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}

template<>
Fault
CheckerCPU::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


#if FULL_SYSTEM
Addr
CheckerCPU::dbg_vtophys(Addr addr)
{
    return vtophys(xcProxy, addr);
}
#endif // FULL_SYSTEM

#if FULL_SYSTEM
void
CheckerCPU::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (cpuXC->status() == ExecContext::Suspended) {
                DPRINTF(IPI,"Suspended Processor awoke\n");
        cpuXC->activate();
    }
}
#endif // FULL_SYSTEM

bool
CheckerCPU::translateInstReq(MemReqPtr &req)
{
#if FULL_SYSTEM
    return (cpuXC->translateInstReq(req) == NoFault);
#else
    cpuXC->translateInstReq(req);
    return true;
#endif
}

void
CheckerCPU::translateDataReadReq(MemReqPtr &req)
{
    cpuXC->translateDataReadReq(req);

    if (req->vaddr != unverifiedReq->vaddr) {
        warn("Request virtual addresses do not match! Inst: %#x, checker:"
             " %#x",
             unverifiedReq->vaddr, req->vaddr);
    }
    req->paddr = unverifiedReq->paddr;

    if (checkFlags(req)) {
        warn("Request flags do not match! Inst: %#x, checker: %#x",
             unverifiedReq->flags, req->flags);
        handleError();
    }
}

void
CheckerCPU::translateDataWriteReq(MemReqPtr &req)
{
    cpuXC->translateDataWriteReq(req);

    if (req->vaddr != unverifiedReq->vaddr) {
        warn("Request virtual addresses do not match! Inst: %#x, checker:"
             " %#x",
             unverifiedReq->vaddr, req->vaddr);
    }
    req->paddr = unverifiedReq->paddr;

    if (checkFlags(req)) {
        warn("Request flags do not match! Inst: %#x, checker: %#x",
             unverifiedReq->flags, req->flags);
        handleError();
    }
}

bool
CheckerCPU::checkFlags(MemReqPtr &req)
{
    // Remove any dynamic flags that don't have to do with the request itself.
    unsigned flags = unverifiedReq->flags;
    unsigned mask = LOCKED | PHYSICAL | VPTE | ALTMODE | UNCACHEABLE | NO_FAULT;
    flags = flags & (mask);
    if (flags == req->flags) {
        return false;
    } else {
        return true;
    }
}

/* start simulation, program loaded, processor precise state initialized */
template <class DynInstPtr>
void
Checker<DynInstPtr>::tick(DynInstPtr &completed_inst)
{
    DynInstPtr inst;

    if (!instList.empty()) {
        if (youngestSN < completed_inst->seqNum) {
            DPRINTF(Checker, "Adding instruction [sn:%lli] PC:%#x to list.\n",
                    completed_inst->seqNum, completed_inst->readPC());
            instList.push_back(completed_inst);
            youngestSN = completed_inst->seqNum;
        }

        if (!instList.front()->isCompleted()) {
            return;
        } else {
            inst = instList.front();
            instList.pop_front();
        }
    } else {
        if (!completed_inst->isCompleted()) {
            if (youngestSN < completed_inst->seqNum) {
                DPRINTF(Checker, "Adding instruction [sn:%lli] PC:%#x to list.\n",
                        completed_inst->seqNum, completed_inst->readPC());
                instList.push_back(completed_inst);
                youngestSN = completed_inst->seqNum;
            }
            return;
        } else {
            if (youngestSN < completed_inst->seqNum) {
                inst = completed_inst;
                youngestSN = completed_inst->seqNum;
            } else {
//                panic("SN already seen yet the list is empty!");
                return;
            }
        }
    }

    while (1) {
        DPRINTF(Checker, "Processing instruction [sn:%lli] PC:%#x.\n",
                inst->seqNum, inst->readPC());
//    verifyInst = completed_inst;
        unverifiedResult.integer = inst->readIntResult();
        unverifiedReq = inst->req;
        numCycles++;

        Fault fault = NoFault;

        // maintain $r0 semantics
        cpuXC->setIntReg(ZeroReg, 0);
#ifdef TARGET_ALPHA
        cpuXC->setFloatRegDouble(ZeroReg, 0.0);
#endif // TARGET_ALPHA

        // Try to fetch an instruction

        // set up memory request for instruction fetch
#if FULL_SYSTEM
#define IFETCH_FLAGS(pc)	((pc) & 1) ? PHYSICAL : 0
#else
#define IFETCH_FLAGS(pc)	0
#endif

        if (changedPC) {
            DPRINTF(Checker, "Changed PC recently to %#x\n",
                    cpuXC->readPC());
            if (willChangePC) {
                if (newPC == cpuXC->readPC()) {
                    DPRINTF(Checker, "Changed PC matches expected PC\n");
                } else {
                    warn("Changed PC does not match expected PC, changed: %#x, "
                         "expected: %#x",
                         cpuXC->readPC(), newPC);
                    handleError();
                }
                willChangePC = false;
            }
            changedPC = false;
        }
        if (changedNextPC) {
            DPRINTF(Checker, "Changed NextPC recently to %#x\n",
                    cpuXC->readNextPC());
            changedNextPC = false;
        }

        memReq->cmd = Read;
        memReq->reset(cpuXC->readPC() & ~3, sizeof(uint32_t),
                      IFETCH_FLAGS(cpuXC->readPC()));

        bool succeeded = translateInstReq(memReq);

        if (!succeeded) {
            if (inst->getFault() == NoFault) {
                warn("Instruction PC %#x was not found in the ITB!",
                     cpuXC->readPC());
                handleError();

                // go to the next instruction
                cpuXC->setPC(cpuXC->readNextPC());
                cpuXC->setNextPC(cpuXC->readNextPC() + sizeof(MachInst));

                return;
            } else {
                fault = inst->getFault();
            }
        }

        if (fault == NoFault) {
//        fault = cpuXC->mem->read(memReq, machInst);
            cpuXC->mem->read(memReq, machInst);

            // If we've got a valid instruction (i.e., no fault on instruction
            // fetch), then execute it.

        // keep an instruction count
            numInst++;
//	numInsts++;

            // decode the instruction
            machInst = gtoh(machInst);
            // Checks that the instruction matches what we expected it to be.
            // Checks both the machine instruction and the PC.
            validateInst(inst);

            curStaticInst = StaticInst::decode(makeExtMI(machInst, cpuXC->readPC()));

#if FULL_SYSTEM
            cpuXC->setInst(machInst);
#endif // FULL_SYSTEM

            fault = inst->getFault();
        }

        // Either the instruction was a fault and we should process the fault,
        // or we should just go ahead execute the instruction.  This assumes
        // that the instruction is properly marked as a fault.
        if (fault == NoFault) {

            cpuXC->func_exe_inst++;

            fault = curStaticInst->execute(this, NULL);

            // Checks to make sure instrution results are correct.
            validateExecution(inst);

//	if (curStaticInst->isMemRef()) {
//	    numMemRefs++;
//	}

            if (curStaticInst->isLoad()) {
                ++numLoad;
            }
        }

        if (fault != NoFault) {
#if FULL_SYSTEM
            fault->invoke(xcProxy);
            willChangePC = true;
            newPC = cpuXC->readPC();
            DPRINTF(Checker, "Fault, PC is now %#x\n", newPC);
#else // !FULL_SYSTEM
            fatal("fault (%d) detected @ PC 0x%08p", fault, cpuXC->readPC());
#endif // FULL_SYSTEM
        } else {
#if THE_ISA != MIPS_ISA
            // go to the next instruction
            cpuXC->setPC(cpuXC->readNextPC());
            cpuXC->setNextPC(cpuXC->readNextPC() + sizeof(MachInst));
#else
            // go to the next instruction
            cpuXC->setPC(cpuXC->readNextPC());
            cpuXC->setNextPC(cpuXC->readNextNPC());
            cpuXC->setNextNPC(cpuXC->readNextNPC() + sizeof(MachInst));
#endif

        }

#if FULL_SYSTEM
        Addr oldpc;
        int count = 0;
        do {
            oldpc = cpuXC->readPC();
            system->pcEventQueue.service(xcProxy);
            count++;
        } while (oldpc != cpuXC->readPC());
        if (count > 1) {
            willChangePC = true;
            newPC = cpuXC->readPC();
            DPRINTF(Checker, "PC Event, PC is now %#x\n", newPC);
        }
#endif

        // Checks PC, next PC.  Optionally can check all registers. (Or just those
        // that have been modified).
        validateState();

        if (instList.empty()) {
            break;
        } else if (instList.front()->isCompleted()) {
            inst = instList.front();
            instList.pop_front();
        } else {
            break;
        }
    }
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::switchOut(Sampler *s)
{
    sampler = s;
    instList.clear();
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::takeOverFrom(BaseCPU *oldCPU)
{
//    BaseCPU::takeOverFrom(oldCPU);

    // if any of this CPU's ExecContexts are active, mark the CPU as
    // running and schedule its tick event.
/*
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
    }
*/
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::validateInst(DynInstPtr &inst)
{
    if (inst->readPC() != cpuXC->readPC()) {
        warn("PCs do not match! Inst: %#x, checker: %#x",
             inst->readPC(), cpuXC->readPC());
        if (changedPC) {
            warn("Changed PCs recently, may not be an error");
        } else {
            handleError();
        }
    }

    if (static_cast<MachInst>(inst->staticInst->machInst) !=
        machInst) {
        warn("Binary instructions do not match! Inst: %#x, checker: %#x",
             static_cast<MachInst>(inst->staticInst->machInst),
             machInst);
        handleError();
    }
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::validateExecution(DynInstPtr &inst)
{
    if (inst->numDestRegs()) {
        if (inst->isUnverifiable()) {
            // @todo: Support more destination registers.
            // Grab the result from the instruction and write it to the
            // register.
            RegIndex idx = inst->destRegIdx(0);
            if (idx < TheISA::FP_Base_DepTag) {
                cpuXC->setIntReg(idx, inst->readIntResult());
            } else if (idx < TheISA::Fpcr_DepTag) {
                cpuXC->setFloatRegInt(idx, inst->readIntResult());
            } else {
                cpuXC->setMiscReg(idx, inst->readIntResult());
            }
        } else if (result.integer != inst->readIntResult()) {
            warn("Instruction results do not match! (May not be integer results) "
                 "Inst: %#x, checker: %#x",
                 inst->readIntResult(), result.integer);
            handleError();
        }
    }

    if (inst->readNextPC() != cpuXC->readNextPC()) {
        warn("Instruction next PCs do not match! Inst: %#x, checker: %#x",
             inst->readNextPC(), cpuXC->readNextPC());
        handleError();
    }

    // Checking side effect registers can be difficult if they are not
    // checked simultaneously with the execution of the instruction.
    // This is because other valid instructions may have modified
    // these registers in the meantime, and their values are not
    // stored within the DynInst.
    while (!miscRegIdxs.empty()) {
        int misc_reg_idx = miscRegIdxs.front();
        miscRegIdxs.pop();

        if (inst->xcBase()->readMiscReg(misc_reg_idx) !=
            cpuXC->readMiscReg(misc_reg_idx)) {
            warn("Misc reg idx %i (side effect) does not match! Inst: %#x, "
                 "checker: %#x",
                 misc_reg_idx, inst->xcBase()->readMiscReg(misc_reg_idx),
                 cpuXC->readMiscReg(misc_reg_idx));
            handleError();
        }
    }
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::validateState()
{
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::dumpInsts()
{
    int num = 0;

    InstListIt inst_list_it = --(instList.end());

    cprintf("Inst list size: %i\n", instList.size());

    while (inst_list_it != instList.end())
    {
        cprintf("Instruction:%i\n",
                num);

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Completed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isCompleted());

        cprintf("\n");

        inst_list_it--;
        ++num;
    }

}

template
class Checker<RefCountingPtr<OzoneDynInst<OzoneImpl> > >;

template
class Checker<RefCountingPtr<AlphaDynInst<AlphaSimpleImpl> > >;
