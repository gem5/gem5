/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#include <list>
#include <string>

#include "base/refcnt.hh"
#include "cpu/base.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "cpu/static_inst.hh"
#include "mem/packet_impl.hh"
#include "sim/byteswap.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

#if FULL_SYSTEM
#include "sim/system.hh"
#include "arch/vtophys.hh"
#include "kern/kernel_stats.hh"
#endif // FULL_SYSTEM

using namespace std;
//The CheckerCPU does alpha only
using namespace AlphaISA;

void
CheckerCPU::init()
{
}

CheckerCPU::CheckerCPU(Params *p)
    : BaseCPU(p), thread(NULL), tc(NULL)
{
    memReq = NULL;

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    youngestSN = 0;

    changedPC = willChangePC = changedNextPC = false;

    exitOnError = p->exitOnError;
    warnOnlyOnLoadError = p->warnOnlyOnLoadError;
#if FULL_SYSTEM
    itb = p->itb;
    dtb = p->dtb;
    systemPtr = NULL;
#else
    process = p->process;
#endif

    result.integer = 0;
}

CheckerCPU::~CheckerCPU()
{
}

void
CheckerCPU::setMemory(MemObject *mem)
{
#if !FULL_SYSTEM
    memPtr = mem;
    thread = new SimpleThread(this, /* thread_num */ 0, process,
                              /* asid */ 0, mem);

    thread->setStatus(ThreadContext::Suspended);
    tc = thread->getTC();
    threadContexts.push_back(tc);
#endif
}

void
CheckerCPU::setSystem(System *system)
{
#if FULL_SYSTEM
    systemPtr = system;

    thread = new SimpleThread(this, 0, systemPtr, itb, dtb, false);

    thread->setStatus(ThreadContext::Suspended);
    tc = thread->getTC();
    threadContexts.push_back(tc);
    delete thread->kernelStats;
    thread->kernelStats = NULL;
#endif
}

void
CheckerCPU::setIcachePort(Port *icache_port)
{
    icachePort = icache_port;
}

void
CheckerCPU::setDcachePort(Port *dcache_port)
{
    dcachePort = dcache_port;
}

void
CheckerCPU::serialize(ostream &os)
{
/*
    BaseCPU::serialize(os);
    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc", name()));
    thread->serialize(os);
    cacheCompletionEvent.serialize(os);
*/
}

void
CheckerCPU::unserialize(Checkpoint *cp, const string &section)
{
/*
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_SCALAR(inst);
    thread->unserialize(cp, csprintf("%s.xc", section));
*/
}

Fault
CheckerCPU::copySrcTranslate(Addr src)
{
    panic("Unimplemented!");
}

Fault
CheckerCPU::copy(Addr dest)
{
    panic("Unimplemented!");
}

template <class T>
Fault
CheckerCPU::read(Addr addr, T &data, unsigned flags)
{
    // need to fill in CPU & thread IDs here
    memReq = new Request();

    memReq->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    // translate to physical address
    translateDataReadReq(memReq);

    Packet *pkt = new Packet(memReq, Packet::ReadReq, Packet::Broadcast);

    pkt->dataStatic(&data);

    if (!(memReq->getFlags() & UNCACHEABLE)) {
        // Access memory to see if we have the same data
        dcachePort->sendFunctional(pkt);
    } else {
        // Assume the data is correct if it's an uncached access
        memcpy(&data, &unverifiedResult.integer, sizeof(T));
    }

    delete pkt;

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
    // need to fill in CPU & thread IDs here
    memReq = new Request();

    memReq->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    // translate to physical address
    thread->translateDataWriteReq(memReq);

    // Can compare the write data and result only if it's cacheable,
    // not a store conditional, or is a store conditional that
    // succeeded.
    // @todo: Verify that actual memory matches up with these values.
    // Right now it only verifies that the instruction data is the
    // same as what was in the request that got sent to memory; there
    // is no verification that it is the same as what is in memory.
    // This is because the LSQ would have to be snooped in the CPU to
    // verify this data.
    if (unverifiedReq &&
        !(unverifiedReq->getFlags() & UNCACHEABLE) &&
        (!(unverifiedReq->getFlags() & LOCKED) ||
         ((unverifiedReq->getFlags() & LOCKED) &&
          unverifiedReq->getScResult() == 1))) {
        T inst_data;
/*
        // This code would work if the LSQ allowed for snooping.
        Packet *pkt = new Packet(memReq, Packet::ReadReq, Packet::Broadcast);
        pkt.dataStatic(&inst_data);

        dcachePort->sendFunctional(pkt);

        delete pkt;
*/
        memcpy(&inst_data, unverifiedMemData, sizeof(T));

        if (data != inst_data) {
            warn("%lli: Store value does not match value in memory! "
                 "Instruction: %#x, memory: %#x",
                 curTick, inst_data, data);
            handleError();
        }
    }

    // Assume the result was the same as the one passed in.  This checker
    // doesn't check if the SC should succeed or fail, it just checks the
    // value.
    if (res && unverifiedReq->scResultValid())
        *res = unverifiedReq->getScResult();

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
    return vtophys(tc, addr);
}
#endif // FULL_SYSTEM

bool
CheckerCPU::translateInstReq(Request *req)
{
#if FULL_SYSTEM
    return (thread->translateInstReq(req) == NoFault);
#else
    thread->translateInstReq(req);
    return true;
#endif
}

void
CheckerCPU::translateDataReadReq(Request *req)
{
    thread->translateDataReadReq(req);

    if (req->getVaddr() != unverifiedReq->getVaddr()) {
        warn("%lli: Request virtual addresses do not match! Inst: %#x, "
             "checker: %#x",
             curTick, unverifiedReq->getVaddr(), req->getVaddr());
        handleError();
    }
    req->setPaddr(unverifiedReq->getPaddr());

    if (checkFlags(req)) {
        warn("%lli: Request flags do not match! Inst: %#x, checker: %#x",
             curTick, unverifiedReq->getFlags(), req->getFlags());
        handleError();
    }
}

void
CheckerCPU::translateDataWriteReq(Request *req)
{
    thread->translateDataWriteReq(req);

    if (req->getVaddr() != unverifiedReq->getVaddr()) {
        warn("%lli: Request virtual addresses do not match! Inst: %#x, "
             "checker: %#x",
             curTick, unverifiedReq->getVaddr(), req->getVaddr());
        handleError();
    }
    req->setPaddr(unverifiedReq->getPaddr());

    if (checkFlags(req)) {
        warn("%lli: Request flags do not match! Inst: %#x, checker: %#x",
             curTick, unverifiedReq->getFlags(), req->getFlags());
        handleError();
    }
}

bool
CheckerCPU::checkFlags(Request *req)
{
    // Remove any dynamic flags that don't have to do with the request itself.
    unsigned flags = unverifiedReq->getFlags();
    unsigned mask = LOCKED | PHYSICAL | VPTE | ALTMODE | UNCACHEABLE | NO_FAULT;
    flags = flags & (mask);
    if (flags == req->getFlags()) {
        return false;
    } else {
        return true;
    }
}

void
CheckerCPU::dumpAndExit()
{
    warn("%lli: Checker PC:%#x, next PC:%#x",
         curTick, thread->readPC(), thread->readNextPC());
    panic("Checker found an error!");
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::verify(DynInstPtr &completed_inst)
{
    DynInstPtr inst;

    // Either check this instruction, or add it to a list of
    // instructions waiting to be checked.  Instructions must be
    // checked in program order, so if a store has committed yet not
    // completed, there may be some instructions that are waiting
    // behind it that have completed and must be checked.
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
                return;
            }
        }
    }

    // Try to check all instructions that are completed, ending if we
    // run out of instructions to check or if an instruction is not
    // yet completed.
    while (1) {
        DPRINTF(Checker, "Processing instruction [sn:%lli] PC:%#x.\n",
                inst->seqNum, inst->readPC());
        unverifiedResult.integer = inst->readIntResult();
        unverifiedReq = inst->req;
        unverifiedMemData = inst->memData;
        numCycles++;

        Fault fault = NoFault;

        // maintain $r0 semantics
        thread->setIntReg(ZeroReg, 0);
#ifdef TARGET_ALPHA
        thread->setFloatRegDouble(ZeroReg, 0.0);
#endif // TARGET_ALPHA

        // Check if any recent PC changes match up with anything we
        // expect to happen.  This is mostly to check if traps or
        // PC-based events have occurred in both the checker and CPU.
        if (changedPC) {
            DPRINTF(Checker, "Changed PC recently to %#x\n",
                    thread->readPC());
            if (willChangePC) {
                if (newPC == thread->readPC()) {
                    DPRINTF(Checker, "Changed PC matches expected PC\n");
                } else {
                    warn("%lli: Changed PC does not match expected PC, "
                         "changed: %#x, expected: %#x",
                         curTick, thread->readPC(), newPC);
                    CheckerCPU::handleError();
                }
                willChangePC = false;
            }
            changedPC = false;
        }
        if (changedNextPC) {
            DPRINTF(Checker, "Changed NextPC recently to %#x\n",
                    thread->readNextPC());
            changedNextPC = false;
        }

        // Try to fetch the instruction

#if FULL_SYSTEM
#define IFETCH_FLAGS(pc)	((pc) & 1) ? PHYSICAL : 0
#else
#define IFETCH_FLAGS(pc)	0
#endif

        uint64_t fetch_PC = thread->readPC() & ~3;

        // set up memory request for instruction fetch
        memReq = new Request(inst->threadNumber, fetch_PC,
                             sizeof(uint32_t),
                             IFETCH_FLAGS(thread->readPC()),
                             fetch_PC, thread->readCpuId(), inst->threadNumber);

        bool succeeded = translateInstReq(memReq);

        if (!succeeded) {
            if (inst->getFault() == NoFault) {
                // In this case the instruction was not a dummy
                // instruction carrying an ITB fault.  In the single
                // threaded case the ITB should still be able to
                // translate this instruction; in the SMT case it's
                // possible that its ITB entry was kicked out.
                warn("%lli: Instruction PC %#x was not found in the ITB!",
                     curTick, thread->readPC());
                handleError(inst);

                // go to the next instruction
                thread->setPC(thread->readNextPC());
                thread->setNextPC(thread->readNextPC() + sizeof(MachInst));

                return;
            } else {
                // The instruction is carrying an ITB fault.  Handle
                // the fault and see if our results match the CPU on
                // the next tick().
                fault = inst->getFault();
            }
        }

        if (fault == NoFault) {
            Packet *pkt = new Packet(memReq, Packet::ReadReq,
                                     Packet::Broadcast);

            pkt->dataStatic(&machInst);

            icachePort->sendFunctional(pkt);

            delete pkt;

            // keep an instruction count
            numInst++;

            // decode the instruction
            machInst = gtoh(machInst);
            // Checks that the instruction matches what we expected it to be.
            // Checks both the machine instruction and the PC.
            validateInst(inst);

            curStaticInst = StaticInst::decode(makeExtMI(machInst,
                                                         thread->readPC()));

#if FULL_SYSTEM
            thread->setInst(machInst);
#endif // FULL_SYSTEM

            fault = inst->getFault();
        }

        // Discard fetch's memReq.
        delete memReq;
        memReq = NULL;

        // Either the instruction was a fault and we should process the fault,
        // or we should just go ahead execute the instruction.  This assumes
        // that the instruction is properly marked as a fault.
        if (fault == NoFault) {

            thread->funcExeInst++;

            fault = curStaticInst->execute(this, NULL);

            // Checks to make sure instrution results are correct.
            validateExecution(inst);

            if (curStaticInst->isLoad()) {
                ++numLoad;
            }
        }

        if (fault != NoFault) {
#if FULL_SYSTEM
            fault->invoke(tc);
            willChangePC = true;
            newPC = thread->readPC();
            DPRINTF(Checker, "Fault, PC is now %#x\n", newPC);
#else // !FULL_SYSTEM
            fatal("fault (%d) detected @ PC 0x%08p", fault, thread->readPC());
#endif // FULL_SYSTEM
        } else {
#if THE_ISA != MIPS_ISA
            // go to the next instruction
            thread->setPC(thread->readNextPC());
            thread->setNextPC(thread->readNextPC() + sizeof(MachInst));
#else
            // go to the next instruction
            thread->setPC(thread->readNextPC());
            thread->setNextPC(thread->readNextNPC());
            thread->setNextNPC(thread->readNextNPC() + sizeof(MachInst));
#endif

        }

#if FULL_SYSTEM
        // @todo: Determine if these should happen only if the
        // instruction hasn't faulted.  In the SimpleCPU case this may
        // not be true, but in the O3 or Ozone case this may be true.
        Addr oldpc;
        int count = 0;
        do {
            oldpc = thread->readPC();
            system->pcEventQueue.service(tc);
            count++;
        } while (oldpc != thread->readPC());
        if (count > 1) {
            willChangePC = true;
            newPC = thread->readPC();
            DPRINTF(Checker, "PC Event, PC is now %#x\n", newPC);
        }
#endif

        // @todo:  Optionally can check all registers. (Or just those
        // that have been modified).
        validateState();

        if (memReq) {
            delete memReq;
            memReq = NULL;
        }

        // Continue verifying instructions if there's another completed
        // instruction waiting to be verified.
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
    instList.clear();
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::takeOverFrom(BaseCPU *oldCPU)
{
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::validateInst(DynInstPtr &inst)
{
    if (inst->readPC() != thread->readPC()) {
        warn("%lli: PCs do not match! Inst: %#x, checker: %#x",
             curTick, inst->readPC(), thread->readPC());
        if (changedPC) {
            warn("%lli: Changed PCs recently, may not be an error",
                 curTick);
        } else {
            handleError(inst);
        }
    }

    MachInst mi = static_cast<MachInst>(inst->staticInst->machInst);

    if (mi != machInst) {
        warn("%lli: Binary instructions do not match! Inst: %#x, "
             "checker: %#x",
             curTick, mi, machInst);
        handleError(inst);
    }
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::validateExecution(DynInstPtr &inst)
{
    bool result_mismatch = false;
    if (inst->numDestRegs()) {
        // @todo: Support more destination registers.
        if (inst->isUnverifiable()) {
            // Unverifiable instructions assume they were executed
            // properly by the CPU. Grab the result from the
            // instruction and write it to the register.
            copyResult(inst);
        } else if (result.integer != inst->readIntResult()) {
            result_mismatch = true;
        }
    }

    if (result_mismatch) {
        warn("%lli: Instruction results do not match! (Values may not "
             "actually be integers) Inst: %#x, checker: %#x",
             curTick, inst->readIntResult(), result.integer);

        // It's useful to verify load values from memory, but in MP
        // systems the value obtained at execute may be different than
        // the value obtained at completion.  Similarly DMA can
        // present the same problem on even UP systems.  Thus there is
        // the option to only warn on loads having a result error.
        if (inst->isLoad() && warnOnlyOnLoadError) {
            copyResult(inst);
        } else {
            handleError(inst);
        }
    }

    if (inst->readNextPC() != thread->readNextPC()) {
        warn("%lli: Instruction next PCs do not match! Inst: %#x, "
             "checker: %#x",
             curTick, inst->readNextPC(), thread->readNextPC());
        handleError(inst);
    }

    // Checking side effect registers can be difficult if they are not
    // checked simultaneously with the execution of the instruction.
    // This is because other valid instructions may have modified
    // these registers in the meantime, and their values are not
    // stored within the DynInst.
    while (!miscRegIdxs.empty()) {
        int misc_reg_idx = miscRegIdxs.front();
        miscRegIdxs.pop();

        if (inst->tcBase()->readMiscReg(misc_reg_idx) !=
            thread->readMiscReg(misc_reg_idx)) {
            warn("%lli: Misc reg idx %i (side effect) does not match! "
                 "Inst: %#x, checker: %#x",
                 curTick, misc_reg_idx,
                 inst->tcBase()->readMiscReg(misc_reg_idx),
                 thread->readMiscReg(misc_reg_idx));
            handleError(inst);
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
Checker<DynInstPtr>::copyResult(DynInstPtr &inst)
{
    RegIndex idx = inst->destRegIdx(0);
    if (idx < TheISA::FP_Base_DepTag) {
        thread->setIntReg(idx, inst->readIntResult());
    } else if (idx < TheISA::Fpcr_DepTag) {
        thread->setFloatRegBits(idx, inst->readIntResult());
    } else {
        thread->setMiscReg(idx, inst->readIntResult());
    }
}

template <class DynInstPtr>
void
Checker<DynInstPtr>::dumpAndExit(DynInstPtr &inst)
{
    cprintf("Error detected, instruction information:\n");
    cprintf("PC:%#x, nextPC:%#x\n[sn:%lli]\n[tid:%i]\n"
            "Completed:%i\n",
            inst->readPC(),
            inst->readNextPC(),
            inst->seqNum,
            inst->threadNumber,
            inst->isCompleted());
    inst->dump();
    CheckerCPU::dumpAndExit();
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
