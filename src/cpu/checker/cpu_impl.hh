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
#include "arch/vtophys.hh"
#endif // FULL_SYSTEM

using namespace std;
//The CheckerCPU does alpha only
using namespace AlphaISA;

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
                                                         thread->getTC()));

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
#endif
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
Checker<DynInstPtr>::switchOut()
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
