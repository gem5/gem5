/*
 * Copyright (c) 2011 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Geoffrey Blake
 */

#ifndef __CPU_CHECKER_CPU_IMPL_HH__
#define __CPU_CHECKER_CPU_IMPL_HH__

#include <list>
#include <string>

#include "arch/isa_traits.hh"
#include "arch/vtophys.hh"
#include "base/refcnt.hh"
#include "config/the_isa.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/exetrace.hh"
#include "cpu/reg_class.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "cpu/checker/cpu.hh"
#include "debug/Checker.hh"
#include "sim/full_system.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

using namespace std;
using namespace TheISA;

template <class Impl>
void
Checker<Impl>::advancePC(const Fault &fault)
{
    if (fault != NoFault) {
        curMacroStaticInst = StaticInst::nullStaticInstPtr;
        fault->invoke(tc, curStaticInst);
        thread->decoder.reset();
    } else {
        if (curStaticInst) {
            if (curStaticInst->isLastMicroop())
                curMacroStaticInst = StaticInst::nullStaticInstPtr;
            TheISA::PCState pcState = thread->pcState();
            TheISA::advancePC(pcState, curStaticInst);
            thread->pcState(pcState);
            DPRINTF(Checker, "Advancing PC to %s.\n", thread->pcState());
        }
    }
}
//////////////////////////////////////////////////

template <class Impl>
void
Checker<Impl>::handlePendingInt()
{
    DPRINTF(Checker, "IRQ detected at PC: %s with %d insts in buffer\n",
                     thread->pcState(), instList.size());
    DynInstPtr boundaryInst = NULL;
    if (!instList.empty()) {
        // Set the instructions as completed and verify as much as possible.
        DynInstPtr inst;
        typename std::list<DynInstPtr>::iterator itr;

        for (itr = instList.begin(); itr != instList.end(); itr++) {
            (*itr)->setCompleted();
        }

        inst = instList.front();
        boundaryInst = instList.back();
        verify(inst); // verify the instructions
        inst = NULL;
    }
    if ((!boundaryInst && curMacroStaticInst &&
          curStaticInst->isDelayedCommit() &&
          !curStaticInst->isLastMicroop()) ||
        (boundaryInst && boundaryInst->isDelayedCommit() &&
         !boundaryInst->isLastMicroop())) {
        panic("%lli: Trying to take an interrupt in middle of "
              "a non-interuptable instruction!", curTick());
    }
    boundaryInst = NULL;
    thread->decoder.reset();
    curMacroStaticInst = StaticInst::nullStaticInstPtr;
}

template <class Impl>
void
Checker<Impl>::verify(DynInstPtr &completed_inst)
{
    DynInstPtr inst;

    // Make sure serializing instructions are actually
    // seen as serializing to commit. instList should be
    // empty in these cases.
    if ((completed_inst->isSerializing() ||
        completed_inst->isSerializeBefore()) &&
        (!instList.empty() ?
         (instList.front()->seqNum != completed_inst->seqNum) : 0)) {
        panic("%lli: Instruction sn:%lli at PC %s is serializing before but is"
              " entering instList with other instructions\n", curTick(),
              completed_inst->seqNum, completed_inst->pcState());
    }

    // Either check this instruction, or add it to a list of
    // instructions waiting to be checked.  Instructions must be
    // checked in program order, so if a store has committed yet not
    // completed, there may be some instructions that are waiting
    // behind it that have completed and must be checked.
    if (!instList.empty()) {
        if (youngestSN < completed_inst->seqNum) {
            DPRINTF(Checker, "Adding instruction [sn:%lli] PC:%s to list\n",
                    completed_inst->seqNum, completed_inst->pcState());
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
                DPRINTF(Checker, "Adding instruction [sn:%lli] PC:%s to list\n",
                        completed_inst->seqNum, completed_inst->pcState());
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

    // Make sure a serializing instruction is actually seen as
    // serializing. instList should be empty here
    if (inst->isSerializeAfter() && !instList.empty()) {
        panic("%lli: Instruction sn:%lli at PC %s is serializing after but is"
             " exiting instList with other instructions\n", curTick(),
             completed_inst->seqNum, completed_inst->pcState());
    }
    unverifiedInst = inst;
    inst = NULL;

    // Try to check all instructions that are completed, ending if we
    // run out of instructions to check or if an instruction is not
    // yet completed.
    while (1) {
        DPRINTF(Checker, "Processing instruction [sn:%lli] PC:%s.\n",
                unverifiedInst->seqNum, unverifiedInst->pcState());
        unverifiedReq = NULL;
        unverifiedReq = unverifiedInst->reqToVerify;
        unverifiedMemData = unverifiedInst->memData;
        // Make sure results queue is empty
        while (!result.empty()) {
            result.pop();
        }
        numCycles++;

        Fault fault = NoFault;

        // maintain $r0 semantics
        thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
        thread->setFloatReg(ZeroReg, 0.0);
#endif

        // Check if any recent PC changes match up with anything we
        // expect to happen.  This is mostly to check if traps or
        // PC-based events have occurred in both the checker and CPU.
        if (changedPC) {
            DPRINTF(Checker, "Changed PC recently to %s\n",
                    thread->pcState());
            if (willChangePC) {
                if (newPCState == thread->pcState()) {
                    DPRINTF(Checker, "Changed PC matches expected PC\n");
                } else {
                    warn("%lli: Changed PC does not match expected PC, "
                         "changed: %s, expected: %s",
                         curTick(), thread->pcState(), newPCState);
                    CheckerCPU::handleError();
                }
                willChangePC = false;
            }
            changedPC = false;
        }

        // Try to fetch the instruction
        uint64_t fetchOffset = 0;
        bool fetchDone = false;

        while (!fetchDone) {
            Addr fetch_PC = thread->instAddr();
            fetch_PC = (fetch_PC & PCMask) + fetchOffset;

            MachInst machInst;

            // If not in the middle of a macro instruction
            if (!curMacroStaticInst) {
                // set up memory request for instruction fetch
                memReq = new Request(unverifiedInst->threadNumber, fetch_PC,
                                     sizeof(MachInst),
                                     0,
                                     masterId,
                                     fetch_PC, thread->contextId());
                memReq->setVirt(0, fetch_PC, sizeof(MachInst),
                                Request::INST_FETCH, masterId, thread->instAddr());


                fault = itb->translateFunctional(memReq, tc, BaseTLB::Execute);

                if (fault != NoFault) {
                    if (unverifiedInst->getFault() == NoFault) {
                        // In this case the instruction was not a dummy
                        // instruction carrying an ITB fault.  In the single
                        // threaded case the ITB should still be able to
                        // translate this instruction; in the SMT case it's
                        // possible that its ITB entry was kicked out.
                        warn("%lli: Instruction PC %s was not found in the "
                             "ITB!", curTick(), thread->pcState());
                        handleError(unverifiedInst);

                        // go to the next instruction
                        advancePC(NoFault);

                        // Give up on an ITB fault..
                        delete memReq;
                        unverifiedInst = NULL;
                        return;
                    } else {
                        // The instruction is carrying an ITB fault.  Handle
                        // the fault and see if our results match the CPU on
                        // the next tick().
                        fault = unverifiedInst->getFault();
                        delete memReq;
                        break;
                    }
                } else {
                    PacketPtr pkt = new Packet(memReq, MemCmd::ReadReq);

                    pkt->dataStatic(&machInst);
                    icachePort->sendFunctional(pkt);
                    machInst = gtoh(machInst);

                    delete memReq;
                    delete pkt;
                }
            }

            if (fault == NoFault) {
                TheISA::PCState pcState = thread->pcState();

                if (isRomMicroPC(pcState.microPC())) {
                    fetchDone = true;
                    curStaticInst =
                        microcodeRom.fetchMicroop(pcState.microPC(), NULL);
                } else if (!curMacroStaticInst) {
                    //We're not in the middle of a macro instruction
                    StaticInstPtr instPtr = nullptr;

                    //Predecode, ie bundle up an ExtMachInst
                    //If more fetch data is needed, pass it in.
                    Addr fetchPC = (pcState.instAddr() & PCMask) + fetchOffset;
                    thread->decoder.moreBytes(pcState, fetchPC, machInst);

                    //If an instruction is ready, decode it.
                    //Otherwise, we'll have to fetch beyond the
                    //MachInst at the current pc.
                    if (thread->decoder.instReady()) {
                        fetchDone = true;
                        instPtr = thread->decoder.decode(pcState);
                        thread->pcState(pcState);
                    } else {
                        fetchDone = false;
                        fetchOffset += sizeof(TheISA::MachInst);
                    }

                    //If we decoded an instruction and it's microcoded,
                    //start pulling out micro ops
                    if (instPtr && instPtr->isMacroop()) {
                        curMacroStaticInst = instPtr;
                        curStaticInst =
                            instPtr->fetchMicroop(pcState.microPC());
                    } else {
                        curStaticInst = instPtr;
                    }
                } else {
                    // Read the next micro op from the macro-op
                    curStaticInst =
                        curMacroStaticInst->fetchMicroop(pcState.microPC());
                    fetchDone = true;
                }
            }
        }
        // reset decoder on Checker
        thread->decoder.reset();

        // Check Checker and CPU get same instruction, and record
        // any faults the CPU may have had.
        Fault unverifiedFault;
        if (fault == NoFault) {
            unverifiedFault = unverifiedInst->getFault();

            // Checks that the instruction matches what we expected it to be.
            // Checks both the machine instruction and the PC.
            validateInst(unverifiedInst);
        }

        // keep an instruction count
        numInst++;


        // Either the instruction was a fault and we should process the fault,
        // or we should just go ahead execute the instruction.  This assumes
        // that the instruction is properly marked as a fault.
        if (fault == NoFault) {
            // Execute Checker instruction and trace
            if (!unverifiedInst->isUnverifiable()) {
                Trace::InstRecord *traceData = tracer->getInstRecord(curTick(),
                                                           tc,
                                                           curStaticInst,
                                                           pcState(),
                                                           curMacroStaticInst);
                fault = curStaticInst->execute(this, traceData);
                if (traceData) {
                    traceData->dump();
                    delete traceData;
                }
            }

            if (fault == NoFault && unverifiedFault == NoFault) {
                thread->funcExeInst++;
                // Checks to make sure instrution results are correct.
                validateExecution(unverifiedInst);

                if (curStaticInst->isLoad()) {
                    ++numLoad;
                }
            } else if (fault != NoFault && unverifiedFault == NoFault) {
                panic("%lli: sn: %lli at PC: %s took a fault in checker "
                      "but not in driver CPU\n", curTick(),
                      unverifiedInst->seqNum, unverifiedInst->pcState());
            } else if (fault == NoFault && unverifiedFault != NoFault) {
                panic("%lli: sn: %lli at PC: %s took a fault in driver "
                      "CPU but not in checker\n", curTick(),
                      unverifiedInst->seqNum, unverifiedInst->pcState());
            }
        }

        // Take any faults here
        if (fault != NoFault) {
            if (FullSystem) {
                fault->invoke(tc, curStaticInst);
                willChangePC = true;
                newPCState = thread->pcState();
                DPRINTF(Checker, "Fault, PC is now %s\n", newPCState);
                curMacroStaticInst = StaticInst::nullStaticInstPtr;
            }
        } else {
           advancePC(fault);
        }

        if (FullSystem) {
            // @todo: Determine if these should happen only if the
            // instruction hasn't faulted.  In the SimpleCPU case this may
            // not be true, but in the O3 case this may be true.
            Addr oldpc;
            int count = 0;
            do {
                oldpc = thread->instAddr();
                system->pcEventQueue.service(tc);
                count++;
            } while (oldpc != thread->instAddr());
            if (count > 1) {
                willChangePC = true;
                newPCState = thread->pcState();
                DPRINTF(Checker, "PC Event, PC is now %s\n", newPCState);
            }
        }

        // @todo:  Optionally can check all registers. (Or just those
        // that have been modified).
        validateState();

        // Continue verifying instructions if there's another completed
        // instruction waiting to be verified.
        if (instList.empty()) {
            break;
        } else if (instList.front()->isCompleted()) {
            unverifiedInst = NULL;
            unverifiedInst = instList.front();
            instList.pop_front();
        } else {
            break;
        }
    }
    unverifiedInst = NULL;
}

template <class Impl>
void
Checker<Impl>::switchOut()
{
    instList.clear();
}

template <class Impl>
void
Checker<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
}

template <class Impl>
void
Checker<Impl>::validateInst(DynInstPtr &inst)
{
    if (inst->instAddr() != thread->instAddr()) {
        warn("%lli: PCs do not match! Inst: %s, checker: %s",
             curTick(), inst->pcState(), thread->pcState());
        if (changedPC) {
            warn("%lli: Changed PCs recently, may not be an error",
                 curTick());
        } else {
            handleError(inst);
        }
    }

    if (curStaticInst != inst->staticInst) {
        warn("%lli: StaticInstPtrs don't match. (%s, %s).\n", curTick(),
                curStaticInst->getName(), inst->staticInst->getName());
    }
}

template <class Impl>
void
Checker<Impl>::validateExecution(DynInstPtr &inst)
{
    uint64_t checker_val;
    uint64_t inst_val;
    int idx = -1;
    bool result_mismatch = false;

    if (inst->isUnverifiable()) {
        // Unverifiable instructions assume they were executed
        // properly by the CPU. Grab the result from the
        // instruction and write it to the register.
        copyResult(inst, 0, idx);
    } else if (inst->numDestRegs() > 0 && !result.empty()) {
        DPRINTF(Checker, "Dest regs %d, number of checker dest regs %d\n",
                         inst->numDestRegs(), result.size());
        for (int i = 0; i < inst->numDestRegs() && !result.empty(); i++) {
            result.front().get(checker_val);
            result.pop();
            inst_val = 0;
            inst->template popResult<uint64_t>(inst_val);
            if (checker_val != inst_val) {
                result_mismatch = true;
                idx = i;
                break;
            }
        }
    } // Checker CPU checks all the saved results in the dyninst passed by
      // the cpu model being checked against the saved results present in
      // the static inst executed in the Checker.  Sometimes the number
      // of saved results differs between the dyninst and static inst, but
      // this is ok and not a bug.  May be worthwhile to try and correct this.

    if (result_mismatch) {
        warn("%lli: Instruction results do not match! (Values may not "
             "actually be integers) Inst: %#x, checker: %#x",
             curTick(), inst_val, checker_val);

        // It's useful to verify load values from memory, but in MP
        // systems the value obtained at execute may be different than
        // the value obtained at completion.  Similarly DMA can
        // present the same problem on even UP systems.  Thus there is
        // the option to only warn on loads having a result error.
        // The load/store queue in Detailed CPU can also cause problems
        // if load/store forwarding is allowed.
        if (inst->isLoad() && warnOnlyOnLoadError) {
            copyResult(inst, inst_val, idx);
        } else {
            handleError(inst);
        }
    }

    if (inst->nextInstAddr() != thread->nextInstAddr()) {
        warn("%lli: Instruction next PCs do not match! Inst: %#x, "
             "checker: %#x",
             curTick(), inst->nextInstAddr(), thread->nextInstAddr());
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

        if (inst->tcBase()->readMiscRegNoEffect(misc_reg_idx) !=
            thread->readMiscRegNoEffect(misc_reg_idx)) {
            warn("%lli: Misc reg idx %i (side effect) does not match! "
                 "Inst: %#x, checker: %#x",
                 curTick(), misc_reg_idx,
                 inst->tcBase()->readMiscRegNoEffect(misc_reg_idx),
                 thread->readMiscRegNoEffect(misc_reg_idx));
            handleError(inst);
        }
    }
}


// This function is weird, if it is called it means the Checker and
// O3 have diverged, so panic is called for now.  It may be useful
// to resynch states and continue if the divergence is a false positive
template <class Impl>
void
Checker<Impl>::validateState()
{
    if (updateThisCycle) {
        // Change this back to warn if divergences end up being false positives
        panic("%lli: Instruction PC %#x results didn't match up, copying all "
             "registers from main CPU", curTick(), unverifiedInst->instAddr());

        // Terribly convoluted way to make sure O3 model does not implode
        bool no_squash_from_TC = unverifiedInst->thread->noSquashFromTC;
        unverifiedInst->thread->noSquashFromTC = true;

        // Heavy-weight copying of all registers
        thread->copyArchRegs(unverifiedInst->tcBase());
        unverifiedInst->thread->noSquashFromTC = no_squash_from_TC;

        // Set curStaticInst to unverifiedInst->staticInst
        curStaticInst = unverifiedInst->staticInst;
        // Also advance the PC.  Hopefully no PC-based events happened.
        advancePC(NoFault);
        updateThisCycle = false;
    }
}

template <class Impl>
void
Checker<Impl>::copyResult(DynInstPtr &inst, uint64_t mismatch_val,
                          int start_idx)
{
    // We've already popped one dest off the queue,
    // so do the fix-up then start with the next dest reg;
    if (start_idx >= 0) {
        RegIndex idx = inst->destRegIdx(start_idx);
        switch (regIdxToClass(idx)) {
          case IntRegClass:
            thread->setIntReg(idx, mismatch_val);
            break;
          case FloatRegClass:
            thread->setFloatRegBits(idx - TheISA::FP_Reg_Base, mismatch_val);
            break;
          case CCRegClass:
            thread->setCCReg(idx - TheISA::CC_Reg_Base, mismatch_val);
            break;
          case MiscRegClass:
            thread->setMiscReg(idx - TheISA::Misc_Reg_Base,
                               mismatch_val);
            break;
        }
    }
    start_idx++;
    uint64_t res = 0;
    for (int i = start_idx; i < inst->numDestRegs(); i++) {
        RegIndex idx = inst->destRegIdx(i);
        inst->template popResult<uint64_t>(res);
        switch (regIdxToClass(idx)) {
          case IntRegClass:
            thread->setIntReg(idx, res);
            break;
          case FloatRegClass:
            thread->setFloatRegBits(idx - TheISA::FP_Reg_Base, res);
            break;
          case CCRegClass:
            thread->setCCReg(idx - TheISA::CC_Reg_Base, res);
            break;
          case MiscRegClass:
            // Try to get the proper misc register index for ARM here...
            thread->setMiscReg(idx - TheISA::Misc_Reg_Base, res);
            break;
            // else Register is out of range...
        }
    }
}

template <class Impl>
void
Checker<Impl>::dumpAndExit(DynInstPtr &inst)
{
    cprintf("Error detected, instruction information:\n");
    cprintf("PC:%s, nextPC:%#x\n[sn:%lli]\n[tid:%i]\n"
            "Completed:%i\n",
            inst->pcState(),
            inst->nextInstAddr(),
            inst->seqNum,
            inst->threadNumber,
            inst->isCompleted());
    inst->dump();
    CheckerCPU::dumpAndExit();
}

template <class Impl>
void
Checker<Impl>::dumpInsts()
{
    int num = 0;

    InstListIt inst_list_it = --(instList.end());

    cprintf("Inst list size: %i\n", instList.size());

    while (inst_list_it != instList.end())
    {
        cprintf("Instruction:%i\n",
                num);

        cprintf("PC:%s\n[sn:%lli]\n[tid:%i]\n"
                "Completed:%i\n",
                (*inst_list_it)->pcState(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isCompleted());

        cprintf("\n");

        inst_list_it--;
        ++num;
    }

}

#endif//__CPU_CHECKER_CPU_IMPL_HH__
