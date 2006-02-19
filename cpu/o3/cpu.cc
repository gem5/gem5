/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "config/full_system.hh"

#if FULL_SYSTEM
#include "sim/system.hh"
#else
#include "sim/process.hh"
#endif
#include "sim/root.hh"

#include "cpu/o3/alpha_dyn_inst.hh"
#include "cpu/o3/alpha_impl.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/exec_context.hh"

using namespace std;

BaseFullCPU::BaseFullCPU(Params &params)
    : BaseCPU(&params), cpu_id(0)
{
}

template <class Impl>
FullO3CPU<Impl>::TickEvent::TickEvent(FullO3CPU<Impl> *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}

template <class Impl>
void
FullO3CPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
FullO3CPU<Impl>::TickEvent::description()
{
    return "FullO3CPU tick event";
}

//Call constructor to all the pipeline stages here
template <class Impl>
FullO3CPU<Impl>::FullO3CPU(Params &params)
#if FULL_SYSTEM
    : BaseFullCPU(params),
#else
    : BaseFullCPU(params),
#endif // FULL_SYSTEM
      tickEvent(this),
      fetch(params),
      decode(params),
      rename(params),
      iew(params),
      commit(params),

      regFile(params.numPhysIntRegs, params.numPhysFloatRegs),

      freeList(TheISA::NumIntRegs, params.numPhysIntRegs,
               TheISA::NumFloatRegs, params.numPhysFloatRegs),

      renameMap(TheISA::NumIntRegs, params.numPhysIntRegs,
                TheISA::NumFloatRegs, params.numPhysFloatRegs,
                TheISA::NumMiscRegs,
                TheISA::ZeroReg,
                TheISA::ZeroReg + TheISA::NumIntRegs),

      rob(params.numROBEntries, params.squashWidth),

      // What to pass to these time buffers?
      // For now just have these time buffers be pretty big.
      timeBuffer(5, 5),
      fetchQueue(5, 5),
      decodeQueue(5, 5),
      renameQueue(5, 5),
      iewQueue(5, 5),

      xc(NULL),

      globalSeqNum(1),

#if FULL_SYSTEM
      system(params.system),
      memCtrl(system->memctrl),
      physmem(system->physmem),
      itb(params.itb),
      dtb(params.dtb),
      mem(params.mem),
#else
      // Hardcoded for a single thread!!
      mem(params.workload[0]->getMemory()),
#endif // FULL_SYSTEM

      icacheInterface(params.icacheInterface),
      dcacheInterface(params.dcacheInterface),
      deferRegistration(params.defReg),
      numInsts(0),
      funcExeInst(0)
{
    _status = Idle;

#if !FULL_SYSTEM
    thread.resize(this->number_of_threads);
#endif

    for (int i = 0; i < this->number_of_threads; ++i) {
#if FULL_SYSTEM
        assert(i == 0);
        system->execContexts[i] =
            new ExecContext(this, i, system, itb, dtb, mem);

        // initialize CPU, including PC
        TheISA::initCPU(&system->execContexts[i]->regs);
        execContexts.push_back(system->execContexts[i]);
#else
        if (i < params.workload.size()) {
            DPRINTF(FullCPU, "FullCPU: Workload[%i]'s starting PC is %#x, "
                    "process is %#x",
                    i, params.workload[i]->prog_entry, thread[i]);
            thread[i] = new ExecContext(this, i, params.workload[i], i);
        }
        assert(params.workload[i]->getMemory() != NULL);
        assert(mem != NULL);
        execContexts.push_back(thread[i]);
#endif // !FULL_SYSTEM
    }

    // Note that this is a hack so that my code which still uses xc-> will
    // still work.  I should remove this eventually
#if FULL_SYSTEM
    xc = system->execContexts[0];
#else
    xc = thread[0];
#endif

    // The stages also need their CPU pointer setup.  However this must be
    // done at the upper level CPU because they have pointers to the upper
    // level CPU, and not this FullO3CPU.

    // Give each of the stages the time buffer they will use.
    fetch.setTimeBuffer(&timeBuffer);
    decode.setTimeBuffer(&timeBuffer);
    rename.setTimeBuffer(&timeBuffer);
    iew.setTimeBuffer(&timeBuffer);
    commit.setTimeBuffer(&timeBuffer);

    // Also setup each of the stages' queues.
    fetch.setFetchQueue(&fetchQueue);
    decode.setFetchQueue(&fetchQueue);
    decode.setDecodeQueue(&decodeQueue);
    rename.setDecodeQueue(&decodeQueue);
    rename.setRenameQueue(&renameQueue);
    iew.setRenameQueue(&renameQueue);
    iew.setIEWQueue(&iewQueue);
    commit.setIEWQueue(&iewQueue);
    commit.setRenameQueue(&renameQueue);

    // Setup the rename map for whichever stages need it.
    rename.setRenameMap(&renameMap);
    iew.setRenameMap(&renameMap);

    // Setup the free list for whichever stages need it.
    rename.setFreeList(&freeList);
    renameMap.setFreeList(&freeList);

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);
}

template <class Impl>
FullO3CPU<Impl>::~FullO3CPU()
{
}

template <class Impl>
void
FullO3CPU<Impl>::fullCPURegStats()
{
    // Register any of the FullCPU's stats here.
}

template <class Impl>
void
FullO3CPU<Impl>::tick()
{
    DPRINTF(FullCPU, "\n\nFullCPU: Ticking main, FullO3CPU.\n");

    //Tick each of the stages if they're actually running.
    //Will want to figure out a way to unschedule itself if they're all
    //going to be idle for a long time.
    fetch.tick();

    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();

    // Now advance the time buffers, unless the stage is stalled.
    timeBuffer.advance();

    fetchQueue.advance();
    decodeQueue.advance();
    renameQueue.advance();
    iewQueue.advance();

    if (_status == Running && !tickEvent.scheduled())
        tickEvent.schedule(curTick + 1);
}

template <class Impl>
void
FullO3CPU<Impl>::init()
{
    if(!deferRegistration)
    {
        this->registerExecContexts();

        // Need to do a copy of the xc->regs into the CPU's regfile so
        // that it can start properly.
#if FULL_SYSTEM
        ExecContext *src_xc = system->execContexts[0];
#else
        ExecContext *src_xc = thread[0];
#endif
        // First loop through the integer registers.
        for (int i = 0; i < TheISA::NumIntRegs; ++i)
        {
            regFile.intRegFile[i] = src_xc->regs.intRegFile[i];
        }

        // Then loop through the floating point registers.
        for (int i = 0; i < TheISA::NumFloatRegs; ++i)
        {
            regFile.floatRegFile[i].d = src_xc->regs.floatRegFile.d[i];
            regFile.floatRegFile[i].q = src_xc->regs.floatRegFile.q[i];
        }

        // Then loop through the misc registers.
        regFile.miscRegs.fpcr = src_xc->regs.miscRegs.fpcr;
        regFile.miscRegs.uniq = src_xc->regs.miscRegs.uniq;
        regFile.miscRegs.lock_flag = src_xc->regs.miscRegs.lock_flag;
        regFile.miscRegs.lock_addr = src_xc->regs.miscRegs.lock_addr;

        // Then finally set the PC and the next PC.
        regFile.pc = src_xc->regs.pc;
        regFile.npc = src_xc->regs.npc;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::activateContext(int thread_num, int delay)
{
    // Needs to set each stage to running as well.

    scheduleTickEvent(delay);

    _status = Running;
}

template <class Impl>
void
FullO3CPU<Impl>::suspendContext(int thread_num)
{
    panic("suspendContext unimplemented!");
}

template <class Impl>
void
FullO3CPU<Impl>::deallocateContext(int thread_num)
{
    panic("deallocateContext unimplemented!");
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(int thread_num)
{
    panic("haltContext unimplemented!");
}

template <class Impl>
void
FullO3CPU<Impl>::switchOut()
{
    panic("FullO3CPU does not have a switch out function.\n");
}

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // Set all status's to active, schedule the
    // CPU's tick event.
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        if (xc->status() == ExecContext::Active && _status != Running) {
            _status = Running;
            tickEvent.schedule(curTick);
        }
    }
}

template <class Impl>
InstSeqNum
FullO3CPU<Impl>::getAndIncrementInstSeq()
{
    // Hopefully this works right.
    return globalSeqNum++;
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(int reg_idx)
{
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
float
FullO3CPU<Impl>::readFloatRegSingle(int reg_idx)
{
    return regFile.readFloatRegSingle(reg_idx);
}

template <class Impl>
double
FullO3CPU<Impl>::readFloatRegDouble(int reg_idx)
{
    return regFile.readFloatRegDouble(reg_idx);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readFloatRegInt(int reg_idx)
{
    return regFile.readFloatRegInt(reg_idx);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegSingle(int reg_idx, float val)
{
    regFile.setFloatRegSingle(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegDouble(int reg_idx, double val)
{
    regFile.setFloatRegDouble(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegInt(int reg_idx, uint64_t val)
{
    regFile.setFloatRegInt(reg_idx, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readPC()
{
    return regFile.readPC();
}

template <class Impl>
void
FullO3CPU<Impl>::setNextPC(uint64_t val)
{
    regFile.setNextPC(val);
}

template <class Impl>
void
FullO3CPU<Impl>::setPC(Addr new_PC)
{
    regFile.setPC(new_PC);
}

template <class Impl>
void
FullO3CPU<Impl>::addInst(DynInstPtr &inst)
{
    instList.push_back(inst);
}

template <class Impl>
void
FullO3CPU<Impl>::instDone()
{
    // Keep an instruction count.
    numInsts++;

    // Check for instruction-count-based events.
    comInstEventQueue[0]->serviceEvents(numInsts);
}

template <class Impl>
void
FullO3CPU<Impl>::removeBackInst(DynInstPtr &inst)
{
    DynInstPtr inst_to_delete;

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction, inst.
    while (instList.back() != inst)
    {
        assert(!instList.empty());

        // Obtain the pointer to the instruction.
        inst_to_delete = instList.back();

        DPRINTF(FullCPU, "FullCPU: Removing instruction %i, PC %#x\n",
                inst_to_delete->seqNum, inst_to_delete->readPC());

        // Remove the instruction from the list.
        instList.pop_back();

        // Mark it as squashed.
        inst_to_delete->setSquashed();
    }
}

template <class Impl>
void
FullO3CPU<Impl>::removeFrontInst(DynInstPtr &inst)
{
    DynInstPtr inst_to_remove;

    // The front instruction should be the same one being asked to be removed.
    assert(instList.front() == inst);

    // Remove the front instruction.
    inst_to_remove = inst;
    instList.pop_front();

    DPRINTF(FullCPU, "FullCPU: Removing committed instruction %#x, PC %#x\n",
            inst_to_remove, inst_to_remove->readPC());
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB()
{
    DPRINTF(FullCPU, "FullCPU: Deleting instructions from instruction "
            "list.\n");

    DynInstPtr rob_tail = rob.readTailInst();

    removeBackInst(rob_tail);
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num)
{
    DPRINTF(FullCPU, "FullCPU: Deleting instructions from instruction "
            "list.\n");

    DynInstPtr inst_to_delete;

    while (instList.back()->seqNum > seq_num) {
        assert(!instList.empty());

        // Obtain the pointer to the instruction.
        inst_to_delete = instList.back();

        DPRINTF(FullCPU, "FullCPU: Removing instruction %i, PC %#x\n",
                inst_to_delete->seqNum, inst_to_delete->readPC());

        // Remove the instruction from the list.
        instList.back() = NULL;
        instList.pop_back();

        // Mark it as squashed.
        inst_to_delete->setSquashed();
    }

}

template <class Impl>
void
FullO3CPU<Impl>::removeAllInsts()
{
    instList.clear();
}

template <class Impl>
void
FullO3CPU<Impl>::dumpInsts()
{
    int num = 0;
    typename list<DynInstPtr>::iterator inst_list_it = instList.begin();

    while (inst_list_it != instList.end())
    {
        cprintf("Instruction:%i\nPC:%#x\nSN:%lli\nIssued:%i\nSquashed:%i\n\n",
                num, (*inst_list_it)->readPC(), (*inst_list_it)->seqNum,
                (*inst_list_it)->isIssued(), (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::wakeDependents(DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}

// Forward declaration of FullO3CPU.
template class FullO3CPU<AlphaSimpleImpl>;
