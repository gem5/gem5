#ifndef __SIMPLE_FULL_CPU_CC__
#define __SIMPLE_FULL_CPU_CC__

#ifdef FULL_SYSTEM
#include "sim/system.hh"
#else
#include "sim/process.hh"
#endif
#include "sim/universe.hh"

#include "cpu/exec_context.hh"
#include "cpu/beta_cpu/full_cpu.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/beta_cpu/alpha_dyn_inst.hh"

using namespace std;

#ifdef FULL_SYSTEM
BaseFullCPU::BaseFullCPU(Params &params)
    : BaseCPU(params.name, params.numberOfThreads,
              params.maxInstsAnyThread, params.maxInstsAllThreads,
              params.maxLoadsAnyThread, params.maxLoadsAllThreads,
              params._system, params.freq)
{
}
#else
BaseFullCPU::BaseFullCPU(Params &params)
    : BaseCPU(params.name, params.numberOfThreads,
              params.maxInstsAnyThread, params.maxInstsAllThreads,
              params.maxLoadsAnyThread, params.maxLoadsAllThreads)
{
}
#endif // FULL_SYSTEM

template <class Impl>
FullBetaCPU<Impl>::TickEvent::TickEvent(FullBetaCPU<Impl> *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}

template <class Impl>
void
FullBetaCPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
FullBetaCPU<Impl>::TickEvent::description()
{
    return "FullBetaCPU tick event";
}

//Call constructor to all the pipeline stages here
template <class Impl>
FullBetaCPU<Impl>::FullBetaCPU(Params &params)
#ifdef FULL_SYSTEM
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

      freeList(Impl::ISA::NumIntRegs, params.numPhysIntRegs,
               Impl::ISA::NumFloatRegs, params.numPhysFloatRegs),

      renameMap(Impl::ISA::NumIntRegs, params.numPhysIntRegs,
                Impl::ISA::NumFloatRegs, params.numPhysFloatRegs,
                Impl::ISA::NumMiscRegs,
                Impl::ISA::ZeroReg,
                Impl::ISA::ZeroReg + Impl::ISA::NumIntRegs),

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

#ifdef FULL_SYSTEM
      system(params.system),
      memCtrl(system->memCtrl),
      physmem(system->physmem),
      itb(params.itb),
      dtb(params.dtb),
      mem(params.mem),
#else
      process(params.process),
      asid(params.asid),
      mem(process->getMemory()),
#endif // FULL_SYSTEM

      icacheInterface(params.icacheInterface),
      dcacheInterface(params.dcacheInterface),
      deferRegistration(params.defReg),
      numInsts(0),
      funcExeInst(0)
{
    _status = Idle;
#ifdef FULL_SYSTEM
    xc = new ExecContext(this, 0, system, itb, dtb, mem);

    // initialize CPU, including PC
    TheISA::initCPU(&xc->regs);
#else
    DPRINTF(FullCPU, "FullCPU: Process's starting PC is %#x, process is %#x",
            process->prog_entry, process);
    xc = new ExecContext(this, /* thread_num */ 0, process, /* asid */ 0);

    assert(process->getMemory() != NULL);
    assert(mem != NULL);
#endif // !FULL_SYSTEM
    execContexts.push_back(xc);

    // The stages also need their CPU pointer setup.  However this must be
    // done at the upper level CPU because they have pointers to the upper
    // level CPU, and not this FullBetaCPU.

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
FullBetaCPU<Impl>::~FullBetaCPU()
{
}

template <class Impl>
void
FullBetaCPU<Impl>::fullCPURegStats()
{
    // Register any of the FullCPU's stats here.
}

template <class Impl>
void
FullBetaCPU<Impl>::tick()
{
    DPRINTF(FullCPU, "\n\nFullCPU: Ticking main, FullBetaCPU.\n");

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
FullBetaCPU<Impl>::init()
{
    if(!deferRegistration)
    {
        this->registerExecContexts();

        // Need to do a copy of the xc->regs into the CPU's regfile so
        // that it can start properly.

        // First loop through the integer registers.
        for (int i = 0; i < Impl::ISA::NumIntRegs; ++i)
        {
            regFile.intRegFile[i] = xc->regs.intRegFile[i];
        }

        // Then loop through the floating point registers.
        for (int i = 0; i < Impl::ISA::NumFloatRegs; ++i)
        {
            regFile.floatRegFile[i].d = xc->regs.floatRegFile.d[i];
            regFile.floatRegFile[i].q = xc->regs.floatRegFile.q[i];
        }

        // Then loop through the misc registers.
        regFile.miscRegs.fpcr = xc->regs.miscRegs.fpcr;
        regFile.miscRegs.uniq = xc->regs.miscRegs.uniq;
        regFile.miscRegs.lock_flag = xc->regs.miscRegs.lock_flag;
        regFile.miscRegs.lock_addr = xc->regs.miscRegs.lock_addr;

        // Then finally set the PC and the next PC.
        regFile.pc = xc->regs.pc;
        regFile.npc = xc->regs.npc;
    }
}

template <class Impl>
void
FullBetaCPU<Impl>::activateContext(int thread_num, int delay)
{
    // Needs to set each stage to running as well.

    scheduleTickEvent(delay);

    _status = Running;
}

template <class Impl>
void
FullBetaCPU<Impl>::suspendContext(int thread_num)
{
    panic("suspendContext unimplemented!");
}

template <class Impl>
void
FullBetaCPU<Impl>::deallocateContext(int thread_num)
{
    panic("deallocateContext unimplemented!");
}

template <class Impl>
void
FullBetaCPU<Impl>::haltContext(int thread_num)
{
    panic("haltContext unimplemented!");
}

template <class Impl>
void
FullBetaCPU<Impl>::switchOut()
{
    panic("FullBetaCPU does not have a switch out function.\n");
}

template <class Impl>
void
FullBetaCPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // Set all status's to active, schedule the
    // CPU's tick event.
    tickEvent.schedule(curTick);
    for (int i = 0; i < execContexts.size(); ++i) {
        execContexts[i]->activate();
    }

    // Switch out the other CPU.
    oldCPU->switchOut();
}

template <class Impl>
InstSeqNum
FullBetaCPU<Impl>::getAndIncrementInstSeq()
{
    // Hopefully this works right.
    return globalSeqNum++;
}

template <class Impl>
uint64_t
FullBetaCPU<Impl>::readIntReg(int reg_idx)
{
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
float
FullBetaCPU<Impl>::readFloatRegSingle(int reg_idx)
{
    return regFile.readFloatRegSingle(reg_idx);
}

template <class Impl>
double
FullBetaCPU<Impl>::readFloatRegDouble(int reg_idx)
{
    return regFile.readFloatRegDouble(reg_idx);
}

template <class Impl>
uint64_t
FullBetaCPU<Impl>::readFloatRegInt(int reg_idx)
{
    return regFile.readFloatRegInt(reg_idx);
}

template <class Impl>
void
FullBetaCPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullBetaCPU<Impl>::setFloatRegSingle(int reg_idx, float val)
{
    regFile.setFloatRegSingle(reg_idx, val);
}

template <class Impl>
void
FullBetaCPU<Impl>::setFloatRegDouble(int reg_idx, double val)
{
    regFile.setFloatRegDouble(reg_idx, val);
}

template <class Impl>
void
FullBetaCPU<Impl>::setFloatRegInt(int reg_idx, uint64_t val)
{
    regFile.setFloatRegInt(reg_idx, val);
}

template <class Impl>
uint64_t
FullBetaCPU<Impl>::readPC()
{
    return regFile.readPC();
}

template <class Impl>
void
FullBetaCPU<Impl>::setNextPC(uint64_t val)
{
    regFile.setNextPC(val);
}

template <class Impl>
void
FullBetaCPU<Impl>::setPC(Addr new_PC)
{
    regFile.setPC(new_PC);
}

template <class Impl>
void
FullBetaCPU<Impl>::addInst(DynInstPtr &inst)
{
    instList.push_back(inst);
}

template <class Impl>
void
FullBetaCPU<Impl>::instDone()
{
    // Keep an instruction count.
    numInsts++;

    // Check for instruction-count-based events.
    comInstEventQueue[0]->serviceEvents(numInsts);
}

template <class Impl>
void
FullBetaCPU<Impl>::removeBackInst(DynInstPtr &inst)
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
FullBetaCPU<Impl>::removeFrontInst(DynInstPtr &inst)
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
FullBetaCPU<Impl>::removeInstsNotInROB()
{
    DPRINTF(FullCPU, "FullCPU: Deleting instructions from instruction "
            "list.\n");

    DynInstPtr rob_tail = rob.readTailInst();

    removeBackInst(rob_tail);
}

template <class Impl>
void
FullBetaCPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num)
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
        instList.pop_back();

        // Mark it as squashed.
        inst_to_delete->setSquashed();
    }

}

template <class Impl>
void
FullBetaCPU<Impl>::removeAllInsts()
{
    instList.clear();
}

template <class Impl>
void
FullBetaCPU<Impl>::dumpInsts()
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
FullBetaCPU<Impl>::wakeDependents(DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}

// Forward declaration of FullBetaCPU.
template FullBetaCPU<AlphaSimpleImpl>;

#endif // __SIMPLE_FULL_CPU_HH__
