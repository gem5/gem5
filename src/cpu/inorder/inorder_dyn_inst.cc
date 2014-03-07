/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Authors: Korey Sewell
 *
 */

#include <iostream>
#include <set>
#include <sstream>
#include <string>

#include "base/bigint.hh"
#include "base/cp_annotate.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/exetrace.hh"
#include "cpu/reg_class.hh"
#include "debug/InOrderDynInst.hh"
#include "mem/request.hh"
#include "sim/fault_fwd.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

InOrderDynInst::InOrderDynInst(InOrderCPU *cpu,
                               InOrderThreadState *state,
                               InstSeqNum seq_num,
                               ThreadID tid,
                               unsigned _asid)
  : seqNum(seq_num), squashSeqNum(0), threadNumber(tid), asid(_asid),
    virtProcNumber(0), staticInst(NULL), traceData(NULL), cpu(cpu),
    thread(state), fault(NoFault), memData(NULL), loadData(0),
    storeData(0), effAddr(0), physEffAddr(0), memReqFlags(0),
    readyRegs(0), pc(0), predPC(0), memAddr(0), nextStage(0),
    memTime(0), splitMemData(NULL), splitMemReq(NULL), totalSize(0),
    split2ndSize(0), split2ndAddr(0), split2ndAccess(false),
    split2ndDataPtr(NULL), split2ndFlags(0), splitInst(false),
    splitFinishCnt(0), split2ndStoreDataPtr(NULL), splitInstSked(false),
    inFrontEnd(true), frontSked(NULL), backSked(NULL),
    squashingStage(0), predictTaken(false), procDelaySlotOnMispred(false),
    fetchMemReq(NULL), dataMemReq(NULL), instEffAddr(0), eaCalcDone(false),
    lqIdx(0), sqIdx(0), onInstList(false)
{
    for(int i = 0; i < MaxInstSrcRegs; i++) {
        _readySrcRegIdx[i] = false;
        _srcRegIdx[i] = 0;
    }

    for(int j = 0; j < MaxInstDestRegs; j++) {
      _destRegIdx[j] = 0;
      _prevDestRegIdx[j] = 0;
    }

    ++instcount;
    DPRINTF(InOrderDynInst, "DynInst: [tid:%i] [sn:%lli] Instruction created."
            " (active insts: %i)\n", threadNumber, seqNum, instcount);

}

int InOrderDynInst::instcount = 0;

int
InOrderDynInst::cpuId() const
{
    return cpu->cpuId();
}

void
InOrderDynInst::setStaticInst(StaticInstPtr si)
{
    staticInst = si;

    for (int i = 0; i < this->staticInst->numDestRegs(); i++) {
        _destRegIdx[i] = this->staticInst->destRegIdx(i);
    }

    for (int i = 0; i < this->staticInst->numSrcRegs(); i++) {
        _srcRegIdx[i] = this->staticInst->srcRegIdx(i);
        this->_readySrcRegIdx[i] = 0;
    }
}

void
InOrderDynInst::initVars()
{
    inFrontEnd = true;

    fetchMemReq = NULL;
    dataMemReq = NULL;
    splitMemData = NULL;
    split2ndAddr = 0;
    split2ndAccess = false;
    splitInst = false;
    splitInstSked = false;    
    splitFinishCnt = 0;
    
    effAddr = 0;
    physEffAddr = 0;

    readyRegs = 0;

    nextStage = 0;

    status.reset();

    memAddrReady = false;
    eaCalcDone = false;

    predictTaken = false;
    procDelaySlotOnMispred = false;

    lqIdx = -1;
    sqIdx = -1;

    // Also make this a parameter, or perhaps get it from xc or cpu.
    asid = 0;

    virtProcNumber = 0;

    // Initialize the fault to be NoFault.
    fault = NoFault;

    // Make sure to have the renamed register entries set to the same
    // as the normal register entries.  It will allow the IQ to work
    // without any modifications.
    if (this->staticInst) {
        for (int i = 0; i < this->staticInst->numDestRegs(); i++) {
            _destRegIdx[i] = this->staticInst->destRegIdx(i);
        }

        for (int i = 0; i < this->staticInst->numSrcRegs(); i++) {
            _srcRegIdx[i] = this->staticInst->srcRegIdx(i);
            this->_readySrcRegIdx[i] = 0;
        }
    }

    // Update Instruction Count for this instruction
    if (instcount > 100) {
        fatal("Number of Active Instructions in CPU is too high. "
                "(Not Dereferencing Ptrs. Correctly?)\n");
    }
}

void
InOrderDynInst::resetInstCount()
{
    instcount = 0;
}


InOrderDynInst::~InOrderDynInst()
{
    if (traceData)
        delete traceData;

    if (splitMemData)
        delete [] splitMemData;

    fault = NoFault;

    --instcount;

    DPRINTF(InOrderDynInst, "DynInst: [tid:%i] [sn:%lli] Instruction destroyed"
            " (active insts: %i)\n", threadNumber, seqNum, instcount);
}

void
InOrderDynInst::setStaticInst(StaticInstPtr &static_inst)
{
    this->staticInst = static_inst;

    // Make sure to have the renamed register entries set to the same
    // as the normal register entries.  It will allow the IQ to work
    // without any modifications.
    if (this->staticInst) {
        for (int i = 0; i < this->staticInst->numDestRegs(); i++) {
            _destRegIdx[i] = this->staticInst->destRegIdx(i);
        }

        for (int i = 0; i < this->staticInst->numSrcRegs(); i++) {
            _srcRegIdx[i] = this->staticInst->srcRegIdx(i);
            this->_readySrcRegIdx[i] = 0;
        }
    }
}

Fault
InOrderDynInst::execute()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

Fault
InOrderDynInst::calcEA()
{
    this->fault = this->staticInst->eaComp(this, this->traceData);
    return this->fault;
}

Fault
InOrderDynInst::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}


Fault
InOrderDynInst::completeAcc(Packet *pkt)
{
    this->fault = this->staticInst->completeAcc(pkt, this, this->traceData);

    return this->fault;
}

Fault
InOrderDynInst::memAccess()
{
    return initiateAcc();
}


Fault
InOrderDynInst::hwrei()
{
#if THE_ISA == ALPHA_ISA
    // Can only do a hwrei when in pal mode.
    if (!(this->instAddr() & 0x3))
        return new AlphaISA::UnimplementedOpcodeFault;

    // Set the next PC based on the value of the EXC_ADDR IPR.
    AlphaISA::PCState pc = this->pcState();
    pc.npc(this->cpu->readMiscRegNoEffect(AlphaISA::IPR_EXC_ADDR,
                                          this->threadNumber));
    this->pcState(pc);
    if (CPA::available()) {
        ThreadContext *tc = this->cpu->tcBase(this->threadNumber);
        CPA::cpa()->swAutoBegin(tc, this->nextInstAddr());
    }

    // Tell CPU to clear any state it needs to if a hwrei is taken.
    this->cpu->hwrei(this->threadNumber);
#endif
    return NoFault;
}


void
InOrderDynInst::trap(Fault fault)
{
    this->cpu->trap(fault, this->threadNumber, this);
}


bool
InOrderDynInst::simPalCheck(int palFunc)
{
#if THE_ISA != ALPHA_ISA
    panic("simPalCheck called, but PAL only exists in Alpha!\n");
#endif
    return this->cpu->simPalCheck(palFunc, this->threadNumber);
}

void
InOrderDynInst::syscall(int64_t callnum)
{
    if (FullSystem)
        panic("Syscall emulation isn't available in FS mode.\n");

    syscallNum = callnum;
    cpu->syscallContext(NoFault, this->threadNumber, this);
}

void
InOrderDynInst::setSquashInfo(unsigned stage_num)
{
    squashingStage = stage_num;

    // If it's a fault, then we need to squash
    // the faulting instruction too. Squash
    // functions squash above a seqNum, so we
    // decrement here for that case
    if (fault != NoFault) {
        squashSeqNum = seqNum - 1;
        return;
    } else
        squashSeqNum = seqNum;

#if ISA_HAS_DELAY_SLOT
    if (staticInst && isControl()) {
        TheISA::PCState nextPC = pc;
        TheISA::advancePC(nextPC, staticInst);

        // Check to see if we should squash after the
        // branch or after a branch delay slot.
        if (pc.nextInstAddr() == pc.instAddr() + sizeof(MachInst))
            squashSeqNum = seqNum + 1;
        else
            squashSeqNum = seqNum;
    }
#endif
}

void
InOrderDynInst::releaseReq(ResourceRequest* req)
{
    std::list<ResourceRequest*>::iterator list_it = reqList.begin();
    std::list<ResourceRequest*>::iterator list_end = reqList.end();

    while(list_it != list_end) {
        if((*list_it)->getResIdx() == req->getResIdx() &&
           (*list_it)->getSlot() == req->getSlot()) {
            DPRINTF(InOrderDynInst, "[tid:%u]: [sn:%i] Done with request "
                    "to %s.\n", threadNumber, seqNum, req->res->name());
            reqList.erase(list_it);
            return;
        }
        list_it++;
    }

    panic("Releasing Res. Request That Isnt There!\n");
}

/** Records an integer source register being set to a value. */
void
InOrderDynInst::setIntSrc(int idx, uint64_t val)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] Int being set "
            "to %#x.\n", threadNumber, seqNum, idx, val);
    instSrc[idx].intVal = val;
}

/** Records an fp register being set to a value. */
void
InOrderDynInst::setFloatSrc(int idx, FloatReg val)
{
    instSrc[idx].fpVal.f = val;
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] FP being set "
            "to %x, %08f...%08f\n", threadNumber, seqNum, idx,
            instSrc[idx].fpVal.i, instSrc[idx].fpVal.f, val);
}

/** Records an fp register being set to an integer value. */
void
InOrderDynInst::setFloatRegBitsSrc(int idx, FloatRegBits val)
{
    instSrc[idx].fpVal.i = val;
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] FPBits being set "
            "to %x, %08f...%x\n", threadNumber, seqNum, idx,
            instSrc[idx].fpVal.i, instSrc[idx].fpVal.f, val);
}

/** Reads a integer register. */
IntReg
InOrderDynInst::readIntRegOperand(const StaticInst *si, int idx, ThreadID tid)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] IntVal read as %#x.\n",
            threadNumber, seqNum, idx, instSrc[idx].intVal);
    return instSrc[idx].intVal;
}

/** Reads a FP register. */
FloatReg
InOrderDynInst::readFloatRegOperand(const StaticInst *si, int idx)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] FPVal being read "
            "as %x, %08f.\n", threadNumber, seqNum, idx,
            instSrc[idx].fpVal.i, instSrc[idx].fpVal.f);
    return instSrc[idx].fpVal.f;
}


/** Reads a FP register as a integer. */
FloatRegBits
InOrderDynInst::readFloatRegOperandBits(const StaticInst *si, int idx)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] [src:%i] FPBits being read "
            "as %x, %08f.\n", threadNumber, seqNum, idx,
            instSrc[idx].fpVal.i, instSrc[idx].fpVal.f);
    return instSrc[idx].fpVal.i;
}

/** Reads a miscellaneous register. */
MiscReg
InOrderDynInst::readMiscReg(int misc_reg)
{
    return this->cpu->readMiscReg(misc_reg, threadNumber);
}


/** Reads a misc. register, including any side-effects the read
 * might have as defined by the architecture.
 */
MiscReg
InOrderDynInst::readMiscRegOperand(const StaticInst *si, int idx)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Misc. Reg Source Value %i"
            " read as %#x.\n", threadNumber, seqNum, idx,
            instSrc[idx].intVal);
    return instSrc[idx].intVal;
}


/** Sets a misc. register, including any side-effects the write
 * might have as defined by the architecture.
 */
void
InOrderDynInst::setMiscRegOperand(const StaticInst *si, int idx,
                       const MiscReg &val)
{
    instResult[idx].type = Integer;
    instResult[idx].res.intVal = val;
    instResult[idx].tick = curTick();

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Misc Reg. Operand %i "
            "being set to %#x.\n", threadNumber, seqNum, idx, val);
}

MiscReg
InOrderDynInst::readRegOtherThread(unsigned reg_idx, ThreadID tid)
{
    if (tid == -1) {
        tid = TheISA::getTargetThread(this->cpu->tcBase(threadNumber));
    }

    RegIndex rel_idx;

    switch (regIdxToClass(reg_idx, &rel_idx)) {
      case IntRegClass:
        return this->cpu->readIntReg(rel_idx, tid);

      case FloatRegClass:
        return this->cpu->readFloatRegBits(rel_idx, tid);

      case MiscRegClass:
        return this->cpu->readMiscReg(rel_idx, tid);  // Misc. Register File

      default:
        panic("register %d out of range\n", reg_idx);

    }
}

/** Sets a Integer register. */
void
InOrderDynInst::setIntRegOperand(const StaticInst *si, int idx, IntReg val)
{
    instResult[idx].type = Integer;
    instResult[idx].res.intVal = val;
    instResult[idx].tick = curTick();

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Result Int Reg. %i "
            "being set to %#x (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, instResult[idx].tick);
}

/** Sets a FP register. */
void
InOrderDynInst::setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
{
    instResult[idx].type = Float;
    instResult[idx].res.fpVal.f = val;
    instResult[idx].tick = curTick();

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Result Float Reg. %i "
            "being set to %#x, %08f (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, val, instResult[idx].tick);
}

/** Sets a FP register as a integer. */
void
InOrderDynInst::setFloatRegOperandBits(const StaticInst *si, int idx,
                              FloatRegBits val)
{
    instResult[idx].type = FloatBits;
    instResult[idx].res.fpVal.i = val;
    instResult[idx].tick = curTick();

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Result Float Reg. Bits %i "
            "being set to %#x (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, instResult[idx].tick);
}

/** Sets a misc. register, including any side-effects the write
 * might have as defined by the architecture.
 */
/* Alter this if/when wanting to *speculate* on Miscellaneous registers */
void
InOrderDynInst::setMiscReg(int misc_reg, const MiscReg &val)
{
    this->cpu->setMiscReg(misc_reg, val, threadNumber);
}

void
InOrderDynInst::setRegOtherThread(unsigned reg_idx, const MiscReg &val,
                                  ThreadID tid)
{
    if (tid == InvalidThreadID) {
        tid = TheISA::getTargetThread(this->cpu->tcBase(threadNumber));
    }

    RegIndex rel_idx;

    switch (regIdxToClass(reg_idx, &rel_idx)) {
      case IntRegClass:
        this->cpu->setIntReg(rel_idx, val, tid);
        break;

      case FloatRegClass:
        this->cpu->setFloatRegBits(rel_idx, val, tid);
        break;

      case CCRegClass:
        this->cpu->setCCReg(rel_idx, val, tid);
        break;

      case MiscRegClass:
        this->cpu->setMiscReg(rel_idx, val, tid); // Misc. Register File
        break;
    }
}

void
InOrderDynInst::deallocateContext(int thread_num)
{
    this->cpu->deallocateContext(thread_num);
}

Fault
InOrderDynInst::readMem(Addr addr, uint8_t *data,
                        unsigned size, unsigned flags)
{
    return cpu->read(this, addr, data, size, flags);
}

Fault
InOrderDynInst::writeMem(uint8_t *data, unsigned size,
                         Addr addr, unsigned flags, uint64_t *res)
{
    return cpu->write(this, data, size, addr, flags, res);
}


void
InOrderDynInst::dump()
{
    cprintf("T%d : %#08d `", threadNumber, pc.instAddr());
    cout << staticInst->disassemble(pc.instAddr());
    cprintf("'\n");
}

void
InOrderDynInst::dump(std::string &outstring)
{
    std::ostringstream s;
    s << "T" << threadNumber << " : " << pc << " "
      << staticInst->disassemble(pc.instAddr());

    outstring = s.str();
}
