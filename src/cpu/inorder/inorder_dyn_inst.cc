/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
#include <string>
#include <sstream>

#include "arch/faults.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/exetrace.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "mem/request.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

InOrderDynInst::InOrderDynInst(TheISA::ExtMachInst machInst, Addr inst_PC,
                               Addr pred_PC, InstSeqNum seq_num,
                               InOrderCPU *cpu)
  : staticInst(machInst, inst_PC), traceData(NULL), cpu(cpu)
{
    seqNum = seq_num;

    PC = inst_PC;
    nextPC = PC + sizeof(MachInst);
    nextNPC = nextPC + sizeof(MachInst);
    predPC = pred_PC;

    initVars();
}

InOrderDynInst::InOrderDynInst(InOrderCPU *cpu,
                               InOrderThreadState *state,
                               InstSeqNum seq_num,
                               ThreadID tid,
                               unsigned _asid)
    : traceData(NULL), cpu(cpu)
{
    seqNum = seq_num;
    thread = state;
    threadNumber = tid;
    asid = _asid;
    initVars();
}

InOrderDynInst::InOrderDynInst(StaticInstPtr &_staticInst)
    : seqNum(0), staticInst(_staticInst), traceData(NULL)
{
    initVars();
}

InOrderDynInst::InOrderDynInst()
    : seqNum(0), traceData(NULL), cpu(cpu)
{
    initVars();
}

int InOrderDynInst::instcount = 0;


void
InOrderDynInst::setMachInst(ExtMachInst machInst)
{
    staticInst = StaticInst::decode(machInst, PC);

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
    nextInstStageNum = 0;

    for(int i = 0; i < MaxInstDestRegs; i++)
        instResult[i].val.integer = 0;

    status.reset();

    memAddrReady = false;
    eaCalcDone = false;
    memOpDone = false;

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
    ++instcount;
    if (instcount > 100) {
        fatal("Number of Active Instructions in CPU is too high. "
                "(Not Dereferencing Ptrs. Correctly?)\n");
    }



    DPRINTF(InOrderDynInst, "DynInst: [tid:%i] [sn:%lli] Instruction created. (active insts: %i)\n",
            threadNumber, seqNum, instcount);
}

void
InOrderDynInst::resetInstCount()
{
    instcount = 0;
}


InOrderDynInst::~InOrderDynInst()
{
    if (fetchMemReq != 0x0) {
        delete fetchMemReq;
        fetchMemReq = NULL;
    }

    if (dataMemReq != 0x0) {
        delete dataMemReq;
        dataMemReq = NULL;
    }

    if (traceData) {
        delete traceData;
    }

    if (splitMemData) {
        delete [] splitMemData;
    }
    
    fault = NoFault;

    --instcount;

    deleteStages();

    DPRINTF(InOrderDynInst, "DynInst: [tid:%i] [sn:%lli] Instruction destroyed. (active insts: %i)\n",
            threadNumber, seqNum, instcount);
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
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->inSyscall = in_syscall;

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
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}


Fault
InOrderDynInst::completeAcc(Packet *pkt)
{
    this->fault = this->staticInst->completeAcc(pkt, this, this->traceData);

    return this->fault;
}

InstStage *InOrderDynInst::addStage()
{
    this->currentInstStage = new InstStage(this, nextInstStageNum++);
    instStageList.push_back( this->currentInstStage );
    return this->currentInstStage;
}

InstStage *InOrderDynInst::addStage(int stage_num)
{
    nextInstStageNum = stage_num;
    return InOrderDynInst::addStage();
}

void InOrderDynInst::deleteStages() {
    std::list<InstStage*>::iterator list_it = instStageList.begin();
    std::list<InstStage*>::iterator list_end = instStageList.end();

    while(list_it != list_end) {
        delete *list_it;
        list_it++;
    }
}

Fault
InOrderDynInst::memAccess()
{
    return initiateAcc();
}


#if FULL_SYSTEM

Fault
InOrderDynInst::hwrei()
{
    panic("InOrderDynInst: hwrei: unimplemented\n");    
    return NoFault;
}


void
InOrderDynInst::trap(Fault fault)
{
    this->cpu->trap(fault, this->threadNumber);
}


bool
InOrderDynInst::simPalCheck(int palFunc)
{
#if THE_ISA != ALPHA_ISA
    panic("simPalCheck called, but PAL only exists in Alpha!\n");
#endif
    return this->cpu->simPalCheck(palFunc, this->threadNumber);
}
#else
void
InOrderDynInst::syscall(int64_t callnum)
{
    cpu->syscall(callnum, this->threadNumber);
}
#endif

void
InOrderDynInst::prefetch(Addr addr, unsigned flags)
{
    cpu->prefetch(this);
}

void
InOrderDynInst::writeHint(Addr addr, int size, unsigned flags)
{
    cpu->writeHint(this);
}

/**
 * @todo Need to find a way to get the cache block size here.
 */
Fault
InOrderDynInst::copySrcTranslate(Addr src)
{
    // Not currently supported.
    return NoFault;
}

/**
 * @todo Need to find a way to get the cache block size here.
 */
Fault
InOrderDynInst::copy(Addr dest)
{
    // Not currently supported.
    return NoFault;
}

void
InOrderDynInst::releaseReq(ResourceRequest* req)
{
    std::list<ResourceRequest*>::iterator list_it = reqList.begin();
    std::list<ResourceRequest*>::iterator list_end = reqList.end();

    while(list_it != list_end) {
        if((*list_it)->getResIdx() == req->getResIdx() &&
           (*list_it)->getSlot() == req->getSlot()) {
            DPRINTF(InOrderDynInst, "[tid:%u]: [sn:%i] Done with request to %s.\n",
                    threadNumber, seqNum, req->res->name());
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
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Source Value %i being set to %#x.\n",
            threadNumber, seqNum, idx, val);
    instSrc[idx].integer = val;
}

/** Records an fp register being set to a value. */
void
InOrderDynInst::setFloatSrc(int idx, FloatReg val)
{
    instSrc[idx].dbl = val;
}

/** Records an fp register being set to an integer value. */
void
InOrderDynInst::setFloatRegBitsSrc(int idx, uint64_t val)
{
    instSrc[idx].integer = val;
}

/** Reads a integer register. */
IntReg
InOrderDynInst::readIntRegOperand(const StaticInst *si, int idx, ThreadID tid)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Source Value %i read as %#x.\n",
            threadNumber, seqNum, idx, instSrc[idx].integer);
    return instSrc[idx].integer;
}

/** Reads a FP register. */
FloatReg
InOrderDynInst::readFloatRegOperand(const StaticInst *si, int idx)
{
    return instSrc[idx].dbl;
}


/** Reads a FP register as a integer. */
FloatRegBits
InOrderDynInst::readFloatRegOperandBits(const StaticInst *si, int idx)
{
    return instSrc[idx].integer;
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
InOrderDynInst::readMiscRegNoEffect(int misc_reg)
{
    return this->cpu->readMiscRegNoEffect(misc_reg, threadNumber);
}

/** Reads a miscellaneous register. */
MiscReg
InOrderDynInst::readMiscRegOperandNoEffect(const StaticInst *si, int idx)
{
    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Misc. Reg Source Value %i"
            " read as %#x.\n", threadNumber, seqNum, idx,
            instSrc[idx].integer);
    return instSrc[idx].integer;
}

/** Reads a misc. register, including any side-effects the read
 * might have as defined by the architecture.
 */
MiscReg
InOrderDynInst::readMiscRegOperand(const StaticInst *si, int idx)
{
    // For In-Order, the side-effect of reading a register happens
    // when explicitly executing a "ReadSrc" command. This simply returns
    // a value.
    return readMiscRegOperandNoEffect(si, idx);
}

/** Sets a misc. register. */
void
InOrderDynInst::setMiscRegOperandNoEffect(const StaticInst * si, int idx,
                                          const MiscReg &val)
{
    instResult[idx].type = Integer;
    instResult[idx].val.integer = val;
    instResult[idx].tick = curTick;

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Misc Reg. Operand %i "
            "being set to %#x.\n", threadNumber, seqNum, idx, val);
}

/** Sets a misc. register, including any side-effects the write
 * might have as defined by the architecture.
 */
void
InOrderDynInst::setMiscRegOperand(const StaticInst *si, int idx,
                       const MiscReg &val)
{
    // For In-Order, the side-effect of setting a register happens
    // when explicitly writing back the register value. This
    // simply maintains the operand value.
    setMiscRegOperandNoEffect(si, idx, val);
}

MiscReg
InOrderDynInst::readRegOtherThread(unsigned reg_idx, ThreadID tid)
{
    if (tid == -1) {
        tid = TheISA::getTargetThread(this->cpu->tcBase(threadNumber));
    }

    if (reg_idx < FP_Base_DepTag) {                   // Integer Register File
        return this->cpu->readIntReg(reg_idx, tid);
    } else if (reg_idx < Ctrl_Base_DepTag) {          // Float Register File
        reg_idx -= FP_Base_DepTag;
        return this->cpu->readFloatRegBits(reg_idx, tid);
    } else {
        reg_idx -= Ctrl_Base_DepTag;
        return this->cpu->readMiscReg(reg_idx, tid);  // Misc. Register File
    }
}

/** Sets a Integer register. */
void
InOrderDynInst::setIntRegOperand(const StaticInst *si, int idx, IntReg val)
{
    instResult[idx].type = Integer;
    instResult[idx].val.integer = val;
    instResult[idx].tick = curTick;

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Result Int Reg. %i "
            "being set to %#x (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, instResult[idx].tick);
}

/** Sets a FP register. */
void
InOrderDynInst::setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
{
    instResult[idx].val.dbl = val;
    instResult[idx].type = Float;
    instResult[idx].tick = curTick;

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Result Float Reg. %i "
            "being set to %#x (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, instResult[idx].tick);
}

/** Sets a FP register as a integer. */
void
InOrderDynInst::setFloatRegOperandBits(const StaticInst *si, int idx,
                              FloatRegBits val)
{
    instResult[idx].type = Integer;
    instResult[idx].val.integer = val;
    instResult[idx].tick = curTick;

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting Result Float Reg. %i "
            "being set to %#x (result-tick:%i).\n",
            threadNumber, seqNum, idx, val, instResult[idx].tick);
}

/** Sets a misc. register. */
/* Alter this when wanting to *speculate* on Miscellaneous registers */
void
InOrderDynInst::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    this->cpu->setMiscRegNoEffect(misc_reg, val, threadNumber);
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

    if (reg_idx < FP_Base_DepTag) {            // Integer Register File
        this->cpu->setIntReg(reg_idx, val, tid);
    } else if (reg_idx < Ctrl_Base_DepTag) {   // Float Register File
        reg_idx -= FP_Base_DepTag;
        this->cpu->setFloatRegBits(reg_idx, val, tid);
    } else {
        reg_idx -= Ctrl_Base_DepTag;
        this->cpu->setMiscReg(reg_idx, val, tid); // Misc. Register File
    }
}

void
InOrderDynInst::deallocateContext(int thread_num)
{
    this->cpu->deallocateContext(thread_num);
}

template<class T>
inline Fault
InOrderDynInst::read(Addr addr, T &data, unsigned flags)
{
    if (traceData) {
        traceData->setAddr(addr);
        traceData->setData(data);
    }

    return cpu->read(this, addr, data, flags);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
InOrderDynInst::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
InOrderDynInst::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
InOrderDynInst::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
InOrderDynInst::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
InOrderDynInst::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
InOrderDynInst::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}

template<>
Fault
InOrderDynInst::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}

template<class T>
inline Fault
InOrderDynInst::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData) {
        traceData->setAddr(addr);
        traceData->setData(data);
    }

    storeData  = data;

    DPRINTF(InOrderDynInst, "[tid:%i]: [sn:%i] Setting store data to %#x.\n",
            threadNumber, seqNum, storeData);
    return cpu->write(this, data, addr, flags, res);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
InOrderDynInst::write(uint64_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderDynInst::write(uint32_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderDynInst::write(uint16_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
InOrderDynInst::write(uint8_t data, Addr addr,
                       unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
InOrderDynInst::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
InOrderDynInst::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}


template<>
Fault
InOrderDynInst::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


void
InOrderDynInst::dump()
{
    cprintf("T%d : %#08d `", threadNumber, PC);
    cout << staticInst->disassemble(PC);
    cprintf("'\n");
}

void
InOrderDynInst::dump(std::string &outstring)
{
    std::ostringstream s;
    s << "T" << threadNumber << " : 0x" << PC << " "
      << staticInst->disassemble(PC);

    outstring = s.str();
}


#define NOHASH
#ifndef NOHASH

#include "base/hashmap.hh"

unsigned int MyHashFunc(const InOrderDynInst *addr)
{
    unsigned a = (unsigned)addr;
    unsigned hash = (((a >> 14) ^ ((a >> 2) & 0xffff))) & 0x7FFFFFFF;

    return hash;
}

typedef m5::hash_map<const InOrderDynInst *, const InOrderDynInst *, MyHashFunc>
my_hash_t;

my_hash_t thishash;
#endif
