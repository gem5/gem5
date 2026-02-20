/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SALAM_LLVM_INSTRUCTION_HH__
#define __SALAM_LLVM_INSTRUCTION_HH__

#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Value.h>

#include <cstdlib>
#include <iostream>

#include "../HWModeling/hw_interface.hh"
#include "basic_block.hh"
#include "debug_flags.hh"
#include "mem_request.hh"
#include "operand.hh"
#include "value.hh"

namespace SALAM
{

class BasicBlock; // Required Declaration

//---------------------------------------------------------------------------//
//--------- Instruction Base Class ------------------------------------------//
//---------------------------------------------------------------------------//

class Instruction : public Value
{
  private:
    std::map<uint64_t, std::shared_ptr<SALAM::Instruction>>
        dynamicDependencies;
    std::vector<std::shared_ptr<SALAM::Instruction>> dynamicUsers;
    uint64_t llvmOpCode;
    uint64_t cycleCount;
    uint64_t currentCycle;
    uint64_t functional_unit = 0;
    HWInterface *hw_interface;

  protected:
    valueListTy staticDependencies;
    // Operands
    std::vector<SALAM::Operand> operands;

    bool running = false;
    class Instruction_Debugger : public Debugger
    {
      public:
        Instruction_Debugger();
        ~Instruction_Debugger() = default;
        using SALAM::Debugger::dumper;
        virtual void dumper(SALAM::Instruction *inst);
    };
    Instruction_Debugger *inst_dbg;
    bool launched = false;
    bool committed = false;
    bool isready = false;

  public:
    Instruction(uint64_t id, gem5::SimObject *owner, bool dbg); //
    Instruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                uint64_t OpCode); //
    Instruction(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles); //
    Instruction(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t functional_unit); //
    ~Instruction();                                         //
    bool
    operator==(const std::shared_ptr<SALAM::Instruction> inst) const
    {
        return this->getUID() == inst->getUID();
    }
    bool
    operator!=(const std::shared_ptr<SALAM::Instruction> inst) const
    {
        return !operator==(inst);
    }
    using SALAM::Value::initialize;
    virtual void initialize(llvm::Value *irval, irvmap *irmap,
                            SALAM::valueListTy *valueList); //
    virtual std::shared_ptr<SALAM::BasicBlock>
    getTarget()
    {
        return nullptr;
    }
    uint64_t
    getDependencyCount()
    {
        return dynamicDependencies.size();
    }
    virtual uint64_t
    getCycleCount()
    {
        return cycleCount;
    }
    virtual uint64_t
    getOpode()
    {
        return llvmOpCode;
    }
    uint64_t
    getCurrentCycle()
    {
        return currentCycle;
    }
    virtual valueListTy
    getStaticDependencies() const
    {
        return staticDependencies;
    }
    std::map<uint64_t, std::shared_ptr<SALAM::Instruction>>
    getDynamicDependencies() const
    {
        return dynamicDependencies;
    }
    std::shared_ptr<SALAM::Value>
    getStaticDependencies(int i) const
    {
        return staticDependencies.at(i);
    }
    std::shared_ptr<SALAM::Value>
    getDynamicDependencies(int i) const
    {
        return dynamicDependencies.at(i);
    }
    virtual std::vector<uint64_t> runtimeInitialize();
    void removeDynamicDependency(uint64_t opuid);
    void
    addRuntimeDependency(std::shared_ptr<SALAM::Instruction> dep)
    {
        dynamicDependencies.insert({dep->getUID(), dep});
    }
    void
    addRuntimeUser(std::shared_ptr<SALAM::Instruction> dep)
    {
        dynamicUsers.push_back(dep);
    }
    void signalUsers();
    bool
    isCommitted()
    {
        return committed;
    }
    bool
    hasFunctionalUnit()
    {
        return false;
    }
    bool
    debug()
    {
        return dbg;
    }
    void linkOperands(const SALAM::Operand &newOp);
    std::vector<SALAM::Operand> *
    getOperands()
    {
        return &operands;
    }
    uint64_t
    getFunctionalUnit()
    {
        return functional_unit;
    }
    virtual bool
    isReturn()
    {
        return false;
    }
    virtual bool
    isTerminator()
    {
        return false;
    }
    virtual bool
    isPhi()
    {
        return false;
    }
    virtual bool
    isCall()
    {
        return false;
    }
    virtual bool
    isBr()
    {
        return false;
    }
    virtual bool
    isLoad()
    {
        return false;
    }
    virtual bool
    isStore()
    {
        return false;
    }
    virtual bool
    isGEP()
    {
        return false;
    }
    virtual bool launch();
    virtual bool commit();
    virtual bool ready();
    virtual void
    compute()
    {}
    virtual void reset();
    virtual void setOperandValue(uint64_t uid);
    virtual void
    dump()
    {
        if (dbg) {
            inst_dbg->dumper(this);
        }
    }
    virtual bool
    isInstruction()
    {
        return true;
    }
    virtual bool
    isLoadingInternal()
    {
        return false;
    }
    virtual bool
    isLatchingBrExiting()
    {
        return false;
    }
    std::shared_ptr<SALAM::Instruction>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Instruction>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Instruction>(
            new SALAM::Instruction(*this));
    }
    virtual MemoryRequest *
    createMemoryRequest()
    {
        return nullptr;
    }

    // Functions for getting data from operands
    uint64_t
    getPtrOperandValue(uint64_t op_num)
    {
        return (operands.at(op_num).getPtrRegValue());
    }
};

//---------------------------------------------------------------------------//
//--------- Bad Instruction -------------------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-BadInstruction // --------------------------------------------------//
class BadInstruction : public Instruction
{
    // Used to draw hard dependencies, ie: ret
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;

  protected:
  public:
    BadInstruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu);
    ~BadInstruction() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    std::shared_ptr<SALAM::BadInstruction>
    clone() const
    {
        return std::static_pointer_cast<SALAM::BadInstruction>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::BadInstruction>(
            new SALAM::BadInstruction(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createBadInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);

//---------------------------------------------------------------------------//
//--------- Terminator Instructions -----------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-Ret // -------------------------------------------------------------//

class Ret : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Ret(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Ret() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    bool
    isReturn() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Ret>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Ret>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Ret>(new SALAM::Ret(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createRetInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);

// SALAM-Br // --------------------------------------------------------------//

class Br : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    std::shared_ptr<SALAM::Value> condition;
    std::shared_ptr<SALAM::BasicBlock> defaultDestination;
    std::shared_ptr<SALAM::BasicBlock> trueDestination;
    std::shared_ptr<SALAM::BasicBlock> falseDestination;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;
    bool conditional = false;
    bool isLatching = false;

  protected:
  public:
    // Branch Constructor
    Br(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
       uint64_t cycles, uint64_t fu);
    ~Br() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    Br &
    isConditional(bool isConditional)
    {
        conditional = isConditional;
        return *this;
    }
    bool
    isConditional()
    {
        return conditional;
    }
    std::shared_ptr<SALAM::BasicBlock> getTarget() override;
    bool
    isTerminator() override
    {
        return true;
    }
    bool
    isBr() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    setLatching(bool latch)
    {
        isLatching = latch;
    }
    virtual bool
    isLatchingBrExiting() override
    {
        return isLatching && (getTarget() == trueDestination);
    }
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Br>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Br>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Br>(new SALAM::Br(*this));
    }
};

std::shared_ptr<SALAM::Instruction> createBrInst(uint64_t id,
                                                 gem5::SimObject *owner,
                                                 bool dbg, uint64_t OpCode,
                                                 uint64_t cycles, uint64_t fu);

// SALAM-Switch // ----------------------------------------------------------//
typedef std::pair<std::shared_ptr<SALAM::Value>,
                  std::shared_ptr<SALAM::BasicBlock>>
    caseArgs;
typedef std::vector<caseArgs> switchArgs;

class Switch : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    switchArgs cases;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;
    std::shared_ptr<SALAM::BasicBlock> defaultDestination;

  protected:
  public:
    Switch(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~Switch() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    std::shared_ptr<SALAM::BasicBlock> getTarget() override;
    bool
    isTerminator() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Switch>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Switch>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Switch>(new SALAM::Switch(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSwitchInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- Binary Operator Instructions ------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-Add // -------------------------------------------------------------//

class Add : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Add(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Add() = default;
    void initialize(llvm::Value *irval, SALAM::irvmap *irmap,
                    SALAM::valueListTy *valueList) override;
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Add>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Add>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Add>(new SALAM::Add(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createAddInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);

// SALAM-FAdd // ------------------------------------------------------------//

class FAdd : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FAdd(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FAdd() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FAdd>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FAdd>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FAdd>(new SALAM::FAdd(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFAddInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-Sub // -------------------------------------------------------------//

class Sub : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Sub(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Sub() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Sub>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Sub>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Sub>(new SALAM::Sub(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSubInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
// SALAM-FSub // ------------------------------------------------------------//

class FSub : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FSub(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FSub() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FSub>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FSub>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FSub>(new SALAM::FSub(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFSubInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-Mul // -------------------------------------------------------------//

class Mul : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::APIntRegister *op1, *op2;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Mul(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Mul() = default;
    void initialize(llvm::Value *irval, SALAM::irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Mul>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Mul>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Mul>(new SALAM::Mul(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createMulInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
// SALAM-FMul // ------------------------------------------------------------//

class FMul : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FMul(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FMul() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FMul>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FMul>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FMul>(new SALAM::FMul(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFMulInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-UDiv // ------------------------------------------------------------//

class UDiv : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    UDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~UDiv() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::UDiv>
    clone() const
    {
        return std::static_pointer_cast<SALAM::UDiv>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::UDiv>(new SALAM::UDiv(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createUDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-SDiv // ------------------------------------------------------------//

class SDiv : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    SDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~SDiv() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::SDiv>
    clone() const
    {
        return std::static_pointer_cast<SALAM::SDiv>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::SDiv>(new SALAM::SDiv(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-FDiv // ------------------------------------------------------------//

class FDiv : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FDiv() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FDiv>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FDiv>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FDiv>(new SALAM::FDiv(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-URem // ------------------------------------------------------------//

class URem : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    URem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~URem() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::URem>
    clone() const
    {
        return std::static_pointer_cast<SALAM::URem>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::URem>(new SALAM::URem(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createURemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-SRem // ------------------------------------------------------------//

class SRem : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    SRem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~SRem() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::SRem>
    clone() const
    {
        return std::static_pointer_cast<SALAM::SRem>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::SRem>(new SALAM::SRem(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSRemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-FRem // ------------------------------------------------------------//

class FRem : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FRem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FRem() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FRem>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FRem>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FRem>(new SALAM::FRem(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFRemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- Bitwise Binary Operator Instructions ----------------------------//
//---------------------------------------------------------------------------//

// SALAM-Shl // -------------------------------------------------------------//

class Shl : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Shl(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Shl() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Shl>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Shl>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Shl>(new SALAM::Shl(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createShlInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
// SALAM-LShr // ------------------------------------------------------------//

class LShr : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    LShr(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~LShr() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::LShr>
    clone() const
    {
        return std::static_pointer_cast<SALAM::LShr>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::LShr>(new SALAM::LShr(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createLShrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-AShr // ------------------------------------------------------------//

class AShr : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    AShr(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~AShr() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::AShr>
    clone() const
    {
        return std::static_pointer_cast<SALAM::AShr>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::AShr>(new SALAM::AShr(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createAShrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-And // -------------------------------------------------------------//

class And : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    And(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~And() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::And>
    clone() const
    {
        return std::static_pointer_cast<SALAM::And>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::And>(new SALAM::And(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createAndInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
// SALAM-Or // --------------------------------------------------------------//

class Or : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Or(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
       uint64_t cycles, uint64_t fu);
    ~Or() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Or>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Or>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Or>(new SALAM::Or(*this));
    }
};

std::shared_ptr<SALAM::Instruction> createOrInst(uint64_t id,
                                                 gem5::SimObject *owner,
                                                 bool dbg, uint64_t OpCode,
                                                 uint64_t cycles, uint64_t fu);
// SALAM-Xor // -------------------------------------------------------------//

class Xor : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Xor(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Xor() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Xor>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Xor>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Xor>(new SALAM::Xor(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createXorInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- Memory Instructions ---------------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-Load // ------------------------------------------------------------//

class Load : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    uint64_t align;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;
    bool loadingInternal = false;

  protected:
  public:
    Load(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~Load() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    bool
    isLoad() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void loadInternal();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    bool
    isLoadingInternal()
    {
        return loadingInternal;
    }
    std::shared_ptr<SALAM::Load>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Load>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Load>(new SALAM::Load(*this));
    }

    MemoryRequest *createMemoryRequest() override;
};

std::shared_ptr<SALAM::Instruction>
createLoadInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-Store // -----------------------------------------------------------//

class Store : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    uint64_t align;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Store(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
          uint64_t cycles, uint64_t fu);
    ~Store() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    bool
    isStore() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Store>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Store>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Store>(new SALAM::Store(*this));
    }

    MemoryRequest *createMemoryRequest() override;
};

std::shared_ptr<SALAM::Instruction>
createStoreInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu);
// SALAM-GEP // -------------------------------------------------------------//

/*

In our storage, pointers are standard uint64_t, for comm interface convience
The GEP indecies will by APSInts, so cast to int64_t for calculating offset
inside GEP, then recast to APSInt

*/

class GetElementPtr : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    std::vector<int64_t> offsets;
    std::vector<bool> offsetOfStruct;
    SALAM::Debugger *dbgr;
    llvm::Type *resultElementType;
    uint64_t resultElementSize;
    uint64_t resultElementSizeInBytes;

  protected:
  public:
    GetElementPtr(uint64_t id, gem5::SimObject *owner, bool dbg,
                  uint64_t OpCode, uint64_t cycles, uint64_t fu);
    ~GetElementPtr() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    GetElementPtr &
    setA()
    {
        std::cout << "a\n";
        return *this;
    }
    GetElementPtr &
    setB()
    {
        std::cout << "b\n";
        return *this;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    virtual bool
    isGEP() override
    {
        return true;
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::GetElementPtr>
    clone() const
    {
        return std::static_pointer_cast<SALAM::GetElementPtr>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::GetElementPtr>(
            new SALAM::GetElementPtr(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createGetElementPtrInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                        uint64_t OpCode, uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- Other / Cast Instructions ---------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-Trunc // -----------------------------------------------------------//

class Trunc : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Trunc(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
          uint64_t cycles, uint64_t fu);
    ~Trunc() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Trunc>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Trunc>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Trunc>(new SALAM::Trunc(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createTruncInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu);
// SALAM-ZExt // ------------------------------------------------------------//

class ZExt : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    ZExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~ZExt() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::ZExt>
    clone() const
    {
        return std::static_pointer_cast<SALAM::ZExt>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::ZExt>(new SALAM::ZExt(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createZExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-SExt // ------------------------------------------------------------//

class SExt : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    SExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~SExt() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::SExt>
    clone() const
    {
        return std::static_pointer_cast<SALAM::SExt>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::SExt>(new SALAM::SExt(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-FPToUI // ----------------------------------------------------------//
class FPToUI;
void initializeFPToUIInst(SALAM::FPToUI &salamInstruction);

class FPToUI : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FPToUI(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~FPToUI() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FPToUI>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FPToUI>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FPToUI>(new SALAM::FPToUI(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFPToUIInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-FPToSI // ----------------------------------------------------------//

class FPToSI : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FPToSI(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~FPToSI() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FPToSI>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FPToSI>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FPToSI>(new SALAM::FPToSI(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFPToSIInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-UIToFP // ----------------------------------------------------------//

class UIToFP : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    UIToFP(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~UIToFP() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::UIToFP>
    clone() const
    {
        return std::static_pointer_cast<SALAM::UIToFP>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::UIToFP>(new SALAM::UIToFP(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createUIToFPInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-SIToFP // ----------------------------------------------------------//

class SIToFP : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    SIToFP(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~SIToFP() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::SIToFP>
    clone() const
    {
        return std::static_pointer_cast<SALAM::SIToFP>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::SIToFP>(new SALAM::SIToFP(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSIToFPInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-FPTrunc // ---------------------------------------------------------//

class FPTrunc : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FPTrunc(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
            uint64_t cycles, uint64_t fu);
    ~FPTrunc() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FPTrunc>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FPTrunc>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FPTrunc>(new SALAM::FPTrunc(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFPTruncInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                  uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-FPExt // -----------------------------------------------------------//

class FPExt : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FPExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
          uint64_t cycles, uint64_t fu);
    ~FPExt() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FPExt>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FPExt>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FPExt>(new SALAM::FPExt(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFPExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu);
// SALAM-PtrToInt // --------------------------------------------------------//

class PtrToInt : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    PtrToInt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu);
    ~PtrToInt() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::PtrToInt>
    clone() const
    {
        return std::static_pointer_cast<SALAM::PtrToInt>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::PtrToInt>(new SALAM::PtrToInt(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createPtrToIntInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu);
// SALAM-IntToPtr // --------------------------------------------------------//

class IntToPtr : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    IntToPtr(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu);
    ~IntToPtr() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::IntToPtr>
    clone() const
    {
        return std::static_pointer_cast<SALAM::IntToPtr>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::IntToPtr>(new SALAM::IntToPtr(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createIntToPtrInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu);

class BitCast : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    BitCast(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
            uint64_t cycles, uint64_t fu);
    ~BitCast() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::BitCast>
    clone() const
    {
        return std::static_pointer_cast<SALAM::BitCast>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::BitCast>(new SALAM::BitCast(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createBitCastInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                  uint64_t OpCode, uint64_t cycles, uint64_t fu);

//---------------------------------------------------------------------------//
//--------- Other / Comparison Instructions ---------------------------------//
//---------------------------------------------------------------------------//

// SALAM-ICmp // ------------------------------------------------------------//

class ICmp : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    uint64_t predicate;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    ICmp(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~ICmp() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::ICmp>
    clone() const
    {
        return std::static_pointer_cast<SALAM::ICmp>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::ICmp>(new SALAM::ICmp(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createICmpInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-FCmp // ------------------------------------------------------------//

class FCmp : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    uint64_t predicate;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    FCmp(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~FCmp() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::FCmp>
    clone() const
    {
        return std::static_pointer_cast<SALAM::FCmp>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::FCmp>(new SALAM::FCmp(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createFCmpInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- Other / Edge Instructions ---------------------------------------//
//---------------------------------------------------------------------------//

// SALAM-Phi // -------------------------------------------------------------//

typedef std::pair<std::shared_ptr<SALAM::BasicBlock>,
                  std::shared_ptr<SALAM::Value>>
    phiArgTy;

typedef std::map<std::shared_ptr<SALAM::BasicBlock>,
                 std::shared_ptr<SALAM::Value>>
    phiArgsTy;

class Phi : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    std::shared_ptr<SALAM::BasicBlock> previousBB;
    phiArgsTy phiArgs; // [BasicBlock, Value]
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    Phi(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
        uint64_t cycles, uint64_t fu);
    ~Phi() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    virtual std::vector<uint64_t> runtimeInitialize() override;
    bool
    isPhi() override
    {
        return true;
    }
    void setPrevBB(std::shared_ptr<SALAM::BasicBlock> prevBB);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    virtual valueListTy getStaticDependencies() const override;
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Phi>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Phi>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Phi>(new SALAM::Phi(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createPHIInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu);
// SALAM-Call // ------------------------------------------------------------//

class Call : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;
    std::shared_ptr<SALAM::Value> callee;

  protected:
  public:
    Call(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu);
    ~Call() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    bool
    isCall() override
    {
        return true;
    }
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Value>
    getCalleeValue()
    {
        return callee;
    }
    std::shared_ptr<SALAM::Call>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Call>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Call>(new SALAM::Call(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createCallInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu);
// SALAM-Select // ----------------------------------------------------------//

class Select : public Instruction
{
  private:
    std::vector<std::vector<uint64_t>> conditions;
    std::shared_ptr<SALAM::Value> condition;
    std::shared_ptr<SALAM::Value> trueValue;
    std::shared_ptr<SALAM::Value> falseValue;
    SALAM::Debugger *dbgr;
    uint64_t currentCycle;

  protected:
  public:
    // ---- Constructor
    Select(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu);
    ~Select() = default;
    void initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList);
    uint64_t
    getCycleCount()
    {
        return conditions.at(0).at(2);
    }
    void compute();
    void
    dump()
    {
        if (dbgr->enabled()) {
            dumper();
            inst_dbg->dumper(static_cast<SALAM::Instruction *>(this));
        }
    }
    void dumper();
    std::shared_ptr<SALAM::Select>
    clone() const
    {
        return std::static_pointer_cast<SALAM::Select>(createClone());
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const override
    {
        return std::shared_ptr<SALAM::Select>(new SALAM::Select(*this));
    }
};

std::shared_ptr<SALAM::Instruction>
createSelectInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu);
//---------------------------------------------------------------------------//
//--------- End Instruction Classes -----------------------------------------//
//---------------------------------------------------------------------------//

} // namespace SALAM

#endif // __SALAM_LLVM_INSTRUCTION_HH__
