/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include <cmath>

#include "instruction.hh"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/GetElementPtrTypeIterator.h"
#include "sim/sim_object.hh"

namespace SALAM
{

//--------------------------------------------------------------------------//
//--------- Instruction Base Class -----------------------------------------//
//--------------------------------------------------------------------------//

SALAM::Instruction::Instruction(uint64_t id, gem5::SimObject *owner, bool dbg)
    : Value(id, owner, dbg)
{
    currentCycle = 0;
}

SALAM::Instruction::Instruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                                uint64_t OpCode)
    : Value(id, owner, dbg), llvmOpCode(OpCode)
{
    currentCycle = 0;
}

SALAM::Instruction::Instruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                                uint64_t OpCode, uint64_t cycles)
    : Value(id, owner, dbg), llvmOpCode(OpCode), cycleCount(cycles)
{
    currentCycle = 0;
}

SALAM::Instruction::Instruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                                uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Value(id, owner, dbg),
      llvmOpCode(OpCode),
      cycleCount(cycles),
      functional_unit(fu)
{
    currentCycle = 0;
}

SALAM::Instruction::~Instruction()
{}

SALAM::Instruction::Instruction_Debugger::Instruction_Debugger()
{}

void
SALAM::Instruction::Instruction_Debugger::dumper(Instruction *inst)
{}

void
SALAM::Instruction::initialize(llvm::Value *irval, irvmap *irmap,
                               SALAM::valueListTy *valueList)
{
    if (dbg) {
        DPRINTFS(LLVMParse, owner,
                 "Initialize Value - Instruction::initialize\n");
    }
    SALAM::Value::initialize(irval, irmap);
    // Fetch the operands of the instruction
    llvm::User *iruser = llvm::dyn_cast<llvm::User>(irval);
    llvm::Instruction *inst = llvm::dyn_cast<llvm::Instruction>(irval);
    assert(iruser);
    assert(inst);
    uint64_t phiBB = 0;
    for (auto const op : iruser->operand_values()) {
        auto mapit = irmap->find(op);
        std::shared_ptr<SALAM::Value> opval;
        if (mapit == irmap->end()) {
            if (dbg) {
                DPRINTFS(LLVMParse, owner,
                         "Instantiate Operand as Constant Data/Expression\n");
            }
            uint64_t id = valueList->back()->getUID() + 1;
            std::shared_ptr<SALAM::Constant> con =
                std::make_shared<SALAM::Constant>(id, owner, dbg);
            valueList->push_back(con);
            irmap->insert(SALAM::irvmaptype(op, con));
            con->initialize(op, irmap, valueList);
            opval = con;
        } else {
            if (dbg) {
                DPRINTFS(LLVMParse, owner,
                         "Instantiate Operands on Value List\n");
            }
            opval = mapit->second;
        }
        if (dbg) {
            DPRINTFS(LLVMParse, owner,
                     "Link Operand to Static Operands List\n");
        }
        staticDependencies.push_back(opval);
        if (llvm::isa<llvm::PHINode>(inst)) {
            if (dbg) {
                DPRINTFS(LLVMParse, owner, "Phi Node Initiated\n");
            }
            llvm::PHINode *phi = llvm::dyn_cast<llvm::PHINode>(inst);
            llvm::Value *bb =
                llvm::dyn_cast<llvm::Value>(phi->getIncomingBlock(phiBB));
            mapit = irmap->find(bb);
            opval = mapit->second;
            staticDependencies.push_back(opval);
            ++phiBB;
        } else if (llvm::isa<llvm::CmpInst>(inst)) {
            if (dbg) {
                DPRINTFS(LLVMParse, owner, "Compare Instruction Initiated\n");
            }
        }
    }
}

void
SALAM::Instruction::signalUsers()
{
    uint64_t count = 0;
    for (auto user : dynamicUsers) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "|| User[%i] =============\n", count);
        }
        user->setOperandValue(uid);
        count++;
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==signalUsers==========\n");
    }
}

void
SALAM::Instruction::removeDynamicDependency(uint64_t opuid)
{
    auto end = dynamicDependencies.end();
    auto it = dynamicDependencies.find(opuid);
    if (it != end) {
        dynamicDependencies.erase(it);
    }
}

bool
SALAM::Instruction::ready()
{
    if (dbg) {
        DPRINTFS(Runtime, owner, "|| Remaining Dependencies: %i \n",
                 getDependencyCount());
    }
    if (getDependencyCount() == 0) {
        isready = true;
        if (dbg) {
            DPRINTFS(Runtime, owner, "||==Return: %s\n",
                     isready ? "true" : "false");
        }
        if (dbg) {
            DPRINTFS(Runtime, owner, "||==ready=================\n");
        }
        return true;
    } else {
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==Return: %s\n",
                 isready ? "true" : "false");
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==ready=================\n");
    }
    return false;
}

bool
SALAM::Instruction::launch()
{
    if (hasFunctionalUnit()) {
        if (!hw_interface->availableFunctionalUnit(getFunctionalUnit())) {
            return false;
            std::cout << "Waiting on next available FU\n";
        } else {
        }
    }
    launched = true;
    if (getCycleCount() == 0) { // Instruction ready to be committed
        if (dbg) {
            DPRINTFS(Runtime, owner, "||  0 Cycle Instruction\n");
        }
        compute();
        commit();
    } else {
        currentCycle++;
        compute();
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==Return: %s\n",
                 isCommitted() ? "true" : "false");
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==launch================\n");
    }
    return isCommitted();
}

bool
SALAM::Instruction::commit()
{
    if (dbg) {
        DPRINTFS(Runtime, owner, "||  Current Cycle: %i\n", getCurrentCycle());
    }
    if (getCurrentCycle() == getCycleCount()) { // Inst ready to be committed
        signalUsers();
        committed = true;
        if (dbg) {
            DPRINTFS(Runtime, owner, "||==Return: %s\n",
                     committed ? "true" : "false");
        }
        if (dbg) {
            DPRINTFS(Runtime, owner, "||==commit================\n");
        }
        if (hasFunctionalUnit()) {
            hw_interface->clearFunctionalUnit(getFunctionalUnit());
        }
        return true;
    } else {
        if (dbg) {
            DPRINTFS(Runtime, owner, "||  Remaining Cycles: %i\n",
                     getCycleCount() - getCurrentCycle());
        }
        currentCycle++;
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==Return: %s\n",
                 committed ? "true" : "false");
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==commit================\n");
    }
    return false;
}

void
SALAM::Instruction::setOperandValue(uint64_t opuid)
{
    uint64_t count = 0;
    for (auto it = operands.begin(); it != operands.end(); ++it) {
        auto op = *it;
        if (op.getUID() == opuid) {
            if (dbg) {
                DPRINTFS(Runtime, owner, "|| Storing Value in Op[%i]\n",
                         count++);
            }
            op.updateOperandRegister();
            // break;
        } else {
            count++;
        }
    }
    removeDynamicDependency(opuid);
}

void
SALAM::Instruction::reset()
{
    isready = false;
    launched = false;
    committed = false;
    currentCycle = 0;
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==reset=================\n");
    }
}

void
SALAM::Instruction::linkOperands(const SALAM::Operand &newOp)
{
    SALAM::Operand op_copy = newOp;
    operands.push_back(op_copy);
}

std::vector<uint64_t>
SALAM::Instruction::runtimeInitialize()
{
    assert(getDependencyCount() == 0);
    std::vector<uint64_t> dep_uids;

    for (auto it = staticDependencies.begin(); it != staticDependencies.end();
         ++it) {
        std::shared_ptr<SALAM::Value> static_dependency = *it;
        auto dep_uid = static_dependency->getUID();
        operands.push_back(SALAM::Operand(static_dependency));
        if ((static_dependency->isConstant()) ||
            (static_dependency->isArgument())) {
            operands.back().updateOperandRegister();
        } else {
            dep_uids.push_back(dep_uid);
        }
    }

    return dep_uids;
}

// SALAM-BadInstruction // -------------------------------------------------//

std::shared_ptr<SALAM::Instruction>
createBadInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::BadInstruction>(id, owner, dbg, OpCode,
                                                   cycles, fu);
}

BadInstruction::BadInstruction(uint64_t id, gem5::SimObject *owner, bool dbg,
                               uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
BadInstruction::initialize(llvm::Value *irval, irvmap *irmap,
                           SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

// SALAM-Ret // ------------------------------------------------------------//
void // Debugging Interface
Ret::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createRetInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Ret>(id, owner, dbg, OpCode, cycles, fu);
}

Ret::Ret(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Ret::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
Ret::compute()
{
    // Ret never calls compute. Special handling occurs in the scheduler.
}

// SALAM-Br // --------------------------------------------------------------//
void // Debugging Interface
Br::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createBrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Br>(id, owner, dbg, OpCode, cycles, fu);
}

Br::Br(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
       uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{

    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

std::shared_ptr<SALAM::BasicBlock>
Br::getTarget()
{

    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Launching Branch: %s\n",
                 ir_string);
    }
    if (conditional) {
        if (condition->getUIntRegValue() == 1) {
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner,
                         "|| Condition: TRUE, Fetching target %s\n",
                         trueDestination->getIRStub());
            }
            return trueDestination;
        } else {
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner,
                         "|| Condition: FALSE, Fetching target %s\n",
                         falseDestination->getIRStub());
            }
            return falseDestination;
        }
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Fetching target %s\n",
                 defaultDestination->getIRStub());
    }
    return defaultDestination;
}

void
Br::initialize(llvm::Value *irval, irvmap *irmap,
               SALAM::valueListTy *valueList)
{
    SALAM::Value::initialize(irval, irmap);
    llvm::BranchInst *br = llvm::dyn_cast<llvm::BranchInst>(irval);
    assert(br);
    isConditional(br->isConditional());
    llvm::Value *defaultDestValue = br->getSuccessor(0);
    auto mapit = irmap->find(defaultDestValue);
    if (mapit == irmap->end()) {
        if (dbg) {
            DPRINTFS(Runtime, owner,
                     "ERROR. No default successor for Br in IR map");
        }
        assert(0);
    } else {
        defaultDestination =
            std::dynamic_pointer_cast<SALAM::BasicBlock>(mapit->second);
    }
    if (isConditional()) {
        llvm::Value *condValue = br->getCondition();
        mapit = irmap->find(condValue);
        if (mapit == irmap->end()) {
            if (dbg) {
                DPRINTFS(Runtime, owner,
                         "ERROR. No condition for Br in IR map.");
            }
            assert(0);
        } else {
            condition = mapit->second;
            staticDependencies.push_back(condition);
            trueDestination = defaultDestination;

            llvm::Value *falseDestValue = br->getSuccessor(1);
            mapit = irmap->find(falseDestValue);
            if (mapit == irmap->end()) {
                if (dbg) {
                    DPRINTFS(
                        Runtime, owner,
                        "ERROR. No secondary successor for Br in IR map.");
                }
                assert(0);
            } else {
                falseDestination =
                    std::dynamic_pointer_cast<SALAM::BasicBlock>(
                        mapit->second);
            }
        }
    }
}

void
Br::compute()
{
    // Br does not use compute.
    // Special handling occurs in the scheduler.
}

// SALAM-Switch // ---------------------------------------------------------//
void // Debugging Interface
Switch::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSwitchInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Switch>(id, owner, dbg, OpCode, cycles, fu);
}

Switch::Switch(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{

    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

std::shared_ptr<SALAM::BasicBlock>
Switch::getTarget()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Launching Switch: %s\n",
                 ir_string);
    }
    auto opdata = operands.front().getSIntRegValue();

    for (auto it = cases.begin(); it != cases.end(); ++it) {
        if (it->first->getSIntRegValue() == opdata) {
            return it->second;
        }
    }
    return defaultDestination;
}

void
Switch::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Value::initialize(irval, irmap);

    llvm::User *iruser = llvm::dyn_cast<llvm::User>(irval);
    llvm::Instruction *inst = llvm::dyn_cast<llvm::Instruction>(irval);
    assert(iruser);
    assert(inst);

    SALAM::valueListTy tmpStaticDeps;
    for (auto const op : iruser->operand_values()) {
        auto mapit = irmap->find(op);
        std::shared_ptr<SALAM::Value> opval;
        if (mapit == irmap->end()) {
            // TODO: Handle constant data and constant expressions
            if (dbg) {
                DPRINTFS(LLVMParse, owner,
                         "Instantiate Operand as Constant Data/Expression\n");
            }
            uint64_t id = valueList->back()->getUID() + 1;
            std::shared_ptr<SALAM::Constant> con =
                std::make_shared<SALAM::Constant>(id, owner, dbg);
            valueList->push_back(con);
            irmap->insert(SALAM::irvmaptype(op, con));
            con->initialize(op, irmap, valueList);
            opval = con;
        } else {
            if (dbg) {
                DPRINTFS(LLVMParse, owner,
                         "Instantiate Operands on Value List\n");
            }
            opval = mapit->second;
        }
        if (dbg) {
            DPRINTFS(LLVMParse, owner,
                     "Link Operand to Static Operands List\n");
        }
        tmpStaticDeps.push_back(opval);
    }

    llvm::SwitchInst *switchInst = llvm::dyn_cast<llvm::SwitchInst>(irval);
    assert(switchInst);
    caseArgs newArgs;
    for (int i = 2; i < tmpStaticDeps.size();) {
        newArgs.first = tmpStaticDeps.at(i);
        ++i;
        newArgs.second =
            std::dynamic_pointer_cast<SALAM::BasicBlock>(tmpStaticDeps.at(i));
        ++i;
        this->cases.push_back(newArgs);
    }

    staticDependencies.push_back(tmpStaticDeps.front());
    defaultDestination =
        std::dynamic_pointer_cast<SALAM::BasicBlock>(tmpStaticDeps.at(1));
}

void
Switch::compute()
{
    // Perform computations
    // Store results in temp location
}

// SALAM-Add // ------------------------------------------------------------//
void // Debugging Interface
Add::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createAddInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Add>(id, owner, dbg, OpCode, cycles, fu);
}

Add::Add(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{

    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Add::initialize(llvm::Value *irval, SALAM::irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
Add::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 + op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d + (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-FAdd // -----------------------------------------------------------//
void // Debugging Interface
FAdd::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFAddInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FAdd>(id, owner, dbg, OpCode, cycles, fu);
}

FAdd::FAdd(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{

    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FAdd::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
FAdd::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t bitcastResult;
    switch (size) {
        case 32: {
            float op1 = operands.at(0).getFloatFromReg();
            float op2 = operands.at(1).getFloatFromReg();
            float result = op1 + op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f + (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            uint32_t tmp_val = *(uint32_t *)&result;
            bitcastResult = tmp_val;
            break;
        }
        case 64: {
            double op1 = operands.at(0).getDoubleFromReg();
            double op2 = operands.at(1).getDoubleFromReg();
            double result = op1 + op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f + (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            bitcastResult = *(uint64_t *)&result;
            break;
        }
        default: {
            assert(0 && "Unsupported floating point type." &&
                   "Compile with AP values enabled for extended FP support.");
        }
    }
    setRegisterValue(bitcastResult);
}

// SALAM-Sub // ------------------------------------------------------------//
void // Debugging Interface
Sub::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSubInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Sub>(id, owner, dbg, OpCode, cycles, fu);
}

Sub::Sub(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Sub::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Sub::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 - op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d - (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-FSub // -----------------------------------------------------------//
void // Debugging Interface
FSub::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFSubInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FSub>(id, owner, dbg, OpCode, cycles, fu);
}

FSub::FSub(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FSub::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
FSub::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t bitcastResult;
    switch (size) {
        case 32: {
            float op1 = operands.at(0).getFloatFromReg();
            float op2 = operands.at(1).getFloatFromReg();
            float result = op1 - op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f - (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            uint32_t tmp_val = *(uint32_t *)&result;
            bitcastResult = tmp_val;
            break;
        }
        case 64: {
            double op1 = operands.at(0).getDoubleFromReg();
            double op2 = operands.at(1).getDoubleFromReg();
            double result = op1 - op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f - (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            bitcastResult = *(uint64_t *)&result;
            break;
        }
        default: {
            assert(0 && "Unsupported floating point type." &&
                   "Compile with AP values enabled for extended FP support.");
        }
    }
    setRegisterValue(bitcastResult);
}

// SALAM-Mul // ------------------------------------------------------------//
void // Debugging Interface
Mul::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createMulInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Mul>(id, owner, dbg, OpCode, cycles, fu);
}

Mul::Mul(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Mul::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Mul::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 * op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d * (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-FMul // -----------------------------------------------------------//
void // Debugging Interface
FMul::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFMulInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FMul>(id, owner, dbg, OpCode, cycles, fu);
}

FMul::FMul(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FMul::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
FMul::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t bitcastResult;
    switch (size) {
        case 32: {
            float op1 = operands.at(0).getFloatFromReg();
            float op2 = operands.at(1).getFloatFromReg();
            float result = op1 * op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f * (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            uint32_t tmp_val = *(uint32_t *)&result;
            bitcastResult = tmp_val;
            break;
        }
        case 64: {
            double op1 = operands.at(0).getDoubleFromReg();
            double op2 = operands.at(1).getDoubleFromReg();
            double result = op1 * op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f * (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            bitcastResult = *(uint64_t *)&result;
            break;
        }
        default: {
            assert(0 && "Unsupported floating point type." &&
                   "Compile with AP values enabled for extended FP support.");
        }
    }
    setRegisterValue(bitcastResult);
}

// SALAM-UDiv // -----------------------------------------------------------//
void // Debugging Interface
UDiv::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createUDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::UDiv>(id, owner, dbg, OpCode, cycles, fu);
}

UDiv::UDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
UDiv::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
UDiv::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 / op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d / (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-SDiv // -----------------------------------------------------------//
void // Debugging Interface
SDiv::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::SDiv>(id, owner, dbg, OpCode, cycles, fu);
}

SDiv::SDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
SDiv::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
SDiv::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    int64_t op1 = operands.at(0).getSIntRegValue();
    int64_t op2 = operands.at(1).getSIntRegValue();
    int64_t result = op1 / op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d / (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue((uint64_t)result);
}

// SALAM-FDiv // -----------------------------------------------------------//
void // Debugging Interface
FDiv::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFDivInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FDiv>(id, owner, dbg, OpCode, cycles, fu);
}

FDiv::FDiv(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FDiv::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
FDiv::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t bitcastResult;
    switch (size) {
        case 32: {
            float op1 = operands.at(0).getFloatFromReg();
            float op2 = operands.at(1).getFloatFromReg();
            float result = op1 / op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f / (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            uint32_t tmp_val = *(uint32_t *)&result;
            bitcastResult = tmp_val;
            break;
        }
        case 64: {
            double op1 = operands.at(0).getDoubleFromReg();
            double op2 = operands.at(1).getDoubleFromReg();
            double result = op1 / op2;
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f / (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            bitcastResult = *(uint64_t *)&result;
            break;
        }
        default: {
            assert(0 && "Unsupported floating point type." &&
                   "Compile with AP values enabled for extended FP support.");
        }
    }
    setRegisterValue(bitcastResult);
}

// SALAM-URem // -----------------------------------------------------------//
void // Debugging Interface
URem::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createURemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::URem>(id, owner, dbg, OpCode, cycles, fu);
}

URem::URem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
URem::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
URem::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 % op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d % (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-SRem // -----------------------------------------------------------//
void // Debugging Interface
SRem::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSRemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::SRem>(id, owner, dbg, OpCode, cycles, fu);
}

SRem::SRem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
SRem::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
SRem::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    int64_t op1 = operands.at(0).getSIntRegValue();
    int64_t op2 = operands.at(1).getSIntRegValue();
    int64_t result = op1 % op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d % (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue((uint64_t)result);
}

// SALAM-FRem // -----------------------------------------------------------//
void // Debugging Interface
FRem::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFRemInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FRem>(id, owner, dbg, OpCode, cycles, fu);
}

FRem::FRem(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FRem::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
FRem::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t bitcastResult;
    switch (size) {
        case 32: {
            float op1 = operands.at(0).getFloatFromReg();
            float op2 = operands.at(1).getFloatFromReg();
            float result = std::remainderf(op1, op2);
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f % (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            uint32_t tmp_val = *(uint32_t *)&result;
            bitcastResult = tmp_val;
            break;
        }
        case 64: {
            double op1 = operands.at(0).getDoubleFromReg();
            double op2 = operands.at(1).getDoubleFromReg();
            double result = std::remainder(op1, op2);
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| (%s) %f % (%s) %f\n",
                         operands.at(0).getIRStub(), op1,
                         operands.at(1).getIRStub(), op2);
            }
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %f\n", ir_stub,
                         result);
            }
            bitcastResult = *(uint64_t *)&result;
            break;
        }
        default: {
            assert(0 && "Unsupported floating point type." &&
                   "Compile with AP values enabled for extended FP support.");
        }
    }
    setRegisterValue(bitcastResult);
}

// SALAM-Shl // ------------------------------------------------------------//
void // Debugging Interface
Shl::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createShlInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Shl>(id, owner, dbg, OpCode, cycles, fu);
}

Shl::Shl(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Shl::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Shl::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 << op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d << (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-LShr // -----------------------------------------------------------//
void // Debugging Interface
LShr::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createLShrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::LShr>(id, owner, dbg, OpCode, cycles, fu);
}

LShr::LShr(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
LShr::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
LShr::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 >> op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d >> (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-AShr // -----------------------------------------------------------//
void // Debugging Interface
AShr::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createAShrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::AShr>(id, owner, dbg, OpCode, cycles, fu);
}

AShr::AShr(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
AShr::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
AShr::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    int64_t op1 = operands.at(0).getSIntRegValue();
    int64_t op2 = operands.at(1).getSIntRegValue();
    int64_t result = op1 >> op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d >> (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue((uint64_t)result);
}

// SALAM-And // ------------------------------------------------------------//
void // Debugging Interface
And::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createAndInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::And>(id, owner, dbg, OpCode, cycles, fu);
}

And::And(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
And::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
And::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 & op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d & (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-Or // -------------------------------------------------------------//
void // Debugging Interface
Or::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createOrInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Or>(id, owner, dbg, OpCode, cycles, fu);
}

Or::Or(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
       uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Or::initialize(llvm::Value *irval, irvmap *irmap,
               SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Or::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 | op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d | (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-Xor // ------------------------------------------------------------//
void // Debugging Interface
Xor::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createXorInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Xor>(id, owner, dbg, OpCode, cycles, fu);
}

Xor::Xor(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Xor::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Xor::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t op1 = operands.at(0).getUIntRegValue();
    uint64_t op2 = operands.at(1).getUIntRegValue();
    uint64_t result = op1 ^ op2;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| (%s) %d ^ (%s) %d\n",
                 operands.at(0).getIRStub(), op1, operands.at(1).getIRStub(),
                 op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-Load // -----------------------------------------------------------//
void // Debugging Interface
Load::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createLoadInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Load>(id, owner, dbg, OpCode, cycles, fu);
}

Load::Load(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Load::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
    llvm::LoadInst *inst = llvm::dyn_cast<llvm::LoadInst>(irval);
    this->align = inst->getAlign().value();
    if (staticDependencies.front()->isGlobalConstant()) {
        loadingInternal = true;
    }
}

void
Load::compute()
{
    // Load does not use compute normally.
    // Special handling is used in the scheduler.
    // We instead use compute just for debug printout
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %s\n", ir_stub,
                 registerDataString());
    }
}

void
Load::loadInternal()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Launching %s\n", ir_string);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Loading internal value from %s\n",
                 operands.front().getIRString());
    }
    setRegisterValue(operands.front().getOpRegister());
    commit();
}

MemoryRequest *
Load::createMemoryRequest()
{
    Addr memAddr = (operands.front().getPtrRegValue());
    size_t reqLen = getSizeInBytes();
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Launching %s\n", ir_string);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Addr[%x] Size[%i]\n", memAddr,
                 reqLen);
    }
    return new MemoryRequest(memAddr, reqLen);
}

// SALAM-Store // ----------------------------------------------------------//
void // Debugging Interface
Store::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createStoreInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Store>(id, owner, dbg, OpCode, cycles, fu);
}

Store::Store(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Store::initialize(llvm::Value *irval, irvmap *irmap,
                  SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
    llvm::StoreInst *inst = llvm::dyn_cast<llvm::StoreInst>(irval);
    this->align = inst->getAlign().value();
}

void
Store::compute()
{
    // Store does not use compute. Special handling is used in the scheduler.
}

MemoryRequest *
Store::createMemoryRequest()
{
    Addr memAddr = (operands.at(1).getPtrRegValue());
    size_t reqLen = operands.at(0).getSizeInBytes();

    MemoryRequest *req;

    auto dataRegister = operands.at(0).getOpRegister();
    // Copy data from the register
    if (dataRegister->isPtr()) {
        uint64_t regData = dataRegister->getPtrData();
        req = new MemoryRequest(memAddr, (uint8_t *)&regData, reqLen);
    } else {
        uint64_t regData;
        if (dataRegister->isInt()) {
            regData = dataRegister->getIntData();
        } else {
            regData = dataRegister->getFloatData();
        }
        req = new MemoryRequest(memAddr, (uint8_t *)&regData, reqLen);
        if (dbg) {
            DPRINTFS(RuntimeCompute, owner, "|| Launching %s\n", ir_string);
        }
        if (dbg) {
            DPRINTFS(RuntimeCompute, owner, "|| Addr[%x] Size[%i]\n", memAddr,
                     reqLen);
        }
    }

    return req;
}

// SALAM-GEP // ------------------------------------------------------------//
void // Debugging Interface
GetElementPtr::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createGetElementPtrInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                        uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::GetElementPtr>(id, owner, dbg, OpCode,
                                                  cycles, fu);
}

GetElementPtr::GetElementPtr(uint64_t id, gem5::SimObject *owner, bool dbg,
                             uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
GetElementPtr::initialize(llvm::Value *irval, irvmap *irmap,
                          SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
    llvm::User *iruser = llvm::dyn_cast<llvm::User>(irval);
    assert(iruser);
    llvm::GetElementPtrInst *GEP =
        llvm::dyn_cast<llvm::GetElementPtrInst>(irval);
    assert(GEP);
    resultElementType = GEP->getResultElementType();
    llvm::DataLayout layout(GEP->getModule());

    if (resultElementType->getTypeID() == llvm::Type::PointerTyID) {
        resultElementSize = 64; // We assume a 64-bit memory address space
    } else {
        resultElementSize = resultElementType->getScalarSizeInBits();
    }
    resultElementSizeInBytes = ((resultElementSize - 1) >> 3) + 1;

    llvm::Type *ElemTy = GEP->getSourceElementType();
    auto it = iruser->operand_values().begin();
    it++;
    std::vector<llvm::Value *> indexValues;
    for (; it != iruser->operand_values().end(); it++) {
        indexValues.push_back(*it);
    }
    llvm::ArrayRef<llvm::Value *> Indices =
        llvm::ArrayRef<llvm::Value *>(indexValues);
    llvm::generic_gep_type_iterator<llvm::Value *const *> GTI = gep_type_begin(
                                                              ElemTy, Indices),
                                                          GTE = gep_type_end(
                                                              ElemTy, Indices);
    for (; GTI != GTE; ++GTI) {
        llvm::Value *idx = GTI.getOperand();
        auto SALAMValue = irmap->find(idx)->second;
        // auto valueID = SALAMValue->getUID();
        if (llvm::StructType *STy = GTI.getStructTypeOrNull()) {
            assert(idx->getType()->isIntegerTy(32) && "Illegal struct idx");
            unsigned FieldNo =
                llvm::cast<llvm::ConstantInt>(idx)->getSExtValue();
            const llvm::StructLayout *Layout = layout.getStructLayout(STy);
            offsets.push_back(Layout->getElementOffset(FieldNo));
            offsetOfStruct.push_back(true);

        } else {
            llvm::Type *idxty = GTI.getIndexedType();
            offsets.push_back(1 * layout.getTypeAllocSize(idxty));
            offsetOfStruct.push_back(false);
        }
    }
}

void
GetElementPtr::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    uint64_t ptr = (operands.front().getPtrRegValue());
    int64_t offset = 0;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Index Values\n");
    }
    for (int i = 1; i < operands.size(); i++) {
        auto idx = operands.at(i);
        if (offsetOfStruct.at(i - 1)) {
            offset += offsets.at(i - 1);
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s, struct offset = %d\n",
                         idx.getIRStub(), offsets.at(i - 1));
            }
        } else {
            int64_t arrayIdx = idx.getSIntRegValue();
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner,
                         "|| %s = %d, dimension offset = %d\n",
                         idx.getIRStub(), arrayIdx, offsets.at(i - 1));
            }
            offset += arrayIdx * offsets.at(i - 1);
        }
    }

    uint64_t result = ptr + offset;
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner,
                 "|| Ptr[%x]  Offset[%x] (Flat Idx[%d])\n", ptr, offset,
                 offset / resultElementSizeInBytes);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Result: Addr[%x]\n", result);
    }
    setRegisterValue(result);
}

// SALAM-Trunc // ----------------------------------------------------------//
void // Debugging Interface
Trunc::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createTruncInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Trunc>(id, owner, dbg, OpCode, cycles, fu);
}

Trunc::Trunc(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Trunc::initialize(llvm::Value *irval, irvmap *irmap,
                  SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
Trunc::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    // The trunc is handled automatically when we set the return register
    uint64_t result = operands.at(0).getUIntRegValue();
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-ZExt // -----------------------------------------------------------//
void // Debugging Interface
ZExt::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createZExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::ZExt>(id, owner, dbg, OpCode, cycles, fu);
}

ZExt::ZExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
ZExt::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
ZExt::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    // Unsigned data doesn't need any modification when ZExtending
    uint64_t result = operands.at(0).getUIntRegValue();
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue(result);
}

// SALAM-SExt // -----------------------------------------------------------//
void // Debugging Interface
SExt::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::SExt>(id, owner, dbg, OpCode, cycles, fu);
}

SExt::SExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
SExt::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
SExt::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    int64_t result = operands.at(0).getSIntRegValue();
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, result);
    }
    setRegisterValue((uint64_t)result);
}

// SALAM-FPToUI // ---------------------------------------------------------//
void // Debugging Interface
FPToUI::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFPToUIInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FPToUI>(id, owner, dbg, OpCode, cycles, fu);
}

FPToUI::FPToUI(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FPToUI::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
FPToUI::compute()
{
    switch (operands.front().getSize()) {
        case 32: {
            float opdata = operands.front().getFloatFromReg();
            setRegisterValue((uint64_t)opdata);
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| Result: %u\n",
                         (uint64_t)opdata);
            }
            break;
        }
        case 64: {
            double opdata = operands.front().getDoubleFromReg();
            setRegisterValue((uint64_t)opdata);
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| Result: %u\n",
                         (uint64_t)opdata);
            }
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
            break;
        }
    }
}

// SALAM-FPToSI // ---------------------------------------------------------//
void // Debugging Interface
FPToSI::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFPToSIInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FPToSI>(id, owner, dbg, OpCode, cycles, fu);
}

FPToSI::FPToSI(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FPToSI::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
FPToSI::compute()
{
    switch (operands.front().getSize()) {
        case 32: {
            float opdata = operands.front().getFloatFromReg();
            int64_t tmp = (int64_t)opdata; // Truncate to integer
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, tmp);
            }
            setRegisterValue((uint64_t)tmp);
            break;
        }
        case 64: {
            double opdata = operands.front().getDoubleFromReg();
            int64_t tmp = (int64_t)opdata; // Truncate to integer
            if (dbg) {
                DPRINTFS(RuntimeCompute, owner, "|| %s = %d\n", ir_stub, tmp);
            }
            setRegisterValue((uint64_t)tmp);
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
            break;
        }
    }
}

// SALAM-UIToFP // ---------------------------------------------------------//
void // Debugging Interface
UIToFP::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createUIToFPInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::UIToFP>(id, owner, dbg, OpCode, cycles, fu);
}

UIToFP::UIToFP(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
UIToFP::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
UIToFP::compute()
{
    auto opdata = operands.front().getUIntRegValue();
    switch (size) {
        case 32: {
            float tmp = (float)opdata; // Cast to float
            uint32_t tmp_val = *(uint32_t *)&tmp;
            setRegisterValue((uint64_t)tmp_val);
            break;
        }
        case 64: {
            double tmp = (double)opdata;         // Cast to double
            setRegisterValue(*(uint64_t *)&tmp); // Bitcast for writeback
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
            break;
        }
    }
}

// SALAM-SIToFP // ---------------------------------------------------------//
void // Debugging Interface
SIToFP::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSIToFPInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::SIToFP>(id, owner, dbg, OpCode, cycles, fu);
}

SIToFP::SIToFP(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
SIToFP::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
SIToFP::compute()
{
    auto opdata = operands.front().getSIntRegValue();
    switch (size) {
        case 32: {
            float tmp = (float)opdata; // Cast to float
            uint32_t tmp_val = *(uint32_t *)&tmp;
            setRegisterValue((uint64_t)tmp_val);
            break;
        }
        case 64: {
            double tmp = (double)opdata;         // Cast to double
            setRegisterValue(*(uint64_t *)&tmp); // Bitcast for writeback
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
            break;
        }
    }
}

// SALAM-FPTrunc // --------------------------------------------------------//
void // Debugging Interface
FPTrunc::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFPTruncInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                  uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FPTrunc>(id, owner, dbg, OpCode, cycles,
                                            fu);
}

FPTrunc::FPTrunc(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FPTrunc::initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
FPTrunc::compute()
{
    switch (operands.front().getSize()) {
        case 64: {
            double opdata = operands.front().getDoubleFromReg();
            float tmp = (float)opdata; // Cast to float
            uint32_t tmp_val = *(uint32_t *)&tmp;
            setRegisterValue((uint64_t)tmp_val);
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
        }
    }
}

// SALAM-FPExt // ----------------------------------------------------------//
void // Debugging Interface
FPExt::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFPExtInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
                uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FPExt>(id, owner, dbg, OpCode, cycles, fu);
}

FPExt::FPExt(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
             uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FPExt::initialize(llvm::Value *irval, irvmap *irmap,
                  SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
FPExt::compute()
{
    switch (operands.front().getSize()) {
        case 32: {
            float opdata = operands.front().getFloatFromReg();
            double tmp = (double)opdata;         // Cast to double
            setRegisterValue(*(uint64_t *)&tmp); // Bitcast for writeback
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
        }
    }
}

// SALAM-PtrToInt // -------------------------------------------------------//
void // Debugging Interface
PtrToInt::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createPtrToIntInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::PtrToInt>(id, owner, dbg, OpCode, cycles,
                                             fu);
}

PtrToInt::PtrToInt(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
PtrToInt::initialize(llvm::Value *irval, irvmap *irmap,
                     SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
PtrToInt::compute()
{
    auto opdata = operands.front().getPtrRegValue();
    setRegisterValue(opdata);
}

// SALAM-IntToPtr // -------------------------------------------------------//
void // Debugging Interface
IntToPtr::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createIntToPtrInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::IntToPtr>(id, owner, dbg, OpCode, cycles,
                                             fu);
}

IntToPtr::IntToPtr(uint64_t id, gem5::SimObject *owner, bool dbg,
                   uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
IntToPtr::initialize(llvm::Value *irval, irvmap *irmap,
                     SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    // ****** //
}

void
IntToPtr::compute()
{
    auto opdata = operands.front().getUIntRegValue();
    setRegisterValue(opdata);
}

// SALAM-BitCast // --------------------------------------------------------//
void // Debugging Interface
BitCast::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createBitCastInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                  uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::BitCast>(id, owner, dbg, OpCode, cycles,
                                            fu);
}

BitCast::BitCast(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
BitCast::initialize(llvm::Value *irval, irvmap *irmap,
                    SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
}

void
BitCast::compute()
{
    auto opdata = operands.front().getPtrRegValue();
    setRegisterValue(opdata);
}

// SALAM-ICmp // -----------------------------------------------------------//
void // Debugging Interface
ICmp::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createICmpInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::ICmp>(id, owner, dbg, OpCode, cycles, fu);
}

ICmp::ICmp(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
ICmp::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    llvm::CmpInst *inst = llvm::dyn_cast<llvm::CmpInst>(irval);
    this->predicate = inst->getPredicate();
    if (dbg) {
        DPRINTFS(SALAM_Debug, owner,
                 "Integer Comparison Predicate [%i | %s]\n", this->predicate,
                 inst->getPredicateName(inst->getPredicate()).str());
    }
}

void
ICmp::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    bool result = false;
    if (operands.at(0).hasIntVal() && operands.at(1).hasIntVal()) {
        uint64_t uOp1 = operands.at(0).getUIntRegValue();
        uint64_t uOp2 = operands.at(1).getUIntRegValue();
        int64_t sOp1 = operands.at(0).getSIntRegValue();
        int64_t sOp2 = operands.at(1).getSIntRegValue();

        switch (predicate) {
            case SALAM::Predicate::ICMP_EQ: {
                result = (uOp1 == uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_NE: {
                result = (uOp1 != uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_UGT: {
                result = (uOp1 > uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_UGE: {
                result = (uOp1 >= uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_ULT: {
                result = (uOp1 < uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_ULE: {
                result = (uOp1 <= uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_SGT: {
                result = (sOp1 > sOp2);
                break;
            }
            case SALAM::Predicate::ICMP_SGE: {
                result = (sOp1 >= sOp2);
                break;
            }
            case SALAM::Predicate::ICMP_SLT: {
                result = (sOp1 < sOp2);
                break;
            }
            case SALAM::Predicate::ICMP_SLE: {
                result = (sOp1 <= sOp2);
                break;
            }
            default:
                break;
        }
    } else if (operands.at(0).hasPtrVal() && operands.at(1).hasPtrVal()) {
        uint64_t uOp1 = operands.at(0).getPtrRegValue();
        uint64_t uOp2 = operands.at(1).getPtrRegValue();
        switch (predicate) {
            case SALAM::Predicate::ICMP_EQ: {
                result = (uOp1 == uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_NE: {
                result = (uOp1 != uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_UGT: {
                result = (uOp1 > uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_UGE: {
                result = (uOp1 >= uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_ULT: {
                result = (uOp1 < uOp2);
                break;
            }
            case SALAM::Predicate::ICMP_ULE: {
                result = (uOp1 <= uOp2);
                break;
            }
            default:
                break;
        }
    } else {
        panic("Got either wrong or differing datatypes for ICMP");
    }
    setRegisterValue(result);
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %s\n", ir_stub,
                 result ? "TRUE" : "FALSE");
    }
}

// SALAM-FCmp // -----------------------------------------------------------//
void // Debugging Interface
FCmp::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createFCmpInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::FCmp>(id, owner, dbg, OpCode, cycles, fu);
}

FCmp::FCmp(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
FCmp::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    llvm::CmpInst *inst = llvm::dyn_cast<llvm::CmpInst>(irval);
    this->predicate = inst->getPredicate();
    if (dbg) {
        DPRINTFS(SALAM_Debug, owner,
                 "Floating-Point Comparison Predicate [%i | %s]\n",
                 this->predicate,
                 inst->getPredicateName(inst->getPredicate()).str());
    }
}

void
FCmp::compute()
{
    // Perform computations
    // Store results in temp location
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    bool result = false;
    double op1, op2;
    switch (operands.front().getSize()) {
        case 32: {
            op1 = (double)operands.at(0).getFloatFromReg();
            op2 = (double)operands.at(1).getFloatFromReg();
            break;
        }
        case 64: {
            op1 = operands.at(0).getDoubleFromReg();
            op2 = operands.at(1).getDoubleFromReg();
            break;
        }
        default: {
            assert(0 && "Must use AP values for nonstandard FP sizes.");
            break;
        }
    }
    bool unordered = (std::isnan(op1) || std::isnan(op2));
    switch (predicate) {
        case SALAM::Predicate::FCMP_FALSE: {
            result = false;
            break;
        }
        case SALAM::Predicate::FCMP_OEQ: {
            result = (!unordered && (op1 == op2));
            break;
        }
        case SALAM::Predicate::FCMP_OGT: {
            result = (!unordered && (op1 > op2));
            break;
        }
        case SALAM::Predicate::FCMP_OGE: {
            result = (!unordered && (op1 >= op2));
            break;
        }
        case SALAM::Predicate::FCMP_OLT: {
            result = (!unordered && (op1 < op2));
            break;
        }
        case SALAM::Predicate::FCMP_OLE: {
            result = (!unordered && (op1 <= op2));
            break;
        }
        case SALAM::Predicate::FCMP_ONE: {
            result = (!unordered && (op1 != op2));
            break;
        }
        case SALAM::Predicate::FCMP_ORD: {
            result = (!unordered);
            break;
        }
        case SALAM::Predicate::FCMP_UNO: {
            result = (unordered);
            break;
        }
        case SALAM::Predicate::FCMP_UEQ: {
            result = (unordered || (op1 == op2));
            break;
        }
        case SALAM::Predicate::FCMP_UGT: {
            result = (unordered || (op1 > op2));
            break;
        }
        case SALAM::Predicate::FCMP_UGE: {
            result = (unordered || (op1 >= op2));
            break;
        }
        case SALAM::Predicate::FCMP_ULT: {
            result = (unordered || (op1 < op2));
            break;
        }
        case SALAM::Predicate::FCMP_ULE: {
            result = (unordered || (op1 <= op2));
            break;
        }
        case SALAM::Predicate::FCMP_UNE: {
            result = (unordered || (op1 != op2));
            break;
        }
        case SALAM::Predicate::FCMP_TRUE: {
            result = true;
            break;
        }
    }
    setRegisterValue(result);
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Comparing %f, %f\n", op1, op2);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %s\n", ir_stub,
                 result ? "TRUE" : "FALSE");
    }
}

// SALAM-Phi // ------------------------------------------------------------//
void // Debugging Interface
Phi::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createPHIInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
              uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Phi>(id, owner, dbg, OpCode, cycles, fu);
}

Phi::Phi(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
         uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Phi::initialize(llvm::Value *irval, irvmap *irmap,
                SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    llvm::PHINode *phi = llvm::dyn_cast<llvm::PHINode>(irval);
    assert(phi);
    phiArgTy args;
    for (int i = 0; i < Instruction::getStaticDependencies().size();) {
        args.second = Instruction::getStaticDependencies(i);
        ++i;
        args.first = std::dynamic_pointer_cast<SALAM::BasicBlock>(
            Instruction::getStaticDependencies(i));
        ++i;
        this->phiArgs.insert(args);
    }
}

std::vector<uint64_t>
Phi::runtimeInitialize()
{
    assert(getDependencyCount() == 0);
    std::vector<uint64_t> dep_uids;
    std::shared_ptr<SALAM::Value> static_dependency;

    auto it = phiArgs.find(previousBB);
    if (it != phiArgs.end()) {
        static_dependency = it->second;
    } else {
        assert(0 && "Previous BasicBlock not found in PHI args");
    }

    auto dep_uid = static_dependency->getUID();
    operands.push_back(SALAM::Operand(static_dependency));
    if ((static_dependency->isConstant()) ||
        (static_dependency->isArgument())) {
        operands.back().updateOperandRegister();
    } else {
        dep_uids.push_back(dep_uid);
    }

    return dep_uids;
}

void
Phi::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner,
                 "|| PHI entered from %s, using value: %s\n",
                 previousBB->getIRStub(), operands.front().getIRString());
    }
    setRegisterValue(operands.front().getOpRegister());
}

void
Phi::setPrevBB(std::shared_ptr<SALAM::BasicBlock> prevBB)
{
    auto it = phiArgs.find(prevBB);
    if (it != phiArgs.end()) {
        previousBB = prevBB;
    } else {
        assert(0 && "Previous BasicBlock not found in PHI args");
    }
}

valueListTy
Phi::getStaticDependencies() const
{
    valueListTy deps;

    auto it = phiArgs.find(previousBB);
    if (it != phiArgs.end()) {
        deps.push_back(it->second);
    } else {
        assert(0 && "Previous BasicBlock not found in PHI args");
    }

    return deps;
}

// SALAM-Call // -----------------------------------------------------------//
void // Debugging Interface
Call::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createCallInst(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Call>(id, owner, dbg, OpCode, cycles, fu);
}

Call::Call(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
           uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Call::initialize(llvm::Value *irval, irvmap *irmap,
                 SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    callee = staticDependencies.back();
    staticDependencies.pop_back();
}

void
Call::compute()
{
    // Call does not use compute. Special handling is used in the scheduler.
}

// SALAM-Select // ---------------------------------------------------------//
void // Debugging Interface
Select::dumper()
{}

std::shared_ptr<SALAM::Instruction>
createSelectInst(uint64_t id, gem5::SimObject *owner, bool dbg,
                 uint64_t OpCode, uint64_t cycles, uint64_t fu)
{
    return std::make_shared<SALAM::Select>(id, owner, dbg, OpCode, cycles, fu);
}

Select::Select(uint64_t id, gem5::SimObject *owner, bool dbg, uint64_t OpCode,
               uint64_t cycles, uint64_t fu)
    : Instruction(id, owner, dbg, OpCode, cycles, fu)
{
    std::vector<uint64_t> base_params;
    base_params.push_back(id);
    base_params.push_back(OpCode);
    base_params.push_back(cycles);
    conditions.push_back(base_params);
}

void
Select::initialize(llvm::Value *irval, irvmap *irmap,
                   SALAM::valueListTy *valueList)
{
    SALAM::Instruction::initialize(irval, irmap, valueList);
    this->condition = getStaticDependencies(0);
    this->trueValue = getStaticDependencies(1);
    this->falseValue = getStaticDependencies(2);
}

void
Select::compute()
{
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Computing %s\n", ir_string);
    }

    auto cond = operands.at(0);
    auto trueVal = operands.at(1);
    auto falseVal = operands.at(2);

    auto resultReg = (cond.getUIntRegValue() == 1) ? trueVal.getOpRegister()
                                                   : falseVal.getOpRegister();
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| Selecting %s condition\n",
                 (cond.getUIntRegValue() == 1) ? "TRUE" : "FALSE");
    }
    if (dbg) {
        DPRINTFS(RuntimeCompute, owner, "|| %s = %s\n", ir_stub,
                 (cond.getUIntRegValue() == 1) ? trueVal.getIRStub()
                                               : falseVal.getIRStub());
    }
    setRegisterValue(resultReg);
}

} // namespace SALAM

//--------------------------------------------------------------------------//
