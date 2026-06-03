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

#include "operand.hh"
#include "sim/sim_object.hh"

SALAM::Constant::Constant(uint64_t id, gem5::SimObject *owner, bool dbg)
    : Value(id, owner, dbg)
{}

void
SALAM::Constant::initialize(llvm::Value *irval, SALAM::irvmap *irmap,
                            SALAM::valueListTy *values)
{
    // Initialize SALAM::Value
    SALAM::Value::initialize(irval, irmap);
    // Parse the constant value
    llvm::ConstantData *cd = llvm::dyn_cast<llvm::ConstantData>(irval);
    llvm::ConstantExpr *ce = llvm::dyn_cast<llvm::ConstantExpr>(irval);
    llvm::Type *irtype = irval->getType();
    if (cd) {
        // The constant is a llvm::ConstantData.
        // Get it's value and store it in constValue
        if (irtype->isFloatingPointTy()) {
            llvm::ConstantFP *fp = llvm::dyn_cast<llvm::ConstantFP>(cd);
            auto apfp = fp->getValueAPF();
            auto api = apfp.bitcastToAPInt();
            addAPFloatRegister(api.getLimitedValue());
        } else if (irtype->isIntegerTy()) {
            llvm::ConstantInt *in = llvm::dyn_cast<llvm::ConstantInt>(cd);
            auto api = in->getValue();
            addAPIntRegister(api.getLimitedValue());
        } else if (irtype->isPointerTy()) {
            assert(llvm::dyn_cast<llvm::ConstantPointerNull>(cd));
            addPointerRegister(false, true);
        }
    } else if (ce) {
        // The constant is an expression. We need to parse the expression
        for (auto op : ce->operand_values()) {
            // Iterate over operands and add
            // new values to our map and value list
            auto mapit = irmap->find(op);
            if (mapit == irmap->end()) {
                uint64_t id = values->back()->getUID() + 1;
                std::shared_ptr<SALAM::Constant> con =
                    std::make_shared<SALAM::Constant>(id, owner, dbg);
                values->push_back(con);
                irmap->insert(SALAM::irvmaptype(op, con));
                operands.push_back(con);
                con->initialize(op, irmap, values);
            } else {
                std::shared_ptr<SALAM::Value> opval = mapit->second;
                operands.push_back(opval);
            }
        }

        switch (ce->getOpcode()) {
            case llvm::Instruction::Trunc: {
                auto opdata = operands.front()->getIntRegValue();
                addAPIntRegister(opdata);
                break;
            }
            case llvm::Instruction::ZExt: {
                auto opdata = operands.front()->getIntRegValue();
                addAPIntRegister(opdata);
                break;
            }
            case llvm::Instruction::SExt: {
                int64_t tmp = operands.front()->getSIntRegValue();
                addAPIntRegister((uint64_t)tmp);
                break;
            }
            case llvm::Instruction::FPToUI: {
                if (operands.front()->getSize() == 32) {
                    auto opdata = operands.front()->getFloatFromReg();
                    addAPIntRegister((uint64_t)opdata);
                } else {
                    auto opdata = operands.front()->getDoubleFromReg();
                    addAPIntRegister((uint64_t)opdata);
                }
                break;
            }
            case llvm::Instruction::FPToSI: {
                if (operands.front()->getSize() == 32) {
                    auto opdata = operands.front()->getFloatFromReg();
                    addAPIntRegister((uint64_t)(int64_t)opdata);
                } else {
                    auto opdata = operands.front()->getDoubleFromReg();
                    addAPIntRegister((uint64_t)(int64_t)opdata);
                }
                break;
            }
            case llvm::Instruction::UIToFP: {
                auto opdata = operands.front()->getUIntRegValue();
                switch (size) {
                    case 32: {
                        float tmp = (float)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    case 64: {
                        double tmp = (double)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    default: {
                        assert(0 &&
                               "Must use AP values for nonstandard FP sizes.");
                        break;
                    }
                }
                break;
            }
            case llvm::Instruction::SIToFP: {
                auto opdata = operands.front()->getSIntRegValue();
                switch (size) {
                    case 32: {
                        float tmp = (float)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    case 64: {
                        double tmp = (double)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    default: {
                        assert(0 &&
                               "Must use AP values for nonstandard FP sizes.");
                        break;
                    }
                }
                break;
            }
            case llvm::Instruction::FPTrunc: {
                switch (operands.front()->getSize()) {
                    case 64: {
                        double opdata = operands.front()->getDoubleFromReg();
                        float tmp = (float)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    default: {
                        assert(0 &&
                               "Must use AP values for nonstandard FP sizes.");
                    }
                }
                break;
            }
            case llvm::Instruction::FPExt: {
                switch (operands.front()->getSize()) {
                    case 32: {
                        float opdata = operands.front()->getFloatFromReg();
                        double tmp = (double)opdata;
                        addAPFloatRegister(*(uint64_t *)&tmp);
                        break;
                    }
                    default: {
                        assert(0 &&
                               "Must use AP values for nonstandard FP sizes.");
                    }
                }
                break;
            }
            case llvm::Instruction::PtrToInt: {
                auto opdata = operands.front()->getReg()->getPtrData();
                addAPIntRegister(opdata);
                break;
            }
            case llvm::Instruction::IntToPtr: {
                auto opdata = operands.front()->getIntRegValue();
                addPointerRegister(opdata, false, false);
                break;
            }
            default:
                assert(0); // We do not support this nested ConstantExpr
        }
    } else {
        assert(0); // The value is not a supported type of llvm::Constant
    }
}

SALAM::GlobalConstant::GlobalConstant(uint64_t id, gem5::SimObject *owner,
                                      bool dbg)
    : Constant(id, owner, dbg)
{}

void
SALAM::GlobalConstant::initialize(llvm::Value *irval, SALAM::irvmap *irmap,
                                  SALAM::valueListTy *values)
{
    // Parse the initializer of the value
    auto glb = llvm::dyn_cast<llvm::GlobalVariable>(irval);
    assert(glb);
    assert(glb->hasInitializer());
    // glb->getInitializer()->print(llvm::outs());

    // Initialize SALAM::ConstantData
    SALAM::Constant::initialize(glb->getInitializer(), irmap, values);
}

SALAM::Argument::Argument(uint64_t id, gem5::SimObject *owner, bool dbg)
    : Value(id, owner, dbg)
{}

void
SALAM::Argument::initialize(llvm::Value *irval, SALAM::irvmap *irmap)
{
    // Initialize SALAM::Value
    SALAM::Value::initialize(irval, irmap);
    addRegister(irval->getType());
}

SALAM::Operand::Operand_Debugger::Operand_Debugger()
{}

void
SALAM::Operand::Operand_Debugger::dumper(Operand *op)
{}

// copy constructor
SALAM::Operand::Operand(const SALAM::Operand &copy_val)
    : SALAM::Value(copy_val)
{
    lockedValue = copy_val.lockedValue;
    set = copy_val.set;
}

// copy constructor from base
SALAM::Operand::Operand(const SALAM::Value &copy_val) : SALAM::Value(copy_val)
{ // Update here for values in the copied value base class
    initOperandReg();
}

SALAM::Operand::Operand(std::shared_ptr<SALAM::Value> copy_val)
    : SALAM::Value(copy_val)
{
    initOperandReg();
}

// operator equals
SALAM::Operand &
SALAM::Operand::operator=(SALAM::Operand &copy_val)
{
    uid = copy_val.uid;
    returnReg = copy_val.returnReg;
    valueTy = copy_val.valueTy;
    size = copy_val.size;
    lockedValue = copy_val.lockedValue;
    set = copy_val.set;
    return *this;
}

void
SALAM::Operand::initOperandReg()
{
    bool istracked = false;
    if (returnReg->isPtr()) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "Operand Ptr Register Initialized\n");
        }
        lockedValue = std::make_shared<PointerRegister>(istracked);
    } else if (returnReg->isInt()) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "Operand Int Register Initialized\n");
        }
        lockedValue = std::make_shared<APIntRegister>(size, istracked);
    } else if (returnReg->isFP()) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "Operand FP Register Initialized\n");
        }
        lockedValue = std::make_shared<APFloatRegister>(valueTy, istracked);
    } else {
        if (dbg) {
            DPRINTFS(Runtime, owner,
                     "Invalid register type. Dumping Operand details\n");
        }
        dump();
        assert(0); // Type is invalid for a register
    }
}

void
SALAM::Operand::initialize(llvm::Value *irval, SALAM::irvmap *irmap)
{
    SALAM::Value::initialize(irval, irmap);
}

void
SALAM::Operand::updateOperandRegister()
{
    assert(lockedValue);
    if (lockedValue->isPtr()) {
        lockedValue->writePtrData(returnReg->getPtrData(true),
                                  getSizeInBytes());
    } else if (lockedValue->isInt()) {
        lockedValue->writeIntData(returnReg->getIntData(true));
    } else if (lockedValue->isFP()) {
        lockedValue->writeFloatData(returnReg->getFloatData(true));
    }
}
