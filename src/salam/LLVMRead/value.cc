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

#include "value.hh"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/raw_ostream.h"
#include "sim/sim_object.hh"

SALAM::Value::Value(uint64_t id, gem5::SimObject *_owner, bool _dbg)
{
    uid = id;
    size = 0;
    owner = _owner;
    dbg = _dbg;
}

SALAM::Value::~Value()
{}

// copy constructor
SALAM::Value::Value(const Value &copy_val)
{
    uid = copy_val.uid;
    returnReg = copy_val.returnReg;
    valueTy = copy_val.valueTy;
    size = copy_val.size;
    ir_string = copy_val.ir_string;
    ir_stub = copy_val.ir_stub;
    owner = copy_val.owner;
    dbg = copy_val.dbg;
}

SALAM::Value::Value(std::shared_ptr<SALAM::Value> copy_val)
{
    uid = copy_val->getUID();
    returnReg = copy_val->getReg();
    valueTy = copy_val->getType();
    size = copy_val->getSize();
    ir_string = copy_val->getIRString();
    ir_stub = copy_val->getIRStub();
    owner = copy_val->getOwner();
    dbg = copy_val->debug();
}

// operator equals
SALAM::Value &
SALAM::Value::operator=(Value &copy_val)
{
    uid = copy_val.uid;
    returnReg = copy_val.returnReg;
    valueTy = copy_val.valueTy;
    size = copy_val.size;
    ir_string = copy_val.ir_string;
    ir_stub = copy_val.ir_stub;
    return *this;
}

SALAM::Value::Value_Debugger::Value_Debugger()
{}

void
SALAM::Value::Value_Debugger::dumper(SALAM::Value *value)
{}

void
SALAM::Value::initialize(llvm::Value *irval, SALAM::irvmap *irmap)
{
    llvm::Type *irtype = irval->getType();
    if (irtype->getTypeID() == llvm::Type::PointerTyID) {
        size = 64; // We assume a 64-bit memory address space
    } else {
        size = irtype->getScalarSizeInBits();
    }
    valueTy = irtype->getTypeID();
    // Link Return Register
    if (size > 0) {
        addRegister(irtype, true);
    }

    std::string tmpStr1;
    llvm::raw_string_ostream ss(tmpStr1);
    ss << *irval;
    ir_string = ss.str();

    std::string tmpStr2;
    llvm::raw_string_ostream ss2(tmpStr2);
    irval->printAsOperand(ss2);
    ir_stub = ss2.str();
}

void
SALAM::Value::addRegister(llvm::Type *irtype, bool istracked)
{
    if (irtype->isPointerTy()) {
        returnReg = std::make_shared<PointerRegister>(istracked);
    } else if (irtype->isIntegerTy()) {
        returnReg = std::make_shared<APIntRegister>(irtype, istracked);
    } else if (irtype->isFloatingPointTy()) {
        returnReg = std::make_shared<APFloatRegister>(irtype, istracked);
    } else {
        returnReg = nullptr;
    }
}

void
SALAM::Value::addAPIntRegister(const uint64_t &val)
{

    assert(valueTy == llvm::Type::IntegerTyID);
    uint64_t bitmask = 0;
    assert((size <= 64) && "Only 64-bit and smaller values are \
             supported when not using AP values.");
    bitmask = (bitmask - 1) >> (64 - size);
    returnReg = std::make_shared<APIntRegister>(val & bitmask);
}
void
SALAM::Value::addAPFloatRegister(const uint64_t &val)
{

    assert((valueTy == llvm::Type::FloatTyID) ||
           valueTy == llvm::Type::DoubleTyID);
    uint64_t bitmask = 0;
    assert((size <= 64) && "Only 64-bit and smaller values are \
            supported when not using AP values.");
    bitmask = (bitmask - 1) >> (64 - size);
    returnReg = std::make_shared<APFloatRegister>(val & bitmask);
}

void
SALAM::Value::addPointerRegister(bool istracked, bool isnull)
{
    assert(valueTy == llvm::Type::PointerTyID);
    returnReg = std::make_shared<PointerRegister>(istracked, isnull);
}
void
SALAM::Value::addPointerRegister(uint64_t val, bool istracked, bool isnull)
{
    assert(valueTy == llvm::Type::PointerTyID);
    returnReg = std::make_shared<PointerRegister>(val, istracked, isnull);
}

void
SALAM::Value::setRegisterValue(const uint64_t data)
{
    if (returnReg->isPtr()) {
        if (dbg) {
            DPRINTFS(Runtime, owner, "| Ptr Register\n");
        }
        returnReg->writePtrData(data);
    } else {
        if (returnReg->isInt()) {
            if (dbg) {
                DPRINTFS(Runtime, owner, "| Int Register\n");
            }
            returnReg->writeIntData(data, getSizeInBytes());
        } else {
            if (dbg) {
                DPRINTFS(Runtime, owner, "| FP Register\n");
            }
            returnReg->writeFloatData(data, getSizeInBytes());
        }
    }
}
void
SALAM::Value::setRegisterValue(uint8_t *data)
{
    if (dbg) {
        DPRINTFS(Runtime, owner, "| Set Register Data - ");
    }
    switch (valueTy) {
        case llvm::Type::FloatTyID: {
            if (dbg) {
                DPRINTFS(Runtime, owner, "Float\n");
            }
            returnReg->writeFloatData(*(uint64_t *)data, (size_t)4);
            break;
        }
        case llvm::Type::DoubleTyID: {
            if (dbg) {
                DPRINTFS(Runtime, owner, "Double\n");
            }
            returnReg->writeFloatData(*(uint64_t *)data, (size_t)8);
            break;
        }
        case llvm::Type::IntegerTyID: {
            if (dbg) {
                DPRINTFS(Runtime, owner, "Integer Type | Size = %d\n", size);
            }
            returnReg->writeIntData(*(uint64_t *)data,
                                    (size_t)getSizeInBytes());
            break;
        }
        case llvm::Type::PointerTyID: {
            if (dbg) {
                DPRINTFS(Runtime, owner, "Pointer\n");
            }
            returnReg->writePtrData(*(uint64_t *)data);
            break;
        }
        default: {
            if (dbg) {
                DPRINTFS(Runtime, owner, "Unsupported type for register op\n");
            }
            assert(0);
        }
    }
}

void
SALAM::Value::setRegisterValue(bool data)
{
    if (dbg) {
        DPRINTFS(Runtime, owner, "| Int Register\n");
    }
    if (returnReg->isInt()) {
        if (data) {
            setRegisterValue((uint64_t)1);
        } else {
            setRegisterValue((uint64_t)0);
        }
    } else {
        if (dbg) {
            DPRINTFS(Runtime, owner, "Unsupported type for register op. \
            Tried to place integer data in non-integer register.\n");
        }
    }
}

void
SALAM::Value::setRegisterValue(std::shared_ptr<SALAM::Register> reg)
{
    if (reg->isPtr()) {
        setRegisterValue((reg->getPtrData()));
    } else if (reg->isFP()) {
        setRegisterValue((reg->getFloatData()));
    } else {
        setRegisterValue((reg->getIntData()));
    }
    if (dbg) {
        DPRINTFS(Runtime, owner, "||==setRegisterValue====\n");
    }
}
