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

#ifndef __SALAM_OPERAND_HH__
#define __SALAM_OPERAND_HH__
#include <llvm-c/Core.h>

#include <map>
#include <memory>
#include <vector>

#include "debug_flags.hh"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APSInt.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Value.h"
#include "registers.hh"
#include "value.hh"

namespace SALAM
{

class Operand : public Value
{
  private:
    std::shared_ptr<SALAM::Register> lockedValue;
    bool set = false;

  protected:
    class Operand_Debugger : public Debugger
    {
      public:
        Operand_Debugger();
        ~Operand_Debugger() = default;
        using SALAM::Debugger::dumper;
        virtual void dumper(SALAM::Operand *op);
    };

    Operand_Debugger *op_dbg;

  public:
    void
    dump()
    {
        if (dbg) {
            op_dbg->dumper(this);
        }
    }
    void initOperandReg();
    // Operand(uint64_t id);
    Operand(const SALAM::Value &copy_val);
    Operand(const Operand &copy_val);
    Operand(std::shared_ptr<SALAM::Value> copy_val);
    Operand &operator=(Operand &copy_val);
    ~Operand() = default;
    using SALAM::Value::initialize;
    virtual void initialize(llvm::Value *irval, irvmap *irmap) override;
    void updateOperandRegister();

    virtual uint64_t
    getPtrRegValue()
    {
        return lockedValue->getPtrData();
    }
#if USE_LLVM_AP_VALUES
    virtual llvm::APFloat
    getFloatRegValue()
    {
        return lockedValue->getFloatData();
    }
    virtual llvm::APSInt
    getIntRegValue()
    {
        return lockedValue->getIntData();
    }
    virtual bool
    hasIntVal()
    {
        return lockedValue->isInt();
    }
    virtual bool
    hasPtrVal()
    {
        return lockedValue->isPtr();
    }
#else
    virtual uint64_t
    getFloatRegValue()
    {
        return lockedValue->getFloatData();
    }
    virtual float
    getFloatFromReg()
    {
        return lockedValue->getFloat();
    }
    virtual double
    getDoubleFromReg()
    {
        return lockedValue->getDouble();
    }
    virtual uint64_t
    getIntRegValue()
    {
        return lockedValue->getIntData();
    }
    virtual uint64_t
    getUIntRegValue()
    {
        return lockedValue->getUnsignedInt();
    }
    virtual int64_t
    getSIntRegValue()
    {
        return lockedValue->getSignedInt(size);
    }
    virtual bool
    hasIntVal()
    {
        return lockedValue->isInt();
    }
    virtual bool
    hasPtrVal()
    {
        return lockedValue->isPtr();
    }
#endif
    std::shared_ptr<SALAM::Register>
    getOpRegister()
    {
        return lockedValue;
    }
};

class Constant : public Value
{
  private:
  protected:
    SALAM::valueListTy operands;

  public:
    Constant(uint64_t id, gem5::SimObject *owner, bool dbg);
    ~Constant() = default;
    virtual bool
    isConstant()
    {
        return true;
    }
    using SALAM::Value::initialize;
    virtual void initialize(llvm::Value *irval, irvmap *irmap,
                            SALAM::valueListTy *values);
};

class GlobalConstant : public Constant
{
  private:
  protected:
  public:
    GlobalConstant(uint64_t id, gem5::SimObject *owner, bool dbg);
    ~GlobalConstant() = default;
    virtual bool
    isGlobalConstant()
    {
        return true;
    }
    virtual void initialize(llvm::Value *irval, irvmap *irmap,
                            SALAM::valueListTy *values) override;
};

class Argument : public Value
{
  private:
  protected:
  public:
    Argument(uint64_t id, gem5::SimObject *owner, bool dbg);
    ~Argument() = default;
    virtual bool
    isArgument()
    {
        return true;
    }
    virtual void initialize(llvm::Value *irval, irvmap *irmap) override;
};

} // namespace SALAM

#endif //__SALAM_OPERAND_HH__
