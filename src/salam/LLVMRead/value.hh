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

#ifndef __SALAM_VALUE_HH__
#define __SALAM_VALUE_HH__

#include <llvm-c/Core.h>

#include <map>
#include <memory>
#include <vector>

#include "debug_flags.hh"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APSInt.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Value.h"
#include "registers.hh"

namespace gem5
{
class SimObject;
}

namespace SALAM
{
class Value;
typedef std::pair<llvm::Value *, std::shared_ptr<Value>> irvmaptype;
typedef std::map<llvm::Value *, std::shared_ptr<Value>> irvmap;
typedef std::vector<std::shared_ptr<Value>> valueListTy;

class Value
{
  public:
  protected:
    uint64_t uid = 0;
    uint64_t size = 0;
    gem5::SimObject *owner;
    std::string ir_string;
    std::string ir_stub;
    llvm::Type::TypeID valueTy;
    std::shared_ptr<SALAM::Register> returnReg;
    bool dbg = false;
    bool inst = false;

    void addRegister(llvm::Type *irtype, bool isTracked = true);
    void addAPIntRegister(const uint64_t &val);
    void addAPFloatRegister(const uint64_t &val);
    void addPointerRegister(bool isTracked = true, bool isNull = false);
    void addPointerRegister(uint64_t val, bool isTracked = true,
                            bool isNull = false);

    class Value_Debugger : public Debugger
    {
      public:
        Value_Debugger();
        ~Value_Debugger() = default;
        using SALAM::Debugger::dumper;
        virtual void dumper(SALAM::Value *value);
    };

    Value_Debugger *value_dbg;
    Value(const SALAM::Value &copy_val);
    Value(std::shared_ptr<SALAM::Value> copy_val);

  public:
    /** Class Constructor.
     * @param id
     */
    Value(uint64_t id, gem5::SimObject *_owner, bool _dbg);
    Value &operator=(Value &copy_val);
    bool
    operator==(const Value &v)
    {
        return uid == v.uid;
    }
    ~Value();
    virtual void initialize(llvm::Value *irval, SALAM::irvmap *irmap);
    uint64_t
    getSize()
    {
        return size;
    }
    uint64_t
    getSizeInBytes()
    {
        if (size == 0) {
            return 0;
        } else {
            return ((size - 1) >> 3) + 1;
        }
    }
    uint64_t
    getUID() const
    {
        return uid;
    }
    std::shared_ptr<SALAM::Register>
    getReg()
    {
        return returnReg;
    }
    llvm::Type::TypeID
    getType()
    {
        return valueTy;
    }
    std::string
    getIRString()
    {
        return ir_string;
    }
    std::string
    getIRStub()
    {
        return ir_stub;
    }
    gem5::SimObject *
    getOwner()
    {
        return owner;
    }
    bool
    debug()
    {
        return dbg;
    }

    // Helper functions for setting the value of the return register
    // directly from the value
    // Using these functions will increment the write counters on
    // tracked registers
    // If you'd like to avoid incrementing write counters, directly pull
    // the register and use its appropriate write function
    void setRegisterValue(const uint64_t data);
    void setRegisterValue(uint8_t *data);
    void setRegisterValue(bool data);
    void setRegisterValue(std::shared_ptr<SALAM::Register> reg);

    // Helper functions for getting the value of the return register
    // directly from the value
    // Using these functions will increment the read counters on
    // tracked registers
    // If you'd like to avoid incrementing read counters, directly pull
    // the register and use its appropriate read function
    virtual uint64_t
    getPtrRegValue()
    {
        return returnReg->getPtrData();
    }
    virtual uint64_t
    getFloatRegValue()
    {
        return returnReg->getFloatData();
    }
    virtual float
    getFloatFromReg()
    {
        return returnReg->getFloat();
    }
    virtual double
    getDoubleFromReg()
    {
        return returnReg->getDouble();
    }
    virtual uint64_t
    getIntRegValue()
    {
        return returnReg->getIntData();
    }
    virtual uint64_t
    getUIntRegValue()
    {
        return returnReg->getUnsignedInt();
    }
    virtual int64_t
    getSIntRegValue()
    {
        return returnReg->getSignedInt(size);
    }

    virtual bool
    isConstant()
    {
        return false;
    }
    virtual bool
    isGlobalConstant()
    {
        return false;
    }
    virtual bool
    isArgument()
    {
        return false;
    }
    virtual bool
    isFunction()
    {
        return false;
    }
    virtual bool
    isBasicBlock()
    {
        return false;
    }
    virtual bool
    isInstruction()
    {
        return false;
    }
    virtual uint64_t
    getOpode()
    {
        return -1;
    }

    void
    value_dump()
    {
        if (dbg) {
            value_dbg->dumper(this);
        }
    }
    std::string
    registerDataString()
    {
        return returnReg->dataString();
    }
    std::shared_ptr<SALAM::Value>
    clone() const
    {
        return createClone();
    }
    virtual std::shared_ptr<SALAM::Value>
    createClone() const
    {
        return std::shared_ptr<SALAM::Value>(new SALAM::Value(*this));
    }
};
} // namespace SALAM

#endif //__SALAM_VALUE_HH__
