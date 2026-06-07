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

#ifndef __SALAM_REGISTERS_HH__
#define __SALAM_REGISTERS_HH__

#include <llvm-c/Core.h>

#include <cmath>

#include "debug_flags.hh"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APSInt.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Value.h"

namespace SALAM
{
/*****************************************************************************
 * Register is the data storage container for SALAM::Values.
 * Every instruction and function argument has a corresponding register that
 * is tracked for power/area/timing. Additionally Constants have corresponding
 * registers, which are not tracked, since they do not have a timing component.
 *****************************************************************************/
class Register
{
  protected:
    bool tracked;
    bool isNULL = false;
    bool dbg = false;
    uint64_t reads = 0;
    uint64_t writes = 0;
    uint64_t regdata;

    class Register_Debugger : public Debugger
    {
      public:
        Register_Debugger();
        ~Register_Debugger() = default;
        using SALAM::Debugger::dumper;
        virtual void dumper(SALAM::Register *reg);
    };

    Register_Debugger reg_dbg;

  public:
    Register(bool trk = true, bool nul = false);
    ~Register();
    virtual uint64_t
    getFloatData(bool incReads = true)
    {
        assert(0 && "Attempted to read float data from non-float register");
        return 0;
    }
    virtual float
    getFloat(bool incReads = true)
    {
        assert(0 && "Attempted to read float data from non-float register");
        return NAN;
    }
    virtual double
    getDouble(bool incReads = true)
    {
        assert(0 && "Attempted to read float data from non-float register");
        return NAN;
    }
    virtual uint64_t
    getIntData(bool incReads = true)
    {
        assert(0 &&
               "Attempted to read integer data from non-integer register");
        return 0;
    }
    virtual uint64_t
    getUnsignedInt(bool incReads = true)
    {
        assert(0 &&
               "Attempted to read integer data from non-integer register");
        return 0;
    }
    virtual int64_t
    getSignedInt(size_t sizeInBits, bool incReads = true)
    {
        assert(0 &&
               "Attempted to read integer data from non-integer register");
        return 0;
    }
    virtual uint64_t
    getPtrData(bool incReads = true)
    {
        assert(0 &&
               "Attempted to read pointer data from non-pointer register");
        return 0;
    }
    virtual void
    writeFloatData(uint64_t apf, size_t len = 8, bool incWrites = true)
    {
        assert(0 && "Attempted to write float data on non-float register");
    }
    virtual void
    writeIntData(uint64_t api, size_t len = 8, bool incWrites = true)
    {
        assert(0 &&
               "Attempted to write interger data on non-integer register");
    }
    virtual void
    writePtrData(uint64_t ptr, size_t len = 8, bool incWrites = true)
    {
        assert(0 && "Attempted to write pointer data on non-pointer register");
    }
    virtual bool
    isInt()
    {
        return false;
    }
    virtual bool
    isFP()
    {
        return false;
    }
    virtual bool
    isPtr()
    {
        return false;
    }
    bool
    isTracked()
    {
        return tracked;
    }
    bool
    isNull()
    {
        return isNULL;
    }
    void
    setNull(bool flag)
    {
        isNULL = flag;
    }
    void
    setTracked(bool flag)
    {
        tracked = flag;
    }
    void
    dump()
    {
        if (dbg) {
            reg_dbg.dumper(this);
        }
    }
    uint64_t
    getReads()
    {
        return reads;
    }
    uint64_t
    getWrites()
    {
        return writes;
    }
    virtual std::string dataString() = 0;
};

class APFloatRegister : public Register
{
  private:
    // We use uint64_t to store the bitcast of the FP value.
    // Compute should be performed after bitcasting back to
    // appropriate type.
    uint64_t data = 0;

  public:
    APFloatRegister(llvm::Type::TypeID T, bool isTracked);
    APFloatRegister(llvm::Type *T, bool isTracked = true);
    // This constructor is only used for constants.
    APFloatRegister(const llvm::APFloat &RHS);
    // ~APFloatRegister() { if (data) delete data; }
    // This constructor is only used for constants.
    APFloatRegister(const uint64_t RHS) : Register(false)
    {
        data = RHS;
        regdata = data;
    }
    virtual uint64_t getFloatData(bool incReads = true) override;
    virtual float getFloat(bool incReads = true) override;
    virtual double getDouble(bool incReads = true) override;
    virtual void writeFloatData(uint64_t apf, size_t len = 8,
                                bool incWrites = true) override;
    virtual bool
    isFP() override
    {
        return true;
    }
    virtual std::string dataString() override;
};

class APIntRegister : public Register
{
  private:
    uint64_t data = 0;

  public:
    APIntRegister(uint64_t bitwidth, bool isTracked);
    APIntRegister(llvm::Type *T, bool isTracked = true);
    // This constructor is only used for constants.
    APIntRegister(const llvm::APInt &RHS);
    // ~APIntRegister() { if (data) delete data; }
    // This constructor is only used for constants.
    APIntRegister(const uint64_t RHS) : Register(false)
    {
        data = RHS;
        regdata = data;
    }
    virtual uint64_t getIntData(bool incReads = true) override;
    virtual uint64_t getUnsignedInt(bool incReads = true) override;
    virtual int64_t getSignedInt(size_t sizeInBits,
                                 bool incReads = true) override;
    virtual void writeIntData(uint64_t api, size_t len = 8,
                              bool incWrites = true) override;
    virtual bool
    isInt() override
    {
        return true;
    }
    virtual std::string dataString() override;
};

class PointerRegister : public Register
{
  private:
    uint64_t pointer = 0;

  public:
    PointerRegister(bool isTracked = true, bool isNull = false);
    PointerRegister(uint64_t val, bool isTracked = true, bool isNull = false);
    // ~PointerRegister() { if (pointer) delete pointer; }
    virtual bool
    isPtr() override
    {
        return true;
    }
    virtual uint64_t getPtrData(bool incReads = true) override;
    virtual void writePtrData(uint64_t ptr, size_t len = 8,
                              bool incWrites = true) override;
    virtual std::string dataString() override;
};
} // namespace SALAM
#endif
