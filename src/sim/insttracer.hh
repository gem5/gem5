/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __INSTRECORD_HH__
#define __INSTRECORD_HH__

#include "base/bigint.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"      // for InstSeqNum
#include "cpu/static_inst.hh"
#include "sim/sim_object.hh"

class ThreadContext;

namespace Trace {

class InstRecord
{
  protected:
    Tick when;

    // The following fields are initialized by the constructor and
    // thus guaranteed to be valid.
    ThreadContext *thread;
    // need to make this ref-counted so it doesn't go away before we
    // dump the record
    StaticInstPtr staticInst;
    TheISA::PCState pc;
    StaticInstPtr macroStaticInst;
    bool misspeculating;
    bool predicate;

    // The remaining fields are only valid for particular instruction
    // types (e.g, addresses for memory ops) or when particular
    // options are enabled (e.g., tracing full register contents).
    // Each data field has an associated valid flag to indicate
    // whether the data field is valid.
    Addr addr;
    bool addr_valid;

    union {
        uint64_t as_int;
        double as_double;
    } data;
    enum {
        DataInvalid = 0,
        DataInt8 = 1,   // set to equal number of bytes
        DataInt16 = 2,
        DataInt32 = 4,
        DataInt64 = 8,
        DataDouble = 3
    } data_status;

    InstSeqNum fetch_seq;
    bool fetch_seq_valid;

    InstSeqNum cp_seq;
    bool cp_seq_valid;

  public:
    InstRecord(Tick _when, ThreadContext *_thread,
               const StaticInstPtr _staticInst,
               TheISA::PCState _pc, bool spec,
               const StaticInstPtr _macroStaticInst = NULL)
        : when(_when), thread(_thread),
          staticInst(_staticInst), pc(_pc),
          macroStaticInst(_macroStaticInst),
          misspeculating(spec), predicate(true)
    {
        data_status = DataInvalid;
        addr_valid = false;

        fetch_seq_valid = false;
        cp_seq_valid = false;
    }

    virtual ~InstRecord() { }

    void setAddr(Addr a) { addr = a; addr_valid = true; }

    void setData(Twin64_t d) { data.as_int = d.a; data_status = DataInt64; }
    void setData(Twin32_t d) { data.as_int = d.a; data_status = DataInt32; }
    void setData(uint64_t d) { data.as_int = d; data_status = DataInt64; }
    void setData(uint32_t d) { data.as_int = d; data_status = DataInt32; }
    void setData(uint16_t d) { data.as_int = d; data_status = DataInt16; }
    void setData(uint8_t d) { data.as_int = d; data_status = DataInt8; }

    void setData(int64_t d) { setData((uint64_t)d); }
    void setData(int32_t d) { setData((uint32_t)d); }
    void setData(int16_t d) { setData((uint16_t)d); }
    void setData(int8_t d)  { setData((uint8_t)d); }

    void setData(double d) { data.as_double = d; data_status = DataDouble; }

    void setFetchSeq(InstSeqNum seq)
    { fetch_seq = seq; fetch_seq_valid = true; }

    void setCPSeq(InstSeqNum seq)
    { cp_seq = seq; cp_seq_valid = true; }

    void setPredicate(bool val) { predicate = val; }

    virtual void dump() = 0;
    
  public:
    Tick getWhen() { return when; }
    ThreadContext *getThread() { return thread; }
    StaticInstPtr getStaticInst() { return staticInst; }
    TheISA::PCState getPCState() { return pc; }
    StaticInstPtr getMacroStaticInst() { return macroStaticInst; }
    bool getMisspeculating() { return misspeculating; }

    Addr getAddr() { return addr; }
    bool getAddrValid() { return addr_valid; }

    uint64_t getIntData() { return data.as_int; }
    double getFloatData() { return data.as_double; }
    int getDataStatus() { return data_status; }

    InstSeqNum getFetchSeq() { return fetch_seq; }
    bool getFetchSeqValid() { return fetch_seq_valid; }

    InstSeqNum getCpSeq() { return cp_seq; }
    bool getCpSeqValid() { return cp_seq_valid; }
};

class InstTracer : public SimObject
{
  public:
    InstTracer(const Params *p) : SimObject(p)
    {}

    virtual ~InstTracer()
    {};

    virtual InstRecord *
        getInstRecord(Tick when, ThreadContext *tc,
                const StaticInstPtr staticInst, TheISA::PCState pc,
                const StaticInstPtr macroStaticInst = NULL) = 0;
};



} // namespace Trace

#endif // __INSTRECORD_HH__
