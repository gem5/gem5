/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 */

#ifndef __EXETRACE_HH__
#define __EXETRACE_HH__

#include <fstream>
#include <vector>

#include "sim/host.hh"
#include "cpu/inst_seq.hh"	// for InstSeqNum
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

class BaseCPU;


namespace Trace {

class InstRecord : public Record
{
  protected:

    // The following fields are initialized by the constructor and
    // thus guaranteed to be valid.
    BaseCPU *cpu;
    // need to make this ref-counted so it doesn't go away before we
    // dump the record
    StaticInstPtr<TheISA> staticInst;
    Addr PC;
    bool misspeculating;
    unsigned thread;

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
        DataInt8 = 1,	// set to equal number of bytes
        DataInt16 = 2,
        DataInt32 = 4,
        DataInt64 = 8,
        DataDouble = 3
    } data_status;

    InstSeqNum fetch_seq;
    bool fetch_seq_valid;

    InstSeqNum cp_seq;
    bool cp_seq_valid;

    struct iRegFile {
        IntRegFile regs;
    };
    iRegFile *iregs;
    bool regs_valid;

  public:
    InstRecord(Tick _cycle, BaseCPU *_cpu,
               const StaticInstPtr<TheISA> &_staticInst,
               Addr _pc, bool spec, int _thread)
        : Record(_cycle), cpu(_cpu), staticInst(_staticInst), PC(_pc),
          misspeculating(spec), thread(_thread)
    {
        data_status = DataInvalid;
        addr_valid = false;
        regs_valid = false;

        fetch_seq_valid = false;
        cp_seq_valid = false;
    }

    virtual ~InstRecord() { }

    virtual void dump(std::ostream &outs);

    void setAddr(Addr a) { addr = a; addr_valid = true; }

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

    void setRegs(const IntRegFile &regs);

    void finalize() { theLog.append(this); }

    enum InstExecFlagBits {
        TRACE_MISSPEC = 0,
        PRINT_CYCLE,
        PRINT_OP_CLASS,
        PRINT_THREAD_NUM,
        PRINT_RESULT_DATA,
        PRINT_EFF_ADDR,
        PRINT_INT_REGS,
        PRINT_FETCH_SEQ,
        PRINT_CP_SEQ,
        NUM_BITS
    };

    static std::vector<bool> flags;

    static void setParams();

    static bool traceMisspec() { return flags[TRACE_MISSPEC]; }
};


inline void
InstRecord::setRegs(const IntRegFile &regs)
{
    if (!iregs)
      iregs = new iRegFile;

    memcpy(&iregs->regs, regs, sizeof(IntRegFile));
    regs_valid = true;
}

inline
InstRecord *
getInstRecord(Tick cycle, ExecContext *xc, BaseCPU *cpu,
              const StaticInstPtr<TheISA> staticInst,
              Addr pc, int thread = 0)
{
    if (DTRACE(InstExec) &&
        (InstRecord::traceMisspec() || !xc->misspeculating())) {
        return new InstRecord(cycle, cpu, staticInst, pc,
                              xc->misspeculating(), thread);
    }

    return NULL;
}


}

#endif // __EXETRACE_HH__
