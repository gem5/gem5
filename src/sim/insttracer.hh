/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
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

#include "base/types.hh"
#include "cpu/inst_seq.hh"
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

    // The remaining fields are only valid for particular instruction
    // types (e.g, addresses for memory ops) or when particular
    // options are enabled (e.g., tracing full register contents).
    // Each data field has an associated valid flag to indicate
    // whether the data field is valid.

    /*** @defgroup mem
     * @{
     * Memory request information in the instruction accessed memory.
     * @see mem_valid
     */
    Addr addr; ///< The address that was accessed
    Addr size; ///< The size of the memory request
    unsigned flags; ///< The flags that were assigned to the request.

    /** @} */

    /** @defgroup data
     * If this instruction wrote any data values they're recorded here
     * WARNING: Instructions are quite loose with with what they write
     * since many instructions write multiple values (e.g. destintation
     * register, flags, status, ...) This only captures the last write.
     * @TODO fix this and record all destintations that an instruction writes
     * @see data_status
     */
    union {
        uint64_t as_int;
        double as_double;
    } data;

    /** @defgroup fetch_seq
     * This records the serial number that the instruction was fetched in.
     * @see fetch_seq_valid
     */
    InstSeqNum fetch_seq;

    /** @defgroup commit_seq
     * This records the instruction number that was committed in the pipeline
     * @see cp_seq_valid
     */
    InstSeqNum cp_seq;

    /** @ingroup data
     * What size of data was written?
     */
    enum DataStatus {
        DataInvalid = 0,
        DataInt8 = 1,   // set to equal number of bytes
        DataInt16 = 2,
        DataInt32 = 4,
        DataInt64 = 8,
        DataDouble = 3
    } data_status;

    /** @ingroup memory
     * Are the memory fields in the record valid?
     */
    bool mem_valid;

    /** @ingroup fetch_seq
     * Are the fetch sequence number fields valid?
     */
    bool fetch_seq_valid;
    /** @ingroup commit_seq
     * Are the commit sequence number fields valid?
     */
    bool cp_seq_valid;

    /** is the predicate for execution this inst true or false (not execed)?
     */
    bool predicate;

  public:
    InstRecord(Tick _when, ThreadContext *_thread,
               const StaticInstPtr _staticInst,
               TheISA::PCState _pc,
               const StaticInstPtr _macroStaticInst = NULL)
        : when(_when), thread(_thread), staticInst(_staticInst), pc(_pc),
        macroStaticInst(_macroStaticInst), addr(0), size(0), flags(0),
        fetch_seq(0), cp_seq(0), data_status(DataInvalid), mem_valid(false),
        fetch_seq_valid(false), cp_seq_valid(false), predicate(true)
    { }

    virtual ~InstRecord() { }

    void setWhen(Tick new_when) { when = new_when; }
    void setMem(Addr a, Addr s, unsigned f)
    {
        addr = a; size = s; flags = f; mem_valid = true;
    }

    template <typename T, size_t N>
    void
    setData(std::array<T, N> d)
    {
        data.as_int = d[0];
        data_status = (DataStatus)sizeof(T);
        static_assert(sizeof(T) == DataInt8 || sizeof(T) == DataInt16 ||
                      sizeof(T) == DataInt32 || sizeof(T) == DataInt64,
                      "Type T has an unrecognized size.");
    }

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
    Tick getWhen() const { return when; }
    ThreadContext *getThread() const { return thread; }
    StaticInstPtr getStaticInst() const { return staticInst; }
    TheISA::PCState getPCState() const { return pc; }
    StaticInstPtr getMacroStaticInst() const { return macroStaticInst; }

    Addr getAddr() const { return addr; }
    Addr getSize() const { return size; }
    unsigned getFlags() const { return flags; }
    bool getMemValid() const { return mem_valid; }

    uint64_t getIntData() const { return data.as_int; }
    double getFloatData() const { return data.as_double; }
    int getDataStatus() const { return data_status; }

    InstSeqNum getFetchSeq() const { return fetch_seq; }
    bool getFetchSeqValid() const { return fetch_seq_valid; }

    InstSeqNum getCpSeq() const { return cp_seq; }
    bool getCpSeqValid() const { return cp_seq_valid; }
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
