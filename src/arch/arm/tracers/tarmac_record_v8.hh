/*
 * Copyright (c) 2017-2018 ARM Limited
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
 * Authors: Giacomo Travaglini
 */

/**
 * @file: The file contains the informations used to generate records
 *        for ARMv8 cores.
 */

#ifndef __ARCH_ARM_TRACERS_TARMAC_RECORD_V8_HH__
#define __ARCH_ARM_TRACERS_TARMAC_RECORD_V8_HH__

#include "tarmac_record.hh"

namespace Trace {

/**
 * TarmacTracer record for ARMv8 CPUs:
 * The record is adding some data to the base TarmacTracer
 * record.
 */
class TarmacTracerRecordV8 : public TarmacTracerRecord
{
  public:

    /**
     * General data shared by all v8 entries
     */
    struct TraceEntryV8
    {
      public:
        TraceEntryV8(std::string _cpuName)
          : cpuName(_cpuName)
        {}

      protected:
        std::string cpuName;
    };

    /**
     * Instruction entry for v8 records
     */
    struct TraceInstEntryV8: public TraceInstEntry, TraceEntryV8
    {
      public:
        TraceInstEntryV8(const TarmacContext& tarmCtx, bool predicate);

        virtual void print(std::ostream& outs,
                           int verbosity = 0,
                           const std::string &prefix = "") const override;

      protected:
        Addr paddr;
        bool paddrValid;
    };

    /**
     * Register entry for v8 records
     */
    struct TraceRegEntryV8: public TraceRegEntry, TraceEntryV8
    {
      public:
        TraceRegEntryV8(const TarmacContext& tarmCtx, const RegId& reg);

        virtual void print(std::ostream& outs,
                           int verbosity = 0,
                           const std::string &prefix = "") const override;

      protected:
        void updateInt(const TarmacContext& tarmCtx,
                       RegIndex regRelIdx) override;

        void updateMisc(const TarmacContext& tarmCtx,
                        RegIndex regRelIdx) override;

        uint8_t regWidth;
    };

    /**
     * Memory Entry for V8
     */
    struct TraceMemEntryV8: public TraceMemEntry, TraceEntryV8
    {
      public:
        TraceMemEntryV8(const TarmacContext& tarmCtx,
                        uint8_t _size, Addr _addr, uint64_t _data);

        virtual void print(std::ostream& outs,
                           int verbosity = 0,
                           const std::string &prefix = "") const override;

      protected:
        Addr paddr;
    };

  public:
    TarmacTracerRecordV8(Tick _when, ThreadContext *_thread,
                         const StaticInstPtr _staticInst, TheISA::PCState _pc,
                         TarmacTracer& _parent,
                         const StaticInstPtr _macroStaticInst = NULL)
      : TarmacTracerRecord(_when, _thread, _staticInst, _pc,
                           _parent, _macroStaticInst)
    {}

  protected:
    /** Generates an Entry for the executed instruction. */
    void addInstEntry(std::vector<InstPtr>& queue, const TarmacContext& ptr);

    /** Generates an Entry for every memory access triggered */
    void addMemEntry(std::vector<MemPtr>& queue, const TarmacContext& ptr);

    /** Generate a Record for every register being written */
    void addRegEntry(std::vector<RegPtr>& queue, const TarmacContext& ptr);
};

} // namespace Trace

#endif // __ARCH_ARM_TRACERS_TARMAC_RECORD_V8_HH__
