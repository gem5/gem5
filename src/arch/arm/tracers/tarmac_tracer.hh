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
 */

/**
 * @file: This file declares the interface of the Tarmac Tracer:
 *        the tracer based on the Tarmac specification.
 */

#ifndef __ARCH_ARM_TRACERS_TARMAC_TRACER_HH__
#define __ARCH_ARM_TRACERS_TARMAC_TRACER_HH__

#include "arch/arm/tracers/tarmac_record.hh"
#include "arch/arm/tracers/tarmac_record_v8.hh"
#include "params/TarmacTracer.hh"
#include "sim/insttracer.hh"

class ThreadContext;

namespace Trace {

/**
 * This object type is encapsulating the informations needed by
 * a Tarmac record to generate it's own entries.
 */
class TarmacContext
{
  public:
    TarmacContext(ThreadContext* _thread,
                  const StaticInstPtr _staticInst,
                  ArmISA::PCState _pc)
      : thread(_thread), staticInst(_staticInst), pc(_pc)
    {}

    std::string tarmacCpuName() const;

  public:
    ThreadContext* thread;
    const StaticInstPtr staticInst;
    ArmISA::PCState pc;
};

/**
 * Tarmac Tracer: this tracer generates a new Tarmac Record for
 * every instruction being executed in gem5.
 * The record is made by a collection of entries which are stored
 * in the tracer queues.
 */
class TarmacTracer : public InstTracer
{
    friend class TarmacTracerRecord;
    friend class TarmacTracerRecordV8;

  public:
    typedef TarmacTracerParams Params;

    TarmacTracer(const Params *p);

    /**
     * Generates a TarmacTracerRecord, depending on the Tarmac version.
     * At the moment supported formats are:
     * - Tarmac
     * - TarmacV8
     */
    InstRecord* getInstRecord(Tick when, ThreadContext *tc,
                              const StaticInstPtr staticInst,
                              ArmISA::PCState pc,
                              const StaticInstPtr macroStaticInst = NULL);

  protected:
    typedef std::unique_ptr<Printable> PEntryPtr;
    typedef TarmacTracerRecord::InstPtr InstPtr;
    typedef TarmacTracerRecord::MemPtr MemPtr;
    typedef TarmacTracerRecord::RegPtr RegPtr;

    /**
     * startTick and endTick allow to trace a specific window of ticks
     * rather than the entire CPU execution.
     */
    Tick startTick;
    Tick endTick;

    /**
     * Collection of heterogeneous printable entries: could be
     * representing either instructions, register or memory entries.
     * When dealing with MacroInstructions the following separate queues
     * will be used. micro-instruction entries will be buffered and
     * dumped to the tracefile only at the end of the Macro.
     */
    std::vector<InstPtr> instQueue;
    std::vector<MemPtr> memQueue;
    std::vector<RegPtr> regQueue;
};

} // namespace Trace

#endif // __ARCH_ARM_TRACERS_TARMAC_TRACER_HH__
