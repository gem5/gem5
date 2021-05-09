/*
 * Copyright (c) 2013-2014 ARM Limited
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
 * @file
 *
 *  Execute function unit descriptions and pipeline implementations.
 */

#ifndef __CPU_MINOR_FUNC_UNIT_HH__
#define __CPU_MINOR_FUNC_UNIT_HH__

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "base/types.hh"
#include "cpu/func_unit.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/timing_expr.hh"
#include "params/MinorFU.hh"
#include "params/MinorFUPool.hh"
#include "params/MinorOpClass.hh"
#include "params/MinorOpClassSet.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/** Boxing for MinorOpClass to get around a build problem with C++11 but
 *  also allow for future additions to op class checking */
class MinorOpClass : public SimObject
{
  public:
    OpClass opClass;

  public:
    MinorOpClass(const MinorOpClassParams &params) :
        SimObject(params),
        opClass(params.opClass)
    { }
};

/** Wrapper for a matchable set of op classes */
class MinorOpClassSet : public SimObject
{
  public:
    std::vector<MinorOpClass *> opClasses;

    /** Convenience packing of opClasses into a bit vector for easier
     *  testing */
    std::vector<bool> capabilityList;

  public:
    MinorOpClassSet(const MinorOpClassSetParams &params);

  public:
    /** Does this set support the given op class */
    bool provides(OpClass op_class) { return capabilityList[op_class]; }
};

/** Extra timing capability to allow individual ops to have their source
 *  register dependency latencies tweaked based on the ExtMachInst of the
 *  source instruction.
 */
class MinorFUTiming: public SimObject
{
  public:
    /** Mask off the ExtMachInst of an instruction before comparing with
     *  match */
    uint64_t mask;
    uint64_t match;

    /** Textual description of the decode's purpose */
    std::string description;

    /** If true, instructions matching this mask/match should *not* be
     *  issued in this FU */
    bool suppress;

    /** Extra latency that the instruction should spend at the end of
     *  the pipeline */
    Cycles extraCommitLat;
    TimingExpr *extraCommitLatExpr;

    /** Extra delay that results should show in the scoreboard after
     *  leaving the pipeline.  If set to Cycles(0) for memory references,
     *  an 'unpredictable' return time will be set in the scoreboard
     *  blocking following dependent instructions from issuing */
    Cycles extraAssumedLat;

    /** Cycle offsets from the scoreboard delivery times of register values
     *  for each of this instruction's source registers (in srcRegs order).
     *  The offsets are subtracted from the scoreboard returnCycle times.
     *  For example, for an instruction type with 3 source registers,
     *  [2, 1, 2] will allow the instruction to issue upto 2 cycles early
     *  for dependencies on the 1st and 3rd register and upto 1 cycle early
     *  on the 2nd. */
    std::vector<Cycles> srcRegsRelativeLats;

    /** Extra opClasses check (after the FU one) */
    MinorOpClassSet *opClasses;

  public:
    MinorFUTiming(const MinorFUTimingParams &params);

  public:
    /** Does the extra decode in this object support the given op class */
    bool provides(OpClass op_class) { return opClasses->provides(op_class); }
};

/** A functional unit that can execute any of opClasses operations with a
 *  single op(eration)Lat(ency) and issueLat(ency) associated with the unit
 *  rather than each operation (as in src/FuncUnit).
 *
 *  This is very similar to cpu/func_unit but replicated here to allow
 *  the Minor functional units to change without having to disturb the common
 *  definition.
 */
class MinorFU : public SimObject
{
  public:
    MinorOpClassSet *opClasses;

    /** Delay from issuing the operation, to it reaching the
     *  end of the associated pipeline */
    Cycles opLat;

    /** Delay after issuing an operation before the next
     *  operation can be issued */
    Cycles issueLat;

    /** FUs which this pipeline can't receive a forwarded (i.e. relative
     *  latency != 0) result from */
    std::vector<unsigned int> cantForwardFromFUIndices;

    /** Extra timing info to give timings to individual ops */
    std::vector<MinorFUTiming *> timings;

  public:
    MinorFU(const MinorFUParams &params) :
        SimObject(params),
        opClasses(params.opClasses),
        opLat(params.opLat),
        issueLat(params.issueLat),
        cantForwardFromFUIndices(params.cantForwardFromFUIndices),
        timings(params.timings)
    { }
};

/** A collection of MinorFUs */
class MinorFUPool : public SimObject
{
  public:
    std::vector<MinorFU *> funcUnits;

  public:
    MinorFUPool(const MinorFUPoolParams &params) :
        SimObject(params),
        funcUnits(params.funcUnits)
    { }
};

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{

/** Container class to box instructions in the FUs to make those
 *  queues have correct bubble behaviour when stepped */
class QueuedInst
{
  public:
    MinorDynInstPtr inst;

  public:
    QueuedInst(MinorDynInstPtr inst_ = MinorDynInst::bubble()) :
        inst(inst_)
    { }

  public:
    /** Report and bubble interfaces */
    void reportData(std::ostream &os) const;
    bool isBubble() const { return inst->isBubble(); }

    static QueuedInst bubble()
    { return QueuedInst(MinorDynInst::bubble()); }
};

/** Functional units have pipelines which stall when an inst gets to
 *  their ends allowing Execute::commit to pick up timing-completed insts
 *  when it feels like it */
typedef SelfStallingPipeline<QueuedInst,
    ReportTraitsAdaptor<QueuedInst> > FUPipelineBase;

/** A functional unit configured from a MinorFU object */
class FUPipeline : public FUPipelineBase, public FuncUnit
{
  public:
    /** Functional unit description that this pipeline implements */
    const MinorFU &description;

    /** An FUPipeline needs access to curCycle, use this timing source */
    ClockedObject &timeSource;

    /** Set of operation classes supported by this FU */
    std::bitset<Num_OpClasses> capabilityList;

    /** FUs which this pipeline can't receive a forwarded (i.e. relative
     *  latency != 0) result from */
    std::vector<bool> cantForwardFromFUIndices;

  public:
    /** When can a new instruction be inserted into the pipeline?  This is
     *  an absolute cycle time unless it is 0 in which case the an
     *  instruction can be pushed straightaway */
    Cycles nextInsertCycle;

  public:
    FUPipeline(const std::string &name, const MinorFU &description_,
        ClockedObject &timeSource_);

  public:
    /** How many cycles must from curCycle before insertion into the
     *  pipeline is allowed */
    Cycles cyclesBeforeInsert();

    /** Can an instruction be inserted now? */
    bool canInsert() const;

    /** Find the extra timing information for this instruction.  Returns
     *  NULL if no decode info. is found */
    MinorFUTiming *findTiming(const StaticInstPtr &inst);

    /** Step the pipeline.  Allow multiple steps? */
    void advance();
};

} // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_FUNC_UNIT_HH__ */
