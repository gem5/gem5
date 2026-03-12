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

#include <bitset>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "base/types.hh"
#include "cpu/func_unit.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/func_unit_op_class.hh"
#include "cpu/minor/func_unit_op_class_set.hh"
#include "cpu/minor/func_unit_timing.hh"
#include "cpu/minor/func_unit_fu.hh"
#include "cpu/minor/func_unit_pool.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

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
