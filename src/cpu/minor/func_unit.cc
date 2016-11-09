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
 *
 * Authors: Andrew Bardsley
 */

#include "cpu/minor/func_unit.hh"

#include <iomanip>
#include <sstream>
#include <typeinfo>

#include "debug/MinorTiming.hh"
#include "enums/OpClass.hh"

MinorOpClass *
MinorOpClassParams::create()
{
    return new MinorOpClass(this);
}

MinorOpClassSet *
MinorOpClassSetParams::create()
{
    return new MinorOpClassSet(this);
}

MinorFUTiming *
MinorFUTimingParams::create()
{
    return new MinorFUTiming(this);
}

MinorFU *
MinorFUParams::create()
{
    return new MinorFU(this);
}

MinorFUPool *
MinorFUPoolParams::create()
{
    return new MinorFUPool(this);
}

MinorOpClassSet::MinorOpClassSet(const MinorOpClassSetParams *params) :
    SimObject(params),
    opClasses(params->opClasses),
    /* Initialise to true for an empty list so that 'fully capable' is
     *  the default */
    capabilityList(Num_OpClasses, (opClasses.empty() ? true : false))
{
    for (unsigned int i = 0; i < opClasses.size(); i++)
        capabilityList[opClasses[i]->opClass] = true;
}

MinorFUTiming::MinorFUTiming(
    const MinorFUTimingParams *params) :
    SimObject(params),
    mask(params->mask),
    match(params->match),
    description(params->description),
    suppress(params->suppress),
    extraCommitLat(params->extraCommitLat),
    extraCommitLatExpr(params->extraCommitLatExpr),
    extraAssumedLat(params->extraAssumedLat),
    srcRegsRelativeLats(params->srcRegsRelativeLats),
    opClasses(params->opClasses)
{ }

namespace Minor
{

void
QueuedInst::reportData(std::ostream &os) const
{
    inst->reportData(os);
}

FUPipeline::FUPipeline(const std::string &name, const MinorFU &description_,
    ClockedObject &timeSource_) :
    FUPipelineBase(name, "insts", description_.opLat),
    description(description_),
    timeSource(timeSource_),
    nextInsertCycle(Cycles(0))
{
    /* Issue latencies are set to 1 in calls to addCapability here.
     * Issue latencies are associated with the pipeline as a whole,
     * rather than instruction classes in Minor */

    /* All pipelines should be able to execute No_OpClass instructions */
    addCapability(No_OpClass, description.opLat, 1);

    /* Add the capabilities listed in the MinorFU for this functional unit */
    for (unsigned int i = 0; i < description.opClasses->opClasses.size();
         i++)
    {
        addCapability(description.opClasses->opClasses[i]->opClass,
            description.opLat, 1);
    }

    for (unsigned int i = 0; i < description.timings.size(); i++) {
        MinorFUTiming &timing = *(description.timings[i]);

        if (DTRACE(MinorTiming)) {
            std::ostringstream lats;

            unsigned int num_lats = timing.srcRegsRelativeLats.size();
            unsigned int j = 0;
            while (j < num_lats) {
                lats << timing.srcRegsRelativeLats[j];

                j++;
                if (j != num_lats)
                    lats << ',';
            }

            DPRINTFS(MinorTiming, static_cast<Named *>(this),
                "Adding extra timing decode pattern %d to FU"
                " mask: %016x match: %016x srcRegLatencies: %s\n",
                i, timing.mask, timing.match, lats.str());
        }
    }

    const std::vector<unsigned> &cant_forward =
        description.cantForwardFromFUIndices;

    /* Setup the bit vector cantForward... with the set indices
     *  specified in the parameters */
    for (auto i = cant_forward.begin(); i != cant_forward.end(); ++i) {
        cantForwardFromFUIndices.resize((*i) + 1, false);
        cantForwardFromFUIndices[*i] = true;
    }
}

Cycles
FUPipeline::cyclesBeforeInsert()
{
    if (nextInsertCycle == 0 || timeSource.curCycle() > nextInsertCycle)
        return Cycles(0);
    else
        return nextInsertCycle - timeSource.curCycle();
}

bool
FUPipeline::canInsert() const
{
    return nextInsertCycle == 0 || timeSource.curCycle() >= nextInsertCycle;
}

void
FUPipeline::advance()
{
    bool was_stalled = stalled;

    /* If an instruction was pushed into the pipeline, set the delay before
     *  the next instruction can follow */
    if (alreadyPushed()) {
        if (nextInsertCycle <= timeSource.curCycle()) {
            nextInsertCycle = timeSource.curCycle() + description.issueLat;
        }
    } else if (was_stalled && nextInsertCycle != 0) {
        /* Don't count stalled cycles as part of the issue latency */
        ++nextInsertCycle;
    }
    FUPipelineBase::advance();
}

MinorFUTiming *
FUPipeline::findTiming(const StaticInstPtr &inst)
{
#if THE_ISA == ARM_ISA
    /* This should work for any ISA with a POD mach_inst */
    TheISA::ExtMachInst mach_inst = inst->machInst;
#else
    /* Just allow extra decode based on op classes */
    uint64_t mach_inst = 0;
#endif

    const std::vector<MinorFUTiming *> &timings =
        description.timings;
    unsigned int num_timings = timings.size();

    for (unsigned int i = 0; i < num_timings; i++) {
        MinorFUTiming &timing = *timings[i];

        if (timing.provides(inst->opClass()) &&
            (mach_inst & timing.mask) == timing.match)
        {
            DPRINTFS(MinorTiming, static_cast<Named *>(this),
                "Found extra timing match (pattern %d '%s')"
                " %s %16x (type %s)\n",
                i, timing.description, inst->disassemble(0), mach_inst,
                typeid(inst).name());

            return &timing;
        }
    }

    if (num_timings != 0) {
        DPRINTFS(MinorTiming, static_cast<Named *>(this),
            "No extra timing info. found for inst: %s"
            " mach_inst: %16x\n",
            inst->disassemble(0), mach_inst);
    }

    return NULL;
}

}
