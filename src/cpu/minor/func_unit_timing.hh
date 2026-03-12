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

#ifndef __CPU_MINOR_FUNC_UNIT_TIMING_HH__
#define __CPU_MINOR_FUNC_UNIT_TIMING_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "base/types.hh"
#include "cpu/minor/func_unit_op_class_set.hh"
#include "cpu/timing_expr.hh"
#include "params/MinorFUTiming.hh"
#include "sim/sim_object.hh"

namespace gem5
{

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

} // namespace gem5

#endif /* __CPU_MINOR_FUNC_UNIT_TIMING_HH__ */
