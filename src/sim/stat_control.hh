/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __SIM_STAT_CONTROL_HH__
#define __SIM_STAT_CONTROL_HH__

#include "base/compiler.hh"
#include "base/types.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Stats, statistics);
namespace statistics
{


void initSimStats();

/**
 * Update the events after resuming from a checkpoint. When resuming from a
 * checkpoint, curTick will be updated, and any already scheduled events can
 * end up scheduled in the past. This function checks if the dumpEvent is
 * scheduled in the past, and reschedules it appropriately.
 */
void updateEvents();

/**
 * Schedule statistics dumping. This allows you to dump and/or reset the
 * built-in statistics. This can either be done once, or it can be done on a
 * regular basis.
 * @param dump Set true to dump the statistics.
 * @param reset Set true to reset the statistics.
 * @param when When the dump and/or reset should occur.
 * @param repeat How often the event should repeat. Set 0 to disable repeating.
 */
void schedStatEvent(bool dump, bool reset, Tick when = curTick(),
                    Tick repeat = 0);

/**
 * Schedule periodic statistics dumping. This allows you to dump and reset the
 * built-in statistics on a regular basis, thereby allowing the extraction of
 * temporal trends in the data.
 * @param period The period at which the dumping should occur.
 */
void periodicStatDump(Tick period = 0);
} // namespace statistics
} // namespace gem5

#endif // __SIM_STAT_CONTROL_HH__
