/*
 * Copyright (c) 2021, 2026 ARM Limited
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

#ifndef __MEM_RUBY_STRUCTURES_BACKPRESSURETRACKER_HH__
#define __MEM_RUBY_STRUCTURES_BACKPRESSURETRACKER_HH__

#include <iostream>

#include "mem/ruby/common/MachineID.hh"
#include "params/BackpressureTracker.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

// Tracks responder-side backpressure information for downstream requests.
class BackpressureTracker : public SimObject
{
  public:
    PARAMS(BackpressureTracker);

    /** Construct a tracker with simulator-owned configuration parameters. */
    BackpressureTracker(const Params &p);

    /**
     * Record the latest backpressure level observed for a responder.
     *
     * @param mach_id Responder identifier associated with the observation.
     * @param cbusy Backpressure level reported by that responder.
     */
    virtual void update(const MachineID &mach_id, int cbusy) = 0;

    /**
     * Record the latest backpressure level for responses observed on an
     * alternate transfer path.
     *
     * @param mach_id Responder identifier associated with the observation.
     * @param cbusy Backpressure level reported by that responder.
     */
    virtual void updateDMT(const MachineID &mach_id, int cbusy) = 0;

    /**
     * Return a summarized backpressure level across tracked responders.
     *
     * The exact aggregation policy is implementation-defined.
     */
    virtual int report() = 0;

    /** Print a human-readable summary of the tracked backpressure state. */
    virtual void print(std::ostream &out) const = 0;
};

inline std::ostream &
operator<<(std::ostream &out, const BackpressureTracker &obj)
{
    obj.print(out);
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_BACKPRESSURETRACKER_HH__
