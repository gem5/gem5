/*
 * Copyright (c) 2026 Arm Limited
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

#ifndef __MEM_RUBY_PROTOCOL_CHI_GENERIC_CBUSY_HH__
#define __MEM_RUBY_PROTOCOL_CHI_GENERIC_CBUSY_HH__

#include <unordered_map>

#include "base/statistics.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/structures/BackpressureGen.hh"
#include "mem/ruby/structures/BackpressureTracker.hh"
#include "params/BaseCBusy.hh"
#include "params/CBusyTracker.hh"

namespace gem5
{

namespace ruby
{

namespace CHI
{

/**
 * Base class for CHI CBusy generators.
 *
 * Subclasses map controller state to a protocol CBusy level using
 * configurable occupancy thresholds.
 */
class BaseCBusy : public BackpressureGen
{
  public:
    PARAMS(BaseCBusy);

    /** Construct a CBusy generator with configured threshold levels. */
    BaseCBusy(const Params &p);

  protected:
    /** Lower bound for CBusy level 1. */
    const unsigned threshold0;
    /** Lower bound for CBusy level 2. */
    const unsigned threshold1;
    /** Lower bound for CBusy level 3. */
    const unsigned threshold2;
};

/**
 * Common CBusy generator base for controllers exposing TBE occupancy APIs.
 * The TBE occupancy determines the CBusy value
 */
class TbeCBusy : public BaseCBusy
{
  public:
    TbeCBusy(const BaseCBusyParams &p) : BaseCBusy(p) {}

  protected:
    template <typename Controller>
    int
    generateFromTbeOccupancy(AbstractController *ctrl) const
    {
        auto chi_ctrl = dynamic_cast<Controller *>(ctrl);
        assert(chi_ctrl);

        auto tbe_used = chi_ctrl->tbeSize() + chi_ctrl->tbeReserved();
        auto tbe_capacity = chi_ctrl->tbeCapacity();
        auto occupancy = (100 * tbe_used) / tbe_capacity;

        // TBE occupancy thresholds for setting CBusy:
        // 0 = occupancy < 50%; 1 = 50%-75%; 2 = 75%-90%; 3 = >90%
        if (occupancy < threshold0) {
            return 0;
        } else if (occupancy < threshold1) {
            return 1;
        } else if (occupancy < threshold2) {
            return 2;
        } else {
            return 3;
        }
    }
};

/**
 * Cache-side CBusy generator.
 *
 * Generates levels 0..3 based only on TBE occupancy.
 */
class CBusy : public TbeCBusy
{
  public:
    CBusy(const BaseCBusyParams &p) : TbeCBusy(p) {}

    /** Compute CBusy from cache controller occupancy. */
    int generate(AbstractController *ctrl,
                 const Message &message) const override;
};

/**
 * Memory-side CBusy generator.
 *
 * Generates levels 0..3 based only on TBE occupancy.
 */
class SnfCBusy : public TbeCBusy
{
  public:
    SnfCBusy(const BaseCBusyParams &p) : TbeCBusy(p) {}

    /** Compute CBusy from memory controller occupancy. */
    int generate(AbstractController *ctrl,
                 const Message &message) const override;
};

/**
 * CHI BackpressureTracker implementation based on observed response CBusy.
 *
 * Tracking algorithm:
 * - Keep one entry per responder with its latest observed CBusy value.
 * - Maintain a running sum of those latest values.
 * - On update for responder R, subtract R's previous value from the sum,
 *   store the new value, then add the new value to the sum.
 *
 * This makes each update O(1) and allows efficient aggregate reporting:
 * max uses the per-responder map, while average uses the running sum.
 */
class CBusyTracker : public BackpressureTracker
{
  private:
    class CBusyTrackerHelper
    {
      public:
        /**
         * Latest observed CBusy per responder.
         * Each responder contributes exactly one value to cbusySum.
         */
        std::unordered_map<MachineID, int> cbusyEntries;
        /**
         * Running sum of the current values in cbusyEntries.
         * On update, we subtract the responder's previous value and add the
         * new one, so this remains the sum across all responders.
         */
        int cbusySum;

        CBusyTrackerHelper() : cbusySum(0) {}

        void updateCBusy(const MachineID &mach_id, int cbusy);
        int avgCBusy() const;
        int maxCBusy() const;
    };

    CBusyTrackerHelper entries;
    CBusyTrackerHelper dmtEntries;

    struct CBusyTrackerStats : public statistics::Group
    {
        CBusyTrackerStats(statistics::Group *parent);

        statistics::Average avgCBusy;
        statistics::Average avgCBusyDMT;
    } stats;

  public:
    PARAMS(CBusyTracker);
    CBusyTracker(const Params &p);

    void update(const MachineID &mach_id, int cbusy) override;
    void updateDMT(const MachineID &mach_id, int cbusy) override;
    int
    report() override
    {
        return entries.maxCBusy();
    }
    void print(std::ostream &out) const override;
};

} // namespace CHI
} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_GENERIC_CBUSY_HH__
