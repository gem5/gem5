/*
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
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
 * Authors: Gabe Black
 */

/**
 * @file This file defines the Root simobject and the methods used to control
 * the time syncing mechanism provided through it.
 *
 * Time syncing prevents simulated time from passing faster than real time. It
 * works by scheduling a periodic event that checks to see if its simulated
 * period is shorter than its real period. If it is, it stalls the simulation
 * until they're equal.
 */

#ifndef __SIM_ROOT_HH__
#define __SIM_ROOT_HH__

#include "base/time.hh"
#include "params/Root.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class Root : public SimObject
{
  private:
    static Root *_root;

  protected:
    bool _enabled;
    Time _period;
    Tick _periodTick;
    Time _spinThreshold;

    Time lastTime;

    void timeSync();
    EventWrapper<Root, &Root::timeSync> syncEvent;
    friend class EventWrapper<Root, &Root::timeSync>;

  public:
    /**
     * Use this function to get a pointer to the single Root object in the
     * simulation. This function asserts that such an object has actual been
     * constructed to avoid having to perform that check everywhere the root
     * is used. This is to allow calling the functions below.
     *
     * @return Pointer to the single root object.
     */
    static Root *
    root()
    {
        assert(_root);
        return _root;
    }

  public:

    /// Check whether time syncing is enabled.
    bool timeSyncEnabled() const { return _enabled; }
    /// Retrieve the period for the sync event.
    const Time timeSyncPeriod() const { return _period; }
    /// Retrieve the threshold for time remaining to spin wait.
    const Time timeSyncSpinThreshold() const { return _spinThreshold; }

    /// Enable or disable time syncing.
    void timeSyncEnable(bool en);
    /// Configure the period for time sync events.
    void timeSyncPeriod(Time newPeriod);
    /// Set the threshold for time remaining to spin wait.
    void timeSyncSpinThreshold(Time newThreshold);

    typedef RootParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Root(Params *p);

    /** Schedule the timesync event at loadState() so that curTick is correct
     */
    void loadState(CheckpointIn &cp) override;

    /** Schedule the timesync event at initState() when not unserializing
     */
    void initState() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __SIM_ROOT_HH__
