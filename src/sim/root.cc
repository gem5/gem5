/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Gabe Black
 */

#include "base/logging.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/TimeSync.hh"
#include "sim/eventq_impl.hh"
#include "sim/full_system.hh"
#include "sim/root.hh"

Root *Root::_root = NULL;

/*
 * This function is called periodically by an event in M5 and ensures that
 * at least as much real time has passed between invocations as simulated time.
 * If not, the function either sleeps, or if the difference is small enough
 * spin waits.
 */
void
Root::timeSync()
{
    Time cur_time, diff, period = timeSyncPeriod();

    do {
        cur_time.setTimer();
        diff = cur_time - lastTime;
        Time remainder = period - diff;
        if (diff < period && remainder > _spinThreshold) {
            DPRINTF(TimeSync, "Sleeping to sync with real time.\n");
            // Sleep until the end of the period, or until a signal.
            sleep(remainder);
            // Refresh the current time.
            cur_time.setTimer();
        }
    } while (diff < period);
    lastTime = cur_time;
    schedule(&syncEvent, curTick() + _periodTick);
}

void
Root::timeSyncEnable(bool en)
{
    if (en == _enabled)
        return;
    _enabled = en;
    if (_enabled) {
        // Get event going.
        Tick periods = ((curTick() + _periodTick - 1) / _periodTick);
        Tick nextPeriod = periods * _periodTick;
        schedule(&syncEvent, nextPeriod);
    } else {
        // Stop event.
        deschedule(&syncEvent);
    }
}

/// Configure the period for time sync events.
void
Root::timeSyncPeriod(Time newPeriod)
{
    bool en = timeSyncEnabled();
    _period = newPeriod;
    _periodTick = _period.getTick();
    timeSyncEnable(en);
}

/// Set the threshold for time remaining to spin wait.
void
Root::timeSyncSpinThreshold(Time newThreshold)
{
    bool en = timeSyncEnabled();
    _spinThreshold = newThreshold;
    timeSyncEnable(en);
}

Root::Root(RootParams *p)
    : SimObject(p), _enabled(false), _periodTick(p->time_sync_period),
      syncEvent([this]{ timeSync(); }, name())
{
    _period.setTick(p->time_sync_period);
    _spinThreshold.setTick(p->time_sync_spin_threshold);

    assert(_root == NULL);
    _root = this;
    lastTime.setTimer();

    simQuantum = p->sim_quantum;
}

void
Root::startup()
{
    timeSyncEnable(params()->time_sync_enable);
}

void
Root::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(FullSystem);
    std::string isa = THE_ISA_STR;
    SERIALIZE_SCALAR(isa);
}


bool FullSystem;
unsigned int FullSystemInt;

Root *
RootParams::create()
{
    static bool created = false;
    if (created)
        panic("only one root object allowed!");

    created = true;

    FullSystem = full_system;
    FullSystemInt = full_system ? 1 : 0;

    return new Root(this);
}
