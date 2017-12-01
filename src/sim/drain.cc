/*
 * Copyright (c) 2012, 2015, 2017 ARM Limited
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
 * Authors: Andreas Sandberg
 */

#include "sim/drain.hh"

#include <algorithm>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"

DrainManager DrainManager::_instance;

DrainManager::DrainManager()
    : _count(0),
      _state(DrainState::Running)
{
}

DrainManager::~DrainManager()
{
}

bool
DrainManager::tryDrain()
{
    panic_if(_state == DrainState::Drained,
             "Trying to drain a drained system\n");

    panic_if(_count != 0,
             "Drain counter must be zero at the start of a drain cycle\n");

    DPRINTF(Drain, "Trying to drain %u objects.\n", drainableCount());
    _state = DrainState::Draining;
    for (auto *obj : _allDrainable) {
        DrainState status = obj->dmDrain();
        if (DTRACE(Drain) && status != DrainState::Drained) {
            SimObject *temp = dynamic_cast<SimObject*>(obj);
            if (temp)
                DPRINTF(Drain, "Failed to drain %s\n", temp->name());
        }
        _count += status == DrainState::Drained ? 0 : 1;
    }

    if (_count == 0) {
        DPRINTF(Drain, "Drain done.\n");
        _state = DrainState::Drained;
        return true;
    } else {
        DPRINTF(Drain, "Need another drain cycle. %u/%u objects not ready.\n",
                _count, drainableCount());
        return false;
    }
}

void
DrainManager::resume()
{
    panic_if(_state == DrainState::Running,
             "Trying to resume a system that is already running\n");

    warn_if(_state == DrainState::Draining,
            "Resuming a system that isn't fully drained, this is untested and "
            "likely to break\n");

    panic_if(_state == DrainState::Resuming,
             "Resuming a system that is already trying to resume. This should "
             "never happen.\n");

    panic_if(_count != 0,
             "Resume called in the middle of a drain cycle. %u objects "
             "left to drain.\n", _count);

    // At this point in time the DrainManager and all objects will be
    // in the the Drained state. New objects (i.e., objects created
    // while resuming) will inherit the Resuming state from the
    // DrainManager, which means we have to resume objects until all
    // objects are in the Running state.
    _state = DrainState::Resuming;

    do {
        DPRINTF(Drain, "Resuming %u objects.\n", drainableCount());
        for (auto *obj : _allDrainable) {
            if (obj->drainState() != DrainState::Running) {
                assert(obj->drainState() == DrainState::Drained ||
                       obj->drainState() == DrainState::Resuming);
                obj->dmDrainResume();
            }
        }
    } while (!allInState(DrainState::Running));

    _state = DrainState::Running;
}

void
DrainManager::preCheckpointRestore()
{
    panic_if(_state != DrainState::Running,
             "preCheckpointRestore() called on a system that isn't in the "
             "Running state.\n");

    DPRINTF(Drain, "Applying pre-restore fixes to %u objects.\n",
            drainableCount());
    _state = DrainState::Drained;
    for (auto *obj : _allDrainable)
        obj->_drainState = DrainState::Drained;
}

void
DrainManager::signalDrainDone()
{
    assert(_count > 0);
    if (--_count == 0) {
        DPRINTF(Drain, "All %u objects drained..\n", drainableCount());
        exitSimLoop("Finished drain", 0);
    }
}


void
DrainManager::registerDrainable(Drainable *obj)
{
    std::lock_guard<std::mutex> lock(globalLock);
    assert(std::find(_allDrainable.begin(), _allDrainable.end(), obj) ==
           _allDrainable.end());
    _allDrainable.push_back(obj);
}

void
DrainManager::unregisterDrainable(Drainable *obj)
{
    std::lock_guard<std::mutex> lock(globalLock);
    auto o = std::find(_allDrainable.begin(), _allDrainable.end(), obj);
    assert(o != _allDrainable.end());
    _allDrainable.erase(o);
}

bool
DrainManager::allInState(DrainState state) const
{
    for (const auto *obj : _allDrainable) {
        if (obj->drainState() != state)
            return false;
    }

    return true;
}

size_t
DrainManager::drainableCount() const
{
    std::lock_guard<std::mutex> lock(globalLock);
    return _allDrainable.size();
}



Drainable::Drainable()
    : _drainManager(DrainManager::instance()),
      _drainState(_drainManager.state())
{
    _drainManager.registerDrainable(this);
}

Drainable::~Drainable()
{
    _drainManager.unregisterDrainable(this);
}

DrainState
Drainable::dmDrain()
{
    _drainState = DrainState::Draining;
    _drainState = drain();
    assert(_drainState == DrainState::Draining ||
           _drainState == DrainState::Drained);

    return _drainState;
}

void
Drainable::dmDrainResume()
{
    panic_if(_drainState != DrainState::Drained &&
             _drainState != DrainState::Resuming,
             "Trying to resume an object that hasn't been drained\n");

    _drainState = DrainState::Running;
    drainResume();
}
