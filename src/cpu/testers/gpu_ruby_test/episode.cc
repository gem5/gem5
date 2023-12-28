/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/testers/gpu_ruby_test/episode.hh"

#include <fstream>
#include <unordered_set>

#include "base/random.hh"
#include "cpu/testers/gpu_ruby_test/protocol_tester.hh"
#include "cpu/testers/gpu_ruby_test/tester_thread.hh"

namespace gem5
{

Episode::Episode(ProtocolTester* _tester, TesterThread* _thread, int num_loads,
                 int num_stores)
      : tester(_tester),
        thread(_thread),
        numLoads(num_loads),
        numStores(num_stores),
        nextActionIdx(0)
{
    assert(tester && thread);

    episodeId = tester->getNextEpisodeID();
    numLanes = thread->getNumLanes();
    assert(numLanes > 0);

    addrManager = tester->getAddressManager();
    assert(addrManager);

    atomicLocs.resize(numLanes, AddressManager::INVALID_LOCATION);
    // generate a sequence of actions
    initActions();
    isActive = true;

    DPRINTFN("Episode %d\n", episodeId);
}

Episode::~Episode()
{
    for (Episode::Action* action : actions) {
        assert(action);
        delete action;
    }
}

const Episode::Action*
Episode::peekCurAction() const
{
    if (nextActionIdx < actions.size())
        return actions[nextActionIdx];
    else
        return nullptr;
}

void
Episode::popAction()
{
    assert(nextActionIdx < actions.size());
    nextActionIdx++;
}

void
Episode::initActions()
{
    // first, push Atomic & then Acquire action
    actions.push_back(new Action(Action::Type::ATOMIC, numLanes));
    actions.push_back(new Action(Action::Type::ACQUIRE, numLanes));

    // second, push a number of LD/ST actions
    int num_loads = numLoads;
    int num_stores = numStores;
    while ((num_loads + num_stores) > 0) {
        switch (random_mt.random<unsigned int>() % 2) {
            case 0: // Load
                if (num_loads > 0) {
                    actions.push_back(new Action(Action::Type::LOAD,
                                                   numLanes));
                    num_loads--;
                }
                break;
            case 1: // Store
                if (num_stores > 0) {
                    actions.push_back(new Action(Action::Type::STORE,
                                                   numLanes));
                    num_stores--;
                }
                break;
            default:
                assert(false);
        }
    }

    // last, push an Release & then Atomic action
    actions.push_back(new Action(Action::Type::RELEASE, numLanes));
    actions.push_back(new Action(Action::Type::ATOMIC, numLanes));

    // for each lane, pick a list of locations
    Location normal_loc;

    for (int lane = 0; lane < numLanes; ++lane) {
        normal_loc = AddressManager::INVALID_LOCATION;

        // first, we select atomic loc for this lane
        // atomic loc for this lane should not have been picked yet
        assert(atomicLocs[lane] == AddressManager::INVALID_LOCATION);
        // pick randomly an atomic location
        atomicLocs[lane] = addrManager->getAtomicLoc();
        assert(atomicLocs[lane] >= 0);

        // go through each action in this lane and set its location
        for (Action* action : actions) {
            assert(action);

            switch (action->getType()) {
                case Action::Type::ATOMIC:
                    action->setLocation(lane, atomicLocs[lane]);
                    break;
                case Action::Type::LOAD:
                    // pick randomly a normal location
                    normal_loc = addrManager->
                                            getLoadLoc(atomicLocs[lane]);
                    assert(normal_loc >= AddressManager::INVALID_LOCATION);

                    if (normal_loc != AddressManager::INVALID_LOCATION) {
                        // check DRF
                        if (!tester->checkDRF(atomicLocs[lane],
                                                normal_loc, false) ||
                            !this->checkDRF(atomicLocs[lane], normal_loc,
                                            false, lane)) {
                            panic("TestTh %d - Data race detected. STOPPED!\n",
                                  thread->getTesterThreadId());
                        }
                    }

                    action->setLocation(lane, normal_loc);
                    break;
                case Action::Type::STORE:
                    // pick randomly a normal location
                    normal_loc = addrManager->
                                            getStoreLoc(atomicLocs[lane]);
                    assert(normal_loc >= AddressManager::INVALID_LOCATION);

                    if (normal_loc != AddressManager::INVALID_LOCATION) {
                        // check DRF
                        if (!tester->checkDRF(atomicLocs[lane],
                                                normal_loc, true) ||
                            !this->checkDRF(atomicLocs[lane], normal_loc,
                                            true, lane)) {
                            panic("TestTh %d - Data race detected. STOPPED!\n",
                                  thread->getTesterThreadId());
                        }
                    }

                    action->setLocation(lane, normal_loc);
                    break;
                case Action::Type::ACQUIRE:
                case Action::Type::RELEASE:
                    // no op
                    break;
                default:
                    panic("Invalid action type\n");
            }
        }

        addrManager->finishLocSelection(atomicLocs[lane]);
    }
}

void
Episode::completeEpisode()
{
    // release all locations this episode has picked and used
    Location atomic_loc, normal_loc;
    for (int lane = 0; lane < numLanes; ++lane) {
        atomic_loc = AddressManager::INVALID_LOCATION;
        normal_loc = AddressManager::INVALID_LOCATION;

        std::unordered_set<Location> unique_loc_set;

        for (Action* action : actions) {
            assert(action);

            if (action->isAtomicAction()) {
                if (atomic_loc == AddressManager::INVALID_LOCATION) {
                    atomic_loc = action->getLocation(lane);
                } else {
                    // both atomic ops in the same lane must be
                    // at the same location
                    assert(atomic_loc == action->getLocation(lane));
                }
            } else if (!action->isMemFenceAction()) {
                assert(atomic_loc >= 0);
                normal_loc = action->getLocation(lane);

                if (normal_loc >= 0)
                    unique_loc_set.insert(normal_loc);
            }
        }

        // each unique loc can be released only once
        for (Location loc : unique_loc_set)
            addrManager->releaseLocation(atomic_loc, loc);
    }

    // this episode is no longer active
    isActive = false;
}

bool
Episode::checkDRF(Location atomic_loc, Location loc, bool isStore,
                  int max_lane) const
{
    assert(atomic_loc != AddressManager::INVALID_LOCATION);
    assert(loc != AddressManager::INVALID_LOCATION);
    assert(max_lane <= numLanes);

    for (int lane = 0; lane < max_lane; ++lane) {
        if (atomic_loc == atomicLocs[lane]) {
            for (const Action* action : actions) {
                if (!action->isAtomicAction() &&
                    !action->isMemFenceAction()) {
                    if (isStore && loc == action->getLocation(lane)) {
                        warn("ST at location %d races against thread %d\n",
                             loc, thread->getTesterThreadId());
                        return false;
                    } else if (!isStore &&
                               action->getType() == Action::Type::STORE &&
                               loc == action->getLocation(lane)) {
                        warn("LD at location %d races against thread %d\n",
                             loc, thread->getTesterThreadId());
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

// -------------------- Action class ----------------------------
Episode::Action::Action(Type t, int num_lanes)
    : type(t),
      numLanes(num_lanes)
{
    assert(numLanes > 0);
    locations.resize(numLanes);
    for (Location &loc : locations) loc = AddressManager::INVALID_LOCATION;
}

void
Episode::Action::setLocation(int lane, Location loc)
{
    assert(lane >= 0 && lane < numLanes);
    locations[lane] = loc;
}

AddressManager::Location
Episode::Action::getLocation(int lane) const
{
    assert(lane >= 0 && lane < numLanes);
    return locations[lane];
}

bool
Episode::Action::isAtomicAction() const
{
    return (type == Type::ATOMIC);
}

bool
Episode::Action::isMemFenceAction() const
{
    return (type == Type::ACQUIRE || type == Type::RELEASE);
}

const std::string
Episode::Action::printType() const
{
    if (type == Type::ACQUIRE)
        return "ACQUIRE";
    else if (type == Type::RELEASE)
        return "RELEASE";
    else if (type == Type::ATOMIC)
        return "ATOMIC";
    else if (type == Type::LOAD)
        return "LOAD";
    else if (type == Type::STORE)
        return "STORE";
    else
        panic("Invalid action type\n");
}

} // namespace gem5
