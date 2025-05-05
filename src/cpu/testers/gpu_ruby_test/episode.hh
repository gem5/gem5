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

#ifndef CPU_TESTERS_PROTOCOL_TESTER_EPISODE_HH_
#define CPU_TESTERS_PROTOCOL_TESTER_EPISODE_HH_

#include <vector>

#include "base/random.hh"
#include "cpu/testers/gpu_ruby_test/address_manager.hh"

namespace gem5
{

class ProtocolTester;
class TesterThread;

class Episode
{
  public:
    typedef AddressManager::Location Location;
    typedef AddressManager::Value Value;

    class Action
    {
      public:
        enum class Type
        {
            ACQUIRE,
            RELEASE,
            ATOMIC,
            LOAD,
            STORE,
        };

        Action(Type t, int num_lanes);
        ~Action() {}

        Type getType() const { return type; }
        void setLocation(int lane, Location loc);
        Location getLocation(int lane) const;
        bool isAtomicAction() const;
        bool isMemFenceAction() const;
        const std::string printType() const;

      private:
        Type type;
        int numLanes;
        typedef std::vector<Location> LocationList;
        LocationList locations;
    };

    Episode(ProtocolTester* tester, TesterThread* thread, int num_loads,
            int num_stores);
    ~Episode();

    // return episode id
    int getEpisodeId() const { return episodeId; }
    // return the action at the head of the action queue
    const Action* peekCurAction() const;
    // pop the action at the head of the action queue
    void popAction();
    // check if there is more action to be issued in this episode
    bool hasMoreActions() const { return nextActionIdx < actions.size();}
    // complete this episode by releasing all locations & updating st effects
    void completeEpisode();
    // check if this episode is executing
    bool isEpsActive() const { return isActive; }
    // check if the input episode and this one have any data race
    bool checkDRF(Location atomic_loc, Location loc, bool isStore,
                  int max_lane) const;

  private:
    // pointers to tester, thread and address amanger structures
    ProtocolTester *tester;
    TesterThread *thread;
    AddressManager *addrManager;

    // a unique episode id
    int episodeId;
    // list of actions in this episode
    typedef std::vector<Action*> ActionList;
    ActionList actions;
    // list of atomic locations picked for this episode
    typedef std::vector<Location> AtomicLocationList;
    AtomicLocationList atomicLocs;

    Random::RandomPtr rng = Random::genRandom();

    // is a thread running this episode?
    bool isActive;
    // episode length = num_loads + num_stores
    int numLoads;
    int numStores;
    // index of the next action in actions
    int nextActionIdx;
    // number of lanes in this thread
    int numLanes;

    // randomly generate actions in this episode
    void initActions();
};

} // namespace gem5

#endif /* CPU_TESTERS_PROTOCOL_TESTER_EPISODE_HH_ */
