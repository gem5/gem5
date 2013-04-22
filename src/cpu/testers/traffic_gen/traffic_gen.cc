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
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 */

#include <sstream>

#include "base/random.hh"
#include "cpu/testers/traffic_gen/traffic_gen.hh"
#include "debug/Checkpoint.hh"
#include "debug/TrafficGen.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace std;

TrafficGen::TrafficGen(const TrafficGenParams* p)
    : MemObject(p),
      system(p->system),
      masterID(system->getMasterId(name())),
      port(name() + ".port", *this),
      stateGraph(*this, port, p->config_file, masterID),
      updateStateGraphEvent(this)
{
}

TrafficGen*
TrafficGenParams::create()
{
    return new TrafficGen(this);
}

BaseMasterPort&
TrafficGen::getMasterPort(const string& if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

void
TrafficGen::init()
{
    if (!port.isConnected())
        fatal("The port of %s is not connected!\n", name());

    // if the system is in timing mode active the request generator
    if (system->isTimingMode()) {
        DPRINTF(TrafficGen, "Timing mode, activating request generator\n");

        // enter initial state
        stateGraph.enterState(stateGraph.currState);
    } else {
        DPRINTF(TrafficGen,
                "Traffic generator is only active in timing mode\n");
    }
}

void
TrafficGen::initState()
{
    // when not restoring from a checkpoint, make sure we kick things off
    if (system->isTimingMode()) {
        Tick nextStateGraphEvent = stateGraph.nextEventTick();
        schedule(updateStateGraphEvent, nextStateGraphEvent);
    } else {
        DPRINTF(TrafficGen,
                "Traffic generator is only active in timing mode\n");
    }
}

unsigned int
TrafficGen::drain(DrainManager *dm)
{
    // @todo we should also stop putting new requests in the queue and
    // either interrupt the current state or wait for a transition
    return port.drain(dm);
}

void
TrafficGen::serialize(ostream &os)
{
    DPRINTF(Checkpoint, "Serializing TrafficGen\n");

    // save ticks of the graph event if it is scheduled
    Tick nextStateGraphEvent = updateStateGraphEvent.scheduled() ?
        updateStateGraphEvent.when() : 0;

    DPRINTF(TrafficGen, "Saving nextStateGraphEvent=%llu\n",
            nextStateGraphEvent);

    SERIALIZE_SCALAR(nextStateGraphEvent);

    Tick nextTransitionTick = stateGraph.nextTransitionTick;
    SERIALIZE_SCALAR(nextTransitionTick);

    // @todo: also serialise the current state, figure out the best
    // way to drain and restore
}

void
TrafficGen::unserialize(Checkpoint* cp, const string& section)
{
    // restore scheduled events
    Tick nextStateGraphEvent;
    UNSERIALIZE_SCALAR(nextStateGraphEvent);
    if (nextStateGraphEvent != 0) {
        schedule(updateStateGraphEvent, nextStateGraphEvent);
    }

    Tick nextTransitionTick;
    UNSERIALIZE_SCALAR(nextTransitionTick);
    stateGraph.nextTransitionTick = nextTransitionTick;
}

void
TrafficGen::updateStateGraph()
{
    // schedule next update event based on either the next execute
    // tick or the next transition, which ever comes first
    Tick nextStateGraphEvent = stateGraph.nextEventTick();
    DPRINTF(TrafficGen, "Updating state graph, next event at %lld\n",
            nextStateGraphEvent);
    schedule(updateStateGraphEvent, nextStateGraphEvent);

    // perform the update associated with the current update event
    stateGraph.update();
}

void
TrafficGen::StateGraph::parseConfig(const string& file_name,
                                    MasterID master_id)
{
    // keep track of the transitions parsed to create the matrix when
    // done
    vector<Transition> transitions;

    // open input file
    ifstream infile;
    infile.open(file_name.c_str(), ifstream::in);
    if (!infile.is_open()) {
        fatal("Traffic generator %s config file not found at %s\n",
              owner.name(), file_name);
    }

    // read line by line and determine the action based on the first
    // keyword
    string keyword;
    string line;

    while (getline(infile, line).good()) {
        // see if this line is a comment line, and if so skip it
        if (line.find('#') != 1) {
            // create an input stream for the tokenization
            istringstream is(line);

            // determine the keyword
            is >> keyword;

            if (keyword == "STATE") {
                // parse the behaviour of this state
                uint32_t id;
                Tick duration;
                string mode;

                is >> id >> duration >> mode;

                if (mode == "TRACE") {
                    string traceFile;
                    Addr addrOffset;

                    is >> traceFile >> addrOffset;

                    states[id] = new TraceGen(port, master_id, duration,
                                              traceFile, addrOffset);
                    DPRINTF(TrafficGen, "State: %d TraceGen\n", id);
                } else if (mode == "IDLE") {
                    states[id] = new IdleGen(port, master_id, duration);
                    DPRINTF(TrafficGen, "State: %d IdleGen\n", id);
                } else if (mode == "LINEAR" || mode == "RANDOM") {
                    uint32_t read_percent;
                    Addr start_addr;
                    Addr end_addr;
                    Addr blocksize;
                    Tick min_period;
                    Tick max_period;
                    Addr data_limit;

                    is >> read_percent >> start_addr >> end_addr >>
                        blocksize >> min_period >> max_period >> data_limit;

                    DPRINTF(TrafficGen, "%s, addr %x to %x, size %d,"
                            " period %d to %d, %d%% reads\n",
                            mode, start_addr, end_addr, blocksize, min_period,
                            max_period, read_percent);

                    if (read_percent > 100)
                        panic("%s cannot have more than 100% reads", name());

                    if (mode == "LINEAR") {
                        states[id] = new LinearGen(port, master_id,
                                                   duration, start_addr,
                                                   end_addr, blocksize,
                                                   min_period, max_period,
                                                   read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d LinearGen\n", id);
                    } else if (mode == "RANDOM") {
                        states[id] = new RandomGen(port, master_id,
                                                   duration, start_addr,
                                                   end_addr, blocksize,
                                                   min_period, max_period,
                                                   read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d RandomGen\n", id);
                    }
                } else {
                    fatal("%s: Unknown traffic generator mode: %s",
                          name(), mode);
                }
            } else if (keyword == "TRANSITION") {
                Transition transition;

                is >> transition.from >> transition.to >> transition.p;

                transitions.push_back(transition);

                DPRINTF(TrafficGen, "Transition: %d -> %d\n", transition.from,
                        transition.to);
            } else if (keyword == "INIT") {
                // set the initial state as the active state
                is >> currState;

                DPRINTF(TrafficGen, "Initial state: %d\n", currState);
            }
        }
    }

    // resize and populate state transition matrix
    transitionMatrix.resize(transitions.size());
    for (size_t i = 0; i < transitions.size(); i++) {
        transitionMatrix[i].resize(transitions.size());
    }

    for (vector<Transition>::iterator t = transitions.begin();
         t != transitions.end(); ++t) {
        transitionMatrix[t->from][t->to] = t->p;
    }

    // ensure the egress edges do not have a probability larger than
    // one
    for (size_t i = 0; i < transitions.size(); i++) {
        double sum = 0;
        for (size_t j = 0; j < transitions.size(); j++) {
            sum += transitionMatrix[i][j];
        }

        // avoid comparing floating point numbers
        if (abs(sum - 1.0) > 0.001)
            fatal("%s has transition probability != 1 for state %d\n",
                  name(), i);
    }

    // close input file
    infile.close();
}

void
TrafficGen::StateGraph::update()
{
    // if we have reached the time for the next state transition, then
    // perform the transition
    if (curTick() >= nextTransitionTick) {
        transition();
    } else {
        // we are still in the current state and should execute it
        states[currState]->execute();
    }
}

void
TrafficGen::StateGraph::transition()
{
    // exit the current state
    states[currState]->exit();

    // determine next state
    double p = random_mt.gen_real1();
    assert(currState < transitionMatrix.size());
    double cumulative = 0.0;
    size_t i = 0;
    do {
        cumulative += transitionMatrix[currState][i];
        ++i;
    } while (cumulative < p && i < transitionMatrix[currState].size());

    enterState(i - 1);
}

void
TrafficGen::StateGraph::enterState(uint32_t newState)
{
    DPRINTF(TrafficGen, "Transition to state %d\n", newState);

    currState = newState;
    nextTransitionTick += states[currState]->duration;
    states[currState]->enter();
}

bool
TrafficGen::TrafficGenPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;

    return true;
}
