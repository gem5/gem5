/*
 * Copyright (c) 2012-2013, 2016 ARM Limited
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
#include "cpu/testers/traffic_gen/traffic_gen.hh"

#include <libgen.h>
#include <unistd.h>

#include <sstream>

#include "base/intmath.hh"
#include "base/random.hh"
#include "debug/Checkpoint.hh"
#include "debug/TrafficGen.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace std;

TrafficGen::TrafficGen(const TrafficGenParams* p)
    : MemObject(p),
      system(p->system),
      masterID(system->getMasterId(name())),
      configFile(p->config_file),
      elasticReq(p->elastic_req),
      progressCheck(p->progress_check),
      noProgressEvent(this),
      nextTransitionTick(0),
      nextPacketTick(0),
      currState(0),
      port(name() + ".port", *this),
      retryPkt(NULL),
      retryPktTick(0),
      updateEvent(this),
      numSuppressed(0)
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

        parseConfig();

        // enter initial state
        enterState(currState);
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
        // call nextPacketTick on the state to advance it
        nextPacketTick = states[currState]->nextPacketTick(elasticReq, 0);
        schedule(updateEvent, std::min(nextPacketTick, nextTransitionTick));
    } else {
        DPRINTF(TrafficGen,
                "Traffic generator is only active in timing mode\n");
    }
}

DrainState
TrafficGen::drain()
{
    if (!updateEvent.scheduled()) {
        // no event has been scheduled yet (e.g. switched from atomic mode)
        return DrainState::Drained;
    }

    if (retryPkt == NULL) {
        // shut things down
        nextPacketTick = MaxTick;
        nextTransitionTick = MaxTick;
        deschedule(updateEvent);
        return DrainState::Drained;
    } else {
        return DrainState::Draining;
    }
}

void
TrafficGen::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing TrafficGen\n");

    // save ticks of the graph event if it is scheduled
    Tick nextEvent = updateEvent.scheduled() ? updateEvent.when() : 0;

    DPRINTF(TrafficGen, "Saving nextEvent=%llu\n", nextEvent);

    SERIALIZE_SCALAR(nextEvent);

    SERIALIZE_SCALAR(nextTransitionTick);

    SERIALIZE_SCALAR(nextPacketTick);

    SERIALIZE_SCALAR(currState);
}

void
TrafficGen::unserialize(CheckpointIn &cp)
{
    // restore scheduled events
    Tick nextEvent;
    UNSERIALIZE_SCALAR(nextEvent);
    if (nextEvent != 0) {
        schedule(updateEvent, nextEvent);
    }

    UNSERIALIZE_SCALAR(nextTransitionTick);

    UNSERIALIZE_SCALAR(nextPacketTick);

    // @todo In the case of a stateful generator state such as the
    // trace player we would also have to restore the position in the
    // trace playback and the tick offset
    UNSERIALIZE_SCALAR(currState);
}

void
TrafficGen::update()
{
    // shift our progress-tracking event forward
    reschedule(noProgressEvent, curTick() + progressCheck, true);

    // if we have reached the time for the next state transition, then
    // perform the transition
    if (curTick() >= nextTransitionTick) {
        transition();
    } else {
        assert(curTick() >= nextPacketTick);
        // get the next packet and try to send it
        PacketPtr pkt = states[currState]->getNextPacket();

        // suppress packets that are not destined for a memory, such as
        // device accesses that could be part of a trace
        if (system->isMemAddr(pkt->getAddr())) {
            numPackets++;
            if (!port.sendTimingReq(pkt)) {
                retryPkt = pkt;
                retryPktTick = curTick();
            }
        } else {
            DPRINTF(TrafficGen, "Suppressed packet %s 0x%x\n",
                    pkt->cmdString(), pkt->getAddr());

            ++numSuppressed;
            if (numSuppressed % 10000)
                warn("%s suppressed %d packets with non-memory addresses\n",
                     name(), numSuppressed);

            delete pkt->req;
            delete pkt;
            pkt = nullptr;
        }
    }

    // if we are waiting for a retry, do not schedule any further
    // events, in the case of a transition or a successful send, go
    // ahead and determine when the next update should take place
    if (retryPkt == NULL) {
        // schedule next update event based on either the next execute
        // tick or the next transition, which ever comes first
        nextPacketTick = states[currState]->nextPacketTick(elasticReq, 0);
        Tick nextEventTick = std::min(nextPacketTick, nextTransitionTick);
        DPRINTF(TrafficGen, "Next event scheduled at %lld\n", nextEventTick);
        schedule(updateEvent, nextEventTick);
    }
}

std::string
TrafficGen::resolveFile(const std::string &name)
{
    // Do nothing for empty and absolute file names
    if (name.empty() || name[0] == '/')
        return name;

    char *config_path = strdup(configFile.c_str());
    char *config_dir = dirname(config_path);
    const std::string config_rel = csprintf("%s/%s", config_dir, name);
    free(config_path);

    // Check the path relative to the config file first
    if (access(config_rel.c_str(), R_OK) == 0)
        return config_rel;

    // Fall back to the old behavior and search relative to the
    // current working directory.
    return name;
}

void
TrafficGen::parseConfig()
{
    // keep track of the transitions parsed to create the matrix when
    // done
    vector<Transition> transitions;

    // open input file
    ifstream infile;
    infile.open(configFile.c_str(), ifstream::in);
    if (!infile.is_open()) {
        fatal("Traffic generator %s config file not found at %s\n",
              name(), configFile);
    }

    bool init_state_set = false;

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
                    traceFile = resolveFile(traceFile);

                    states[id] = new TraceGen(name(), masterID, duration,
                                              traceFile, addrOffset);
                    DPRINTF(TrafficGen, "State: %d TraceGen\n", id);
                } else if (mode == "IDLE") {
                    states[id] = new IdleGen(name(), masterID, duration);
                    DPRINTF(TrafficGen, "State: %d IdleGen\n", id);
                } else if (mode == "LINEAR" || mode == "RANDOM" ||
                           mode == "DRAM"   || mode == "DRAM_ROTATE") {
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


                    if (blocksize > system->cacheLineSize())
                        fatal("TrafficGen %s block size (%d) is larger than "
                              "cache line size (%d)\n", name(),
                              blocksize, system->cacheLineSize());

                    if (read_percent > 100)
                        fatal("%s cannot have more than 100% reads", name());

                    if (min_period > max_period)
                        fatal("%s cannot have min_period > max_period", name());

                    if (mode == "LINEAR") {
                        states[id] = new LinearGen(name(), masterID,
                                                   duration, start_addr,
                                                   end_addr, blocksize,
                                                   min_period, max_period,
                                                   read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d LinearGen\n", id);
                    } else if (mode == "RANDOM") {
                        states[id] = new RandomGen(name(), masterID,
                                                   duration, start_addr,
                                                   end_addr, blocksize,
                                                   min_period, max_period,
                                                   read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d RandomGen\n", id);
                    } else if (mode == "DRAM" || mode == "DRAM_ROTATE") {
                        // stride size (bytes) of the request for achieving
                        // required hit length
                        unsigned int stride_size;
                        unsigned int page_size;
                        unsigned int nbr_of_banks_DRAM;
                        unsigned int nbr_of_banks_util;
                        unsigned int addr_mapping;
                        unsigned int nbr_of_ranks;

                        is >> stride_size >> page_size >> nbr_of_banks_DRAM >>
                            nbr_of_banks_util >> addr_mapping >>
                            nbr_of_ranks;

                        if (stride_size > page_size)
                            warn("DRAM generator stride size (%d) is greater "
                                 "than page size (%d)  of the memory\n",
                                 blocksize, page_size);

                        if (nbr_of_banks_util > nbr_of_banks_DRAM)
                            fatal("Attempting to use more banks (%d) than "
                                  "what is available (%d)\n",
                                  nbr_of_banks_util, nbr_of_banks_DRAM);

                        // count the number of sequential packets to
                        // generate
                        unsigned int num_seq_pkts = 1;

                        if (stride_size > blocksize) {
                            num_seq_pkts = divCeil(stride_size, blocksize);
                            DPRINTF(TrafficGen, "stride size: %d "
                                    "block size: %d, num_seq_pkts: %d\n",
                                    stride_size, blocksize, num_seq_pkts);
                        }

                        if (mode == "DRAM") {
                            states[id] = new DramGen(name(), masterID,
                                                     duration, start_addr,
                                                     end_addr, blocksize,
                                                     min_period, max_period,
                                                     read_percent, data_limit,
                                                     num_seq_pkts, page_size,
                                                     nbr_of_banks_DRAM,
                                                     nbr_of_banks_util,
                                                     addr_mapping,
                                                     nbr_of_ranks);
                            DPRINTF(TrafficGen, "State: %d DramGen\n", id);
                        } else {
                            // Will rotate to the next rank after rotating
                            // through all banks, for each command type.
                            // In the 50% read case, series will be issued
                            // for both RD & WR before the rank in incremented
                            unsigned int max_seq_count_per_rank =
                                (read_percent == 50) ? nbr_of_banks_util * 2
                                                     : nbr_of_banks_util;

                            states[id] = new DramRotGen(name(), masterID,
                                                     duration, start_addr,
                                                     end_addr, blocksize,
                                                     min_period, max_period,
                                                     read_percent, data_limit,
                                                     num_seq_pkts, page_size,
                                                     nbr_of_banks_DRAM,
                                                     nbr_of_banks_util,
                                                     addr_mapping,
                                                     nbr_of_ranks,
                                                     max_seq_count_per_rank);
                            DPRINTF(TrafficGen, "State: %d DramRotGen\n", id);
                        }
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

                init_state_set = true;

                DPRINTF(TrafficGen, "Initial state: %d\n", currState);
            }
        }
    }

    if (!init_state_set)
        fatal("%s: initial state not specified (add 'INIT <id>' line "
              "to the config file)\n", name());

    // resize and populate state transition matrix
    transitionMatrix.resize(states.size());
    for (size_t i = 0; i < states.size(); i++) {
        transitionMatrix[i].resize(states.size());
    }

    for (vector<Transition>::iterator t = transitions.begin();
         t != transitions.end(); ++t) {
        transitionMatrix[t->from][t->to] = t->p;
    }

    // ensure the egress edges do not have a probability larger than
    // one
    for (size_t i = 0; i < states.size(); i++) {
        double sum = 0;
        for (size_t j = 0; j < states.size(); j++) {
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
TrafficGen::transition()
{
    // exit the current state
    states[currState]->exit();

    // determine next state
    double p = random_mt.random<double>();
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
TrafficGen::enterState(uint32_t newState)
{
    DPRINTF(TrafficGen, "Transition to state %d\n", newState);

    currState = newState;
    // we could have been delayed and not transitioned on the exact
    // tick when we were supposed to (due to back pressure when
    // sending a packet)
    nextTransitionTick = curTick() + states[currState]->duration;
    states[currState]->enter();
}

void
TrafficGen::recvReqRetry()
{
    assert(retryPkt != NULL);

    DPRINTF(TrafficGen, "Received retry\n");
    numRetries++;
    // attempt to send the packet, and if we are successful start up
    // the machinery again
    if (port.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
        // remember how much delay was incurred due to back-pressure
        // when sending the request, we also use this to derive
        // the tick for the next packet
        Tick delay = curTick() - retryPktTick;
        retryPktTick = 0;
        retryTicks += delay;

        if (drainState() != DrainState::Draining) {
            // packet is sent, so find out when the next one is due
            nextPacketTick = states[currState]->nextPacketTick(elasticReq,
                                                               delay);
            Tick nextEventTick = std::min(nextPacketTick, nextTransitionTick);
            schedule(updateEvent, std::max(curTick(), nextEventTick));
        } else {
            // shut things down
            nextPacketTick = MaxTick;
            nextTransitionTick = MaxTick;
            signalDrainDone();
        }
    }
}

void
TrafficGen::noProgress()
{
    fatal("TrafficGen %s spent %llu ticks without making progress",
          name(), progressCheck);
}

void
TrafficGen::regStats()
{
    ClockedObject::regStats();

    // Initialise all the stats
    using namespace Stats;

    numPackets
        .name(name() + ".numPackets")
        .desc("Number of packets generated");

    numRetries
        .name(name() + ".numRetries")
        .desc("Number of retries");

    retryTicks
        .name(name() + ".retryTicks")
        .desc("Time spent waiting due to back-pressure (ticks)");
}

bool
TrafficGen::TrafficGenPort::recvTimingResp(PacketPtr pkt)
{
    delete pkt->req;
    delete pkt;

    return true;
}
