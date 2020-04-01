/*
 * Copyright (c) 2012-2013, 2016-2020 ARM Limited
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
#include "cpu/testers/traffic_gen/traffic_gen.hh"

#include <libgen.h>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "base/intmath.hh"
#include "base/random.hh"
#include "debug/TrafficGen.hh"
#include "params/TrafficGen.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace std;

TrafficGen::TrafficGen(const TrafficGenParams* p)
    : BaseTrafficGen(p),
      configFile(p->config_file),
      currState(0)
{
}

TrafficGen*
TrafficGenParams::create()
{
    return new TrafficGen(this);
}

void
TrafficGen::init()
{
    BaseTrafficGen::init();

    parseConfig();
}

void
TrafficGen::initState()
{
    BaseTrafficGen::initState();

    // when not restoring from a checkpoint, make sure we kick things off
    if (system->isTimingMode()) {
        DPRINTF(TrafficGen, "Timing mode, activating request generator\n");
        start();
    } else {
        DPRINTF(TrafficGen,
                "Traffic generator is only active in timing mode\n");
    }
}

void
TrafficGen::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(currState);

    BaseTrafficGen::serialize(cp);
}

void
TrafficGen::unserialize(CheckpointIn &cp)
{
    // @todo In the case of a stateful generator state such as the
    // trace player we would also have to restore the position in the
    // trace playback and the tick offset
    UNSERIALIZE_SCALAR(currState);

    BaseTrafficGen::unserialize(cp);
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

                    states[id] = createTrace(duration, traceFile, addrOffset);
                    DPRINTF(TrafficGen, "State: %d TraceGen\n", id);
                } else if (mode == "IDLE") {
                    states[id] = createIdle(duration);
                    DPRINTF(TrafficGen, "State: %d IdleGen\n", id);
                } else if (mode == "EXIT") {
                    states[id] = createExit(duration);
                    DPRINTF(TrafficGen, "State: %d ExitGen\n", id);
                } else if (mode == "LINEAR" || mode == "RANDOM" ||
                           mode == "DRAM"   || mode == "DRAM_ROTATE" ||
                           mode == "NVM") {
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


                    if (mode == "LINEAR") {
                        states[id] = createLinear(duration, start_addr,
                                                  end_addr, blocksize,
                                                  min_period, max_period,
                                                  read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d LinearGen\n", id);
                    } else if (mode == "RANDOM") {
                        states[id] = createRandom(duration, start_addr,
                                                  end_addr, blocksize,
                                                  min_period, max_period,
                                                  read_percent, data_limit);
                        DPRINTF(TrafficGen, "State: %d RandomGen\n", id);
                    } else if (mode == "DRAM" || mode == "DRAM_ROTATE" ||
                               mode == "NVM") {
                        // stride size (bytes) of the request for achieving
                        // required hit length
                        unsigned int stride_size;
                        unsigned int page_size;
                        unsigned int nbr_of_banks;
                        unsigned int nbr_of_banks_util;
                        unsigned _addr_mapping;
                        unsigned int nbr_of_ranks;

                        is >> stride_size >> page_size >> nbr_of_banks >>
                            nbr_of_banks_util >> _addr_mapping >>
                            nbr_of_ranks;
                        Enums::AddrMap addr_mapping =
                            static_cast<Enums::AddrMap>(_addr_mapping);

                        if (stride_size > page_size)
                            warn("Memory generator stride size (%d) is greater"
                                 " than page size (%d)  of the memory\n",
                                 blocksize, page_size);

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
                            states[id] = createDram(duration, start_addr,
                                                    end_addr, blocksize,
                                                    min_period, max_period,
                                                    read_percent, data_limit,
                                                    num_seq_pkts, page_size,
                                                    nbr_of_banks,
                                                    nbr_of_banks_util,
                                                    addr_mapping,
                                                    nbr_of_ranks);
                            DPRINTF(TrafficGen, "State: %d DramGen\n", id);
                        } else if (mode == "DRAM_ROTATE") {
                            // Will rotate to the next rank after rotating
                            // through all banks, for each command type.
                            // In the 50% read case, series will be issued
                            // for both RD & WR before the rank in incremented
                            unsigned int max_seq_count_per_rank =
                                (read_percent == 50) ? nbr_of_banks_util * 2
                                                     : nbr_of_banks_util;

                            states[id] = createDramRot(duration, start_addr,
                                                       end_addr, blocksize,
                                                       min_period, max_period,
                                                       read_percent,
                                                       data_limit,
                                                       num_seq_pkts, page_size,
                                                       nbr_of_banks,
                                                       nbr_of_banks_util,
                                                       addr_mapping,
                                                       nbr_of_ranks,
                                                       max_seq_count_per_rank);
                            DPRINTF(TrafficGen, "State: %d DramRotGen\n", id);
                        } else {
                            states[id] = createNvm(duration, start_addr,
                                                   end_addr, blocksize,
                                                   min_period, max_period,
                                                   read_percent, data_limit,
                                                   num_seq_pkts, page_size,
                                                   nbr_of_banks,
                                                   nbr_of_banks_util,
                                                   addr_mapping,
                                                   nbr_of_ranks);
                            DPRINTF(TrafficGen, "State: %d NvmGen\n", id);
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

size_t
TrafficGen::nextState()
{
    double p = random_mt.random<double>();
    assert(currState < transitionMatrix.size());
    double cumulative = 0.0;
    size_t i = 0;
    do {
        cumulative += transitionMatrix[currState][i];
        ++i;
    } while (cumulative < p && i < transitionMatrix[currState].size());

    return i - 1;
}

std::shared_ptr<BaseGen>
TrafficGen::nextGenerator()
{
    // Return the initial state if there isn't an active generator,
    // otherwise perform a state transition.
    if (activeGenerator)
        currState = nextState();

    DPRINTF(TrafficGen, "Transition to state %d\n", currState);
    return states[currState];
}
