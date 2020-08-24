/*
 * Copyright (c) 2012-2013, 2016-2018 ARM Limited
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

#ifndef __CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__
#define __CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__

#include <unordered_map>

#include "cpu/testers/traffic_gen/base.hh"

struct TrafficGenParams;

/**
 * The traffic generator is a module that generates stimuli for
 * the memory system, based on a collection of simple behaviours that
 * are either probabilistic or based on traces. It can be used stand
 * alone for creating test cases for interconnect and memory
 * controllers, or function as a black-box replacement for system
 * components that are not yet modelled in detail, e.g. a video engine
 * or baseband subsystem in an SoC.
 *
 * The traffic generator has a single request port that is used to send
 * requests, independent of the specific behaviour. The behaviour of
 * the traffic generator is specified in a configuration file, and this
 * file describes a state transition graph where each state is a
 * specific generator behaviour. Examples include idling, generating
 * linear address sequences, random sequences and replay of captured
 * traces. By describing these behaviours as states, it is straight
 * forward to create very complex behaviours, simply by arranging them
 * in graphs. The graph transitions can also be annotated with
 * probabilities, effectively making it a Markov Chain.
 */
class TrafficGen : public BaseTrafficGen
{
  private: // Params
    /**
     * The config file to parse.
     */
    const std::string configFile;

  private:
    /**
     * Resolve a file path in the configuration file.
     *
     * This method resolves a relative path to a file that has been
     * referenced in the configuration file. It first tries to resolve
     * the file relative to the configuration file's path. If that
     * fails, it falls back to constructing a path relative to the
     * current working directory.
     *
     * Absolute paths are returned unmodified.
     *
     * @param name Path to resolve
     */
    std::string resolveFile(const std::string &name);

     /**
      * Parse the config file and build the state map and
      * transition matrix.
      */
    void parseConfig();

    /**
     * Use the transition matrix to find the next state index.
     */
    size_t nextState();

    /** Struct to represent a probabilistic transition during parsing. */
    struct Transition {
        uint32_t from;
        uint32_t to;
        double p;
    };

    /** State transition matrix */
    std::vector<std::vector<double> > transitionMatrix;

    /** Index of the current state */
    uint32_t currState;

    /** Map of generator states */
    std::unordered_map<uint32_t, std::shared_ptr<BaseGen>> states;

  protected: // BaseTrafficGen
    std::shared_ptr<BaseGen> nextGenerator() override;

  public:

    TrafficGen(const TrafficGenParams* p);

    ~TrafficGen() {}

    void init() override;
    void initState() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

};

#endif //__CPU_TRAFFIC_GEN_TRAFFIC_GEN_HH__
