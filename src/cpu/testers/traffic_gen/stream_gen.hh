/*
 * Copyright (c) 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed here under.  You may use the software subject to the license
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
 * Authors: Giacomo Travaglini
 */

/**
 * @file
 * Declaration of the Stream generator for issuing memory requests
 * with variable/fixed stream and substream IDs.
 */

#ifndef __CPU_TRAFFIC_GEN_STREAM_GEN_HH__
#define __CPU_TRAFFIC_GEN_STREAM_GEN_HH__

#include "params/BaseTrafficGen.hh"

class StreamGen
{
  protected:
    StreamGen(const BaseTrafficGenParams *p)
      : streamIds(p->sids), substreamIds(p->ssids)
    {
        // A non empty vector of StreamIDs must be provided.
        // SubstreamIDs are not mandatory hence having an empty
        // vector means that they are not used and no configuration
        // error must be thrown
        fatal_if(streamIds.empty(),
            "Must provide a vector of StreamIDs");
    }

  public:
    virtual ~StreamGen() {};

    virtual uint32_t pickStreamID() = 0;
    virtual uint32_t pickSubStreamID() = 0;

    /**
     * Factory method for constructing a Stream generator.
     * The Stream generator type is selected by the
     * StreamGenType enum parameter.
     *
     * @params p pointer to BaseTrafficGenParams struct where
     *           the stream generator type is stored.
     * @return a pointer to the newly alocated StremGen
     */
    static StreamGen* create(const BaseTrafficGenParams *p);

    /**
     * Returns true if the substreamID generation is valid
     * and hence should be taken into account.
     * It is valid if the set of substreamIDs passed as a
     * parameter to the TrafficGenerator is a non empty list.
     *
     * @return true if ssid is valid, false otherwise
     */
    bool ssidValid() const { return !substreamIds.empty(); }

  protected:
    /**
     * Store preset Stream and Substream IDs to use for requests
     * This is the set of available streamIDs the generator can
     * pick. The actual ID being picked for a specific memory
     * request is selected by the pickStreamID and pickSubStreamID
     * methods.
     */
    std::vector<uint32_t> streamIds;
    std::vector<uint32_t> substreamIds;
};

class FixedStreamGen : public StreamGen
{
  public:
    FixedStreamGen(const BaseTrafficGenParams *p)
      : StreamGen(p)
    {
        // For a fixed stream generator only one sid must be provided. The
        // ssid can have either 0 (not used) or 1 value.
        fatal_if(streamIds.size() != 1 || substreamIds.size() > 1,
                 "Invalid sids/ssids configuration");
    }

    uint32_t pickStreamID() override
    { return streamIds[0]; }

    uint32_t pickSubStreamID() override
    { return substreamIds[0]; }
};

class RandomStreamGen : public StreamGen
{
  public:
    RandomStreamGen(const BaseTrafficGenParams *p)
      : StreamGen(p)
    {}

    uint32_t pickStreamID() override
    { return randomPick(streamIds); }

    uint32_t pickSubStreamID() override
    { return randomPick(substreamIds); }

  protected:
    /** Function to pick one of the preset Stream or Substream ID */
    uint32_t randomPick(const std::vector<uint32_t> &svec);
};

#endif // __CPU_TRAFFIC_GEN_STREAM_GEN_HH__
