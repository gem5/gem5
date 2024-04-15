/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 */

/* @file
 * Reference counted class containing ethernet packet data
 */

#ifndef __DEV_NET_ETHERPKT_HH__
#define __DEV_NET_ETHERPKT_HH__

#include <cassert>
#include <iosfwd>
#include <memory>

#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

/*
 * Reference counted class containing ethernet packet data
 */
class EthPacketData
{
  public:
    /**
     * Pointer to packet data will be deleted
     */
    uint8_t *data;

    /**
     * Total size of the allocated data buffer.
     */
    unsigned bufLength;

    /**
     * Amount of space occupied by the payload in the data buffer
     */
    unsigned length;

    /**
     * Effective length, used for modeling timing in the simulator.
     * This could be different from length if the packets are assumed
     * to use a tightly packed or compressed format, but it's not worth
     * the performance/complexity hit to perform that packing or compression
     * in the simulation.
     */
    unsigned simLength;

    EthPacketData() : data(nullptr), bufLength(0), length(0), simLength(0) {}

    explicit EthPacketData(unsigned size)
        : data(new uint8_t[size]), bufLength(size), length(0), simLength(0)
    {}

    ~EthPacketData()
    {
        if (data)
            delete[] data;
    }

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);
};

typedef std::shared_ptr<EthPacketData> EthPacketPtr;

} // namespace gem5

#endif // __DEV_NET_ETHERPKT_HH__
