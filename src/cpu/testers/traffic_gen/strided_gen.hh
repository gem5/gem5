/*
 * Copyright (c) 2012-2013, 2017-2018 ARM Limited
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
 */

/**
 * @file
 * Declaration of the strided generator that generates sequential
 * requests.
 */

#ifndef __CPU_TRAFFIC_GEN_STRIDED_GEN_HH__
#define __CPU_TRAFFIC_GEN_STRIDED_GEN_HH__

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base_gen.hh"
#include "mem/packet.hh"

namespace gem5
{

/**
 * The strided generator generates sequential requests from a
 * start to an end address, with a fixed block size. A
 * fraction of the requests are reads, as determined by the
 * read percent. There is an optional data limit for when to
 * stop generating new requests.
 */
class StridedGen : public StochasticGen
{

  public:

    /**
     * Create a strided address sequence generator. Set
     * min_period == max_period for a fixed inter-transaction
     * time.
     *
     * @param obj SimObject owning this sequence generator
     * @param requestor_id RequestorID related to the memory requests
     * @param cacheline_size cache line size in the system
     * @param duration duration of this state before transitioning
     * @param start_addr Start address
     * @param end_addr End address
     * @param offset The offset to start_addr for generating addresses.
     * First generated address = start_addr + offset.
     * @param block_size Size used for transactions injected
     * @param stride_size The strided size for consecutive requests
     * @param superblock_size Number of bytes to read before taking a stride
     * @param min_period Lower limit of random inter-transaction time
     * @param max_period Upper limit of random inter-transaction time
     * @param read_percent Percent of transactions that are reads
     * @param data_limit Upper limit on how much data to read/write
     */
    StridedGen(SimObject& obj, RequestorID requestor_id,
            Tick duration, Addr cacheline_size,
            Addr start_addr, Addr end_addr, Addr offset,
            Addr block_size, Addr superblock_size, Addr stride_size,
            Tick min_period, Tick max_period,
            uint8_t read_percent, Addr data_limit);

    void enter();

    PacketPtr getNextPacket();

    Tick nextPacketTick(bool elastic, Tick delay) const;

  private:

    Addr offset;
    Addr superblockSize;
    /* The size of the access stride */
    Addr strideSize;

    /** Address of next request */
    Addr nextAddr;

    /**
     * Counter to determine the amount of data
     * manipulated. Used to determine if we should continue
     * generating requests.
     */
    Addr dataManipulated;
};

} // namespace gem5

#endif
