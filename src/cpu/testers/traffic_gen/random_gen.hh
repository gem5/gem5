/*
 * Copyright (c) 2012-2013, 2017 ARM Limited
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
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Sascha Bischoff
 *          Neha Agarwal
 */

/**
 * @file
 * Declaration of the random generator that randomly selects
 * addresses within a range.
 */

#ifndef __CPU_TRAFFIC_GEN_RANDOM_GEN_HH__
#define __CPU_TRAFFIC_GEN_RANDOM_GEN_HH__

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base_gen.hh"
#include "mem/packet.hh"
#include "proto/protoio.hh"

/**
 * The random generator is similar to the linear one, but does
 * not generate sequential addresses. Instead it randomly
 * picks an address in the range, aligned to the block size.
 */
class RandomGen : public BaseGen
{

  public:

    /**
     * Create a random address sequence generator. Set
     * min_period == max_period for a fixed inter-transaction
     * time.
     *
     * @param _name Name to use for status and debug
     * @param master_id MasterID set on each request
     * @param _duration duration of this state before transitioning
     * @param start_addr Start address
     * @param end_addr End address
     * @param _blocksize Size used for transactions injected
     * @param min_period Lower limit of random inter-transaction time
     * @param max_period Upper limit of random inter-transaction time
     * @param read_percent Percent of transactions that are reads
     * @param data_limit Upper limit on how much data to read/write
     */
    RandomGen(const std::string& _name, MasterID master_id, Tick _duration,
              Addr start_addr, Addr end_addr, Addr _blocksize,
              Tick min_period, Tick max_period,
              uint8_t read_percent, Addr data_limit)
        : BaseGen(_name, master_id, _duration),
          startAddr(start_addr), endAddr(end_addr),
          blocksize(_blocksize), minPeriod(min_period),
          maxPeriod(max_period), readPercent(read_percent),
          dataLimit(data_limit), dataManipulated(0)
    { }

    void enter();

    PacketPtr getNextPacket();

    Tick nextPacketTick(bool elastic, Tick delay) const;

  protected:

    /** Start of address range */
    const Addr startAddr;

    /** End of address range */
    const Addr endAddr;

    /** Block size */
    const Addr blocksize;

    /** Request generation period */
    const Tick minPeriod;
    const Tick maxPeriod;

    /**
     * Percent of generated transactions that should be reads
     */
    const uint8_t readPercent;

    /** Maximum amount of data to manipulate */
    const Addr dataLimit;

    /**
     * Counter to determine the amount of data
     * manipulated. Used to determine if we should continue
     * generating requests.
     */
    Addr dataManipulated;
};

#endif
