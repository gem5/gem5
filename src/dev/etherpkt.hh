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
 *
 * Authors: Nathan Binkert
 *          Lisa Hsu
 */

/* @file
 * Reference counted class containing ethernet packet data
 */

#ifndef __ETHERPKT_HH__
#define __ETHERPKT_HH__

#include <cassert>
#include <iosfwd>
#include <memory>

#include "base/types.hh"
#include "sim/serialize.hh"

/*
 * Reference counted class containing ethernet packet data
 */
class EthPacketData
{
  public:
    /*
     * Pointer to packet data will be deleted
     */
    uint8_t *data;

    /*
     * Length of the current packet
     */
    unsigned length;

  public:
    EthPacketData()
        : data(NULL), length(0)
    { }

    explicit EthPacketData(unsigned size)
        : data(new uint8_t[size]), length(0)
    { }

    ~EthPacketData() { if (data) delete [] data; }

  public:
    /**
     * This function pulls out the MAC source and destination addresses from
     * the packet data and stores them in the caller specified buffers.
     *
     * @param src_addr The buffer to store the source MAC address.
     * @param dst_addr The buffer to store the destination MAC address.
     * @param length This is an inout parameter. The caller stores in this
     * the size of the address buffers. On return, this will contain the
     * actual address size stored in the buffers. (We assume that source
     * address size is equal to that of the destination address.)
     */
    void packAddress(uint8_t *src_addr, uint8_t *dst_addr, unsigned &length);

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);

    unsigned size() const { return length; }
};

typedef std::shared_ptr<EthPacketData> EthPacketPtr;

#endif // __ETHERPKT_HH__
