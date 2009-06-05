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

#include "base/refcnt.hh"
#include "base/types.hh"

/*
 * Reference counted class containing ethernet packet data
 */
class Checkpoint;
class EthPacketData : public RefCounted
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

    EthPacketData(std::auto_ptr<uint8_t> d, int l)
        : data(d.release()), length(l)
    { }

    ~EthPacketData() { if (data) delete [] data; }

  public:
    void serialize(const std::string &base, std::ostream &os);
    void unserialize(const std::string &base, Checkpoint *cp,
                     const std::string &section);
};

typedef RefCountingPtr<EthPacketData> EthPacketPtr;

#endif // __ETHERPKT_HH__
