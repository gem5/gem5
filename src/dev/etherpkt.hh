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

#include <iosfwd>
#include <memory>
#include <assert.h>

#include "base/refcnt.hh"
#include "sim/host.hh"

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
    int length;

    /*
     * Extra space taken up by the packet in whatever data structure
     * it is in.
     *
     * NOTE: This can only be use by *one* data structure at a time!
     */
    int slack;

  public:
    EthPacketData() : data(NULL), length(0), slack(0) { }
    explicit EthPacketData(size_t size)
        : data(new uint8_t[size]), length(0), slack(0) { }
    EthPacketData(std::auto_ptr<uint8_t> d, int l, int s = 0)
        : data(d.release()), length(l), slack(s) { }
    ~EthPacketData() { if (data) delete [] data; }

  public:
    void serialize(const std::string &base, std::ostream &os);
    void unserialize(const std::string &base, Checkpoint *cp,
                     const std::string &section);
};

typedef RefCountingPtr<EthPacketData> EthPacketPtr;

#endif // __ETHERPKT_HH__
