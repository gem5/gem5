/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#ifndef __ETHERPKT_HH__
#define __ETHERPKT_HH__

#include <iosfwd>
#include <memory>
#include <assert.h>

#include "sim/host.hh"
#include "base/refcnt.hh"
#include "base/inet_hdrs.hh"

class Checkpoint;
/*
 * Reference counted class containing ethernet packet data
 */
class EtherPacket : public RefCounted
{
  public:
    uint8_t *data;
    int length;

  public:
    EtherPacket() : data(NULL), length(0) {}
    EtherPacket(std::auto_ptr<uint8_t> d, int l)
        : data(d.release()), length(l) {}
    ~EtherPacket() { if (data) delete [] data; }

  public:
    bool IsUnicast() { return data[0] == 0x00; }
    bool IsMulticast() { return data[0] == 0x01; }
    bool IsBroadcast() { return data[0] == 0xff; }

    bool isIpPkt() {
        eth_header *eth = (eth_header *) data;
        return (eth->type == 0x8);
    }
    bool isTcpPkt(ip_header *ip) {
        return (ip->protocol == 0x6);
    }
    bool isTcpPkt() {
        ip_header *ip = getIpHdr();
        return (ip->protocol == 0x6);
    }
    bool isUdpPkt(ip_header *ip) {
        return (ip->protocol == 17);
    }
    bool isUdpPkt() {
        ip_header *ip = getIpHdr();
        return (ip->protocol == 17);
    }

    ip_header *getIpHdr() {
        assert(isIpPkt());
        return (ip_header *) (data + sizeof(eth_header));
    }

    tcp_header *getTcpHdr(ip_header *ip) {
        assert(isTcpPkt(ip));
        return (tcp_header *) ((uint8_t *) ip + (ip->vers_len & 0xf)*4);
    }

    udp_header *getUdpHdr(ip_header *ip) {
        assert(isUdpPkt(ip));
        return (udp_header *) ((uint8_t *) ip + (ip->vers_len & 0xf)*4);
    }
    typedef RefCountingPtr<EtherPacket> PacketPtr;

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
};

typedef RefCountingPtr<EtherPacket> PacketPtr;

#endif // __ETHERPKT_HH__
