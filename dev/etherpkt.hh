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

#include "sim/host.hh"
#include "base/refcnt.hh"

#define EADDR_LEN 6

class Checkpoint;

struct pseudo_header
{
    uint32_t src_ip_addr;
    uint32_t dest_ip_addr;
    uint16_t protocol;
    uint16_t len;
};

/** Ethernet header struct for casting purposes */
struct eth_header
{
    uint8_t dest[EADDR_LEN];
    uint8_t src[EADDR_LEN];
    uint16_t type;
};

struct ip_header
{
    uint8_t vers_len;
    uint8_t service_type;
    uint16_t dgram_len;
    uint16_t ID;
    uint16_t flags_frag_offset;
    uint8_t TTL;
    uint8_t protocol;
    uint16_t hdr_chksum;
    uint32_t src_ip_addr;
    uint32_t dest_ip_addr;
    uint8_t *options;
    uint8_t *transport_header;
};

struct tcp_header
{
    uint16_t src_port_num;
    uint16_t dest_port_num;
    uint32_t seq_num;
    uint32_t ack_num;
    uint8_t hdr_len;
    uint8_t flags;
    uint16_t rcv_window;
    uint16_t chksum;
    uint16_t urgent;
    uint8_t *options;
    uint8_t *data;
};

struct udp_header
{
    uint16_t src_port_num;
    uint16_t dest_port_num;
    uint16_t len;
    uint16_t chksum;
    uint8_t *data;
};

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

    ip_header *getIpHdr() { return (ip_header *) (data + 14); }

    void *getTransportHdr() {
        ip_header *ip = getIpHdr();
        return (void *) (ip + (ip->vers_len & 0xf));
    }


    typedef RefCountingPtr<EtherPacket> PacketPtr;

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
};

typedef RefCountingPtr<EtherPacket> PacketPtr;

#endif // __ETHERPKT_HH__
