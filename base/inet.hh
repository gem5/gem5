/*
 * Copyright (c) 2002-2003 The Regents of The University of Michigan
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

#ifndef __BASE_INET_HH__
#define __BASE_INET_HH__

#include <string>

#include "dnet/os.h"

#include "dnet/eth.h"
#include "dnet/ip.h"
#include "dnet/ip6.h"
#include "dnet/addr.h"
#include "dnet/arp.h"
#include "dnet/icmp.h"
#include "dnet/tcp.h"
#include "dnet/udp.h"

#include "dnet/intf.h"
#include "dnet/route.h"
#include "dnet/fw.h"

#include "dnet/blob.h"
#include "dnet/rand.h"

#include "sim/host.hh"

std::string eaddr_string(const uint8_t a[6]);

struct EthHdr;
struct IpHdr;
struct TcpHdr;
struct UdpHdr;

struct EthHdr : protected eth_hdr
{
    uint16_t type() const { return ntohs(eth_type); }

    const IpHdr *ip() const
    { return type() == ETH_TYPE_IP ? (const IpHdr *)payload() : 0; }

    IpHdr *ip()
    { return type() == ETH_TYPE_IP ? (IpHdr *)payload() : 0; }

    bool unicast() { return eth_dst.data[0] == 0x00; }
    bool multicast() { return eth_dst.data[0] == 0x01; }
    bool broadcast() { return eth_dst.data[0] == 0xff; }

    int size() const { return sizeof(EthHdr); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

struct IpHdr : protected ip_hdr
{
    uint8_t  version() const { return ip_v; }
    uint8_t  hlen() const { return ip_hl * 4; }
    uint8_t  tos() const { return ip_tos; }
    uint16_t len() const { return ntohs(ip_len); }
    uint16_t id() const { return ntohs(ip_id); }
    uint16_t frag_flags() const { return ntohs(ip_off) >> 13; }
    uint16_t frag_off() const { return ntohs(ip_off) & 0x1fff; }
    uint8_t  ttl() const { return ip_ttl; }
    uint8_t  proto() const { return ip_p; }
    uint16_t sum() const { return ip_sum; }
    uint32_t src() const { return ntohl(ip_src); }
    uint32_t dst() const { return ntohl(ip_dst); }

    void sum(uint16_t sum) { ip_sum = sum; }

    uint16_t ip_cksum() const;
    uint16_t tu_cksum() const;

    const TcpHdr *tcp() const
    { return proto() == IP_PROTO_TCP ? (const TcpHdr *)payload() : 0; }
    const UdpHdr *udp() const
    { return proto() == IP_PROTO_UDP ? (const UdpHdr *)payload() : 0; }

    TcpHdr *tcp()
    { return proto() == IP_PROTO_TCP ? (TcpHdr *)payload() : 0; }
    UdpHdr *udp()
    { return proto() == IP_PROTO_UDP ? (UdpHdr *)payload() : 0; }


    int size() const { return hlen(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

struct TcpHdr : protected tcp_hdr
{
    uint16_t sport() const { return ntohs(th_sport); }
    uint16_t dport() const { return ntohs(th_dport); }
    uint32_t seq() const { return ntohl(th_seq); }
    uint32_t ack() const { return ntohl(th_ack); }
    uint8_t  off() const { return th_off; }
    uint8_t  flags() const { return th_flags & 0x3f; }
    uint16_t win() const { return ntohs(th_win); }
    uint16_t sum() const { return th_sum; }
    uint16_t urp() const { return ntohs(th_urp); }

    void sum(uint16_t sum) { th_sum = sum; }

    int size() const { return off(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

struct UdpHdr : protected udp_hdr
{
    uint16_t sport() const { return ntohs(uh_sport); }
    uint16_t dport() const { return ntohs(uh_dport); }
    uint16_t len() const { return ntohs(uh_ulen); }
    uint16_t sum() const { return ntohs(uh_sum); }

    void sum(uint16_t sum) { uh_sum = htons(sum); }

    int size() const { return sizeof(UdpHdr); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

#endif // __BASE_INET_HH__
