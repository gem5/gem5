/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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
 *          Gabe Black
 */

#include <cstddef>
#include <cstdio>
#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "base/inet.hh"
#include "base/types.hh"

using namespace std;
namespace Net {

EthAddr::EthAddr()
{
    memset(data, 0, ETH_ADDR_LEN);
}

EthAddr::EthAddr(const uint8_t ea[ETH_ADDR_LEN])
{
    *data = *ea;
}

EthAddr::EthAddr(const eth_addr &ea)
{
    *data = *ea.data;
}

EthAddr::EthAddr(const std::string &addr)
{
    parse(addr);
}

const EthAddr &
EthAddr::operator=(const eth_addr &ea)
{
    *data = *ea.data;
    return *this;
}

const EthAddr &
EthAddr::operator=(const std::string &addr)
{
    parse(addr);
    return *this;
}

void
EthAddr::parse(const std::string &addr)
{
    // the hack below is to make sure that ETH_ADDR_LEN is 6 otherwise
    // the sscanf function won't work.
    int bytes[ETH_ADDR_LEN == 6 ? ETH_ADDR_LEN : -1];
    if (sscanf(addr.c_str(), "%x:%x:%x:%x:%x:%x", &bytes[0], &bytes[1],
               &bytes[2], &bytes[3], &bytes[4], &bytes[5]) != ETH_ADDR_LEN) {
        memset(data, 0xff, ETH_ADDR_LEN);
        return;
    }

    for (int i = 0; i < ETH_ADDR_LEN; ++i) {
        if (bytes[i] & ~0xff) {
            memset(data, 0xff, ETH_ADDR_LEN);
            return;
        }

        data[i] = bytes[i];
    }
}

string
EthAddr::string() const
{
    stringstream stream;
    stream << *this;
    return stream.str();
}

bool
operator==(const EthAddr &left, const EthAddr &right)
{
    return memcmp(left.bytes(), right.bytes(), ETH_ADDR_LEN);
}

ostream &
operator<<(ostream &stream, const EthAddr &ea)
{
    const uint8_t *a = ea.addr();
    ccprintf(stream, "%x:%x:%x:%x:%x:%x", a[0], a[1], a[2], a[3], a[4], a[5]);
    return stream;
}

string
IpAddress::string() const
{
    stringstream stream;
    stream << *this;
    return stream.str();
}

bool
operator==(const IpAddress &left, const IpAddress &right)
{
    return left.ip() == right.ip();
}

ostream &
operator<<(ostream &stream, const IpAddress &ia)
{
    uint32_t ip = ia.ip();
    ccprintf(stream, "%x.%x.%x.%x",
            (uint8_t)(ip >> 24), (uint8_t)(ip >> 16),
            (uint8_t)(ip >> 8),  (uint8_t)(ip >> 0));
    return stream;
}

string
IpNetmask::string() const
{
    stringstream stream;
    stream << *this;
    return stream.str();
}

bool
operator==(const IpNetmask &left, const IpNetmask &right)
{
    return (left.ip() == right.ip()) &&
        (left.netmask() == right.netmask());
}

ostream &
operator<<(ostream &stream, const IpNetmask &in)
{
    ccprintf(stream, "%s/%d", (const IpAddress &)in, in.netmask());
    return stream;
}

string
IpWithPort::string() const
{
    stringstream stream;
    stream << *this;
    return stream.str();
}

bool
operator==(const IpWithPort &left, const IpWithPort &right)
{
    return (left.ip() == right.ip()) && (left.port() == right.port());
}

ostream &
operator<<(ostream &stream, const IpWithPort &iwp)
{
    ccprintf(stream, "%s:%d", (const IpAddress &)iwp, iwp.port());
    return stream;
}

uint16_t
cksum(const IpPtr &ptr)
{
    int sum = ip_cksum_add(ptr->bytes(), ptr->hlen(), 0);
    return ip_cksum_carry(sum);
}

uint16_t
__tu_cksum(const IpPtr &ip)
{
    int tcplen = ip->len() - ip->hlen();
    int sum = ip_cksum_add(ip->payload(), tcplen, 0);
    sum = ip_cksum_add(&ip->ip_src, 8, sum); // source and destination
    sum += htons(ip->ip_p + tcplen);
    return ip_cksum_carry(sum);
}

uint16_t
cksum(const TcpPtr &tcp)
{ return __tu_cksum(IpPtr(tcp.packet())); }

uint16_t
cksum(const UdpPtr &udp)
{ return __tu_cksum(IpPtr(udp.packet())); }

bool
IpHdr::options(vector<const IpOpt *> &vec) const
{
    vec.clear();

    const uint8_t *data = bytes() + sizeof(struct ip_hdr);
    int all = hlen() - sizeof(struct ip_hdr);
    while (all > 0) {
        const IpOpt *opt = (const IpOpt *)data;
        int len = opt->len();
        if (all < len)
            return false;

        vec.push_back(opt);
        all -= len;
        data += len;
    }

    return true;
}

bool
TcpHdr::options(vector<const TcpOpt *> &vec) const
{
    vec.clear();

    const uint8_t *data = bytes() + sizeof(struct tcp_hdr);
    int all = off() - sizeof(struct tcp_hdr);
    while (all > 0) {
        const TcpOpt *opt = (const TcpOpt *)data;
        int len = opt->len();
        if (all < len)
            return false;

        vec.push_back(opt);
        all -= len;
        data += len;
    }

    return true;
}

bool
TcpOpt::sack(vector<SackRange> &vec) const
{
    vec.clear();

    const uint8_t *data = bytes() + sizeof(struct tcp_hdr);
    int all = len() - offsetof(tcp_opt, opt_data.sack);
    while (all > 0) {
        const uint16_t *sack = (const uint16_t *)data;
        int len = sizeof(uint16_t) * 2;
        if (all < len) {
            vec.clear();
            return false;
        }

        vec.push_back(RangeIn(ntohs(sack[0]), ntohs(sack[1])));
        all -= len;
        data += len;
    }

    return false;
}

int 
hsplit(const EthPacketPtr &ptr)
{
    int split_point = 0;

    IpPtr ip(ptr);
    if (ip) {
        split_point = ip.pstart();

        TcpPtr tcp(ip);
        if (tcp)
            split_point = tcp.pstart();

        UdpPtr udp(ip);
        if (udp)
            split_point = udp.pstart();
    }
    return split_point;
}


} // namespace Net
