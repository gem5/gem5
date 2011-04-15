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
 *          Steve Reinhardt
 *          Gabe Black
 */

#ifndef __BASE_INET_HH__
#define __BASE_INET_HH__

#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

#include "base/range.hh"
#include "base/types.hh"
#include "dev/etherpkt.hh"
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

namespace Net {

/*
 * Ethernet Stuff
 */
struct EthAddr : protected eth_addr
{
  protected:
    void parse(const std::string &addr);

  public:
    EthAddr();
    EthAddr(const uint8_t ea[ETH_ADDR_LEN]);
    EthAddr(const eth_addr &ea);
    EthAddr(const std::string &addr);
    const EthAddr &operator=(const eth_addr &ea);
    const EthAddr &operator=(const std::string &addr);

    int size() const { return sizeof(eth_addr); }

    const uint8_t *bytes() const { return &data[0]; }
    uint8_t *bytes() { return &data[0]; }

    const uint8_t *addr() const { return &data[0]; }
    bool unicast() const { return data[0] == 0x00; }
    bool multicast() const { return data[0] == 0x01; }
    bool broadcast() const { return data[0] == 0xff; }
    std::string string() const;

    operator uint64_t() const
    {
        uint64_t reg = 0;
        reg |= ((uint64_t)data[0]) << 40;
        reg |= ((uint64_t)data[1]) << 32;
        reg |= ((uint64_t)data[2]) << 24;
        reg |= ((uint64_t)data[3]) << 16;
        reg |= ((uint64_t)data[4]) << 8;
        reg |= ((uint64_t)data[5]) << 0;
        return reg;
    }

};

std::ostream &operator<<(std::ostream &stream, const EthAddr &ea);
bool operator==(const EthAddr &left, const EthAddr &right);

struct EthHdr : public eth_hdr
{
    uint16_t type() const { return ntohs(eth_type); }
    const EthAddr &src() const { return *(EthAddr *)&eth_src; }
    const EthAddr &dst() const { return *(EthAddr *)&eth_dst; }

    int size() const { return sizeof(eth_hdr); }

    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

class EthPtr
{
  protected:
    friend class IpPtr;
    EthPacketPtr p;

  public:
    EthPtr() {}
    EthPtr(const EthPacketPtr &ptr) : p(ptr) { }

    EthHdr *operator->() { return (EthHdr *)p->data; }
    EthHdr &operator*() { return *(EthHdr *)p->data; }
    operator EthHdr *() { return (EthHdr *)p->data; }

    const EthHdr *operator->() const { return (const EthHdr *)p->data; }
    const EthHdr &operator*() const { return *(const EthHdr *)p->data; }
    operator const EthHdr *() const { return (const EthHdr *)p->data; }

    const EthPtr &operator=(const EthPacketPtr &ptr) { p = ptr; return *this; }

    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return p; }
    int off() const { return 0; }
    int pstart() const { return off() + ((const EthHdr*)p->data)->size(); }
};

/*
 * IP Stuff
 */
struct IpAddress
{
  protected:
    uint32_t _ip;

  public:
    IpAddress() : _ip(0)
    {}
    IpAddress(const uint32_t __ip) : _ip(__ip)
    {}

    uint32_t ip() const { return _ip; }

    std::string string() const;
};

std::ostream &operator<<(std::ostream &stream, const IpAddress &ia);
bool operator==(const IpAddress &left, const IpAddress &right);

struct IpNetmask : public IpAddress
{
  protected:
    uint8_t _netmask;

  public:
    IpNetmask() : IpAddress(), _netmask(0)
    {}
    IpNetmask(const uint32_t __ip, const uint8_t __netmask) :
        IpAddress(__ip), _netmask(__netmask)
    {}

    uint8_t netmask() const { return _netmask; }

    std::string string() const;
};

std::ostream &operator<<(std::ostream &stream, const IpNetmask &in);
bool operator==(const IpNetmask &left, const IpNetmask &right);

struct IpWithPort : public IpAddress
{
  protected:
    uint16_t _port;

  public:
    IpWithPort() : IpAddress(), _port(0)
    {}
    IpWithPort(const uint32_t __ip, const uint16_t __port) :
        IpAddress(__ip), _port(__port)
    {}

    uint8_t port() const { return _port; }

    std::string string() const;
};

std::ostream &operator<<(std::ostream &stream, const IpWithPort &iwp);
bool operator==(const IpWithPort &left, const IpWithPort &right);

struct IpOpt;
struct IpHdr : public ip_hdr
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
    void id(uint16_t _id) { ip_id = htons(_id); }
    void len(uint16_t _len) { ip_len = htons(_len); }


    bool options(std::vector<const IpOpt *> &vec) const;

    int size() const { return hlen(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

class IpPtr
{
  protected:
    friend class TcpPtr;
    friend class UdpPtr;
    EthPacketPtr p;

    void set(const EthPacketPtr &ptr)
    {
        p = 0;

        if (ptr) {
            EthHdr *eth = (EthHdr *)ptr->data;
            if (eth->type() == ETH_TYPE_IP)
                p = ptr;
        }
    }

  public:
    IpPtr() : p(0) {}
    IpPtr(const EthPacketPtr &ptr) : p(0) { set(ptr); }
    IpPtr(const EthPtr &ptr) : p(0) { set(ptr.p); }
    IpPtr(const IpPtr &ptr) : p(ptr.p) { }

    IpHdr *get() { return (IpHdr *)(p->data + sizeof(eth_hdr)); }
    IpHdr *operator->() { return get(); }
    IpHdr &operator*() { return *get(); }

    const IpHdr *get() const
    { return (const IpHdr *)(p->data + sizeof(eth_hdr)); }
    const IpHdr *operator->() const { return get(); }
    const IpHdr &operator*() const { return *get(); }

    const IpPtr &operator=(const EthPacketPtr &ptr) { set(ptr); return *this; }
    const IpPtr &operator=(const EthPtr &ptr) { set(ptr.p); return *this; }
    const IpPtr &operator=(const IpPtr &ptr) { p = ptr.p; return *this; }

    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return p; }
    int off() const { return sizeof(eth_hdr); }
    int pstart() const { return off() + get()->size(); }
};

uint16_t cksum(const IpPtr &ptr);

struct IpOpt : public ip_opt
{
    uint8_t type() const { return opt_type; }
    uint8_t typeNumber() const { return IP_OPT_NUMBER(opt_type); }
    uint8_t typeClass() const { return IP_OPT_CLASS(opt_type); }
    uint8_t typeCopied() const { return IP_OPT_COPIED(opt_type); }
    uint8_t len() const { return IP_OPT_TYPEONLY(type()) ? 1 : opt_len; }

    bool isNumber(int num) const { return typeNumber() == IP_OPT_NUMBER(num); }
    bool isClass(int cls) const { return typeClass() == IP_OPT_CLASS(cls); }
    bool isCopied(int cpy) const { return typeCopied() == IP_OPT_COPIED(cpy); }

    const uint8_t *data() const { return opt_data.data8; }
    void sec(ip_opt_data_sec &sec) const;
    void lsrr(ip_opt_data_rr &rr) const;
    void ssrr(ip_opt_data_rr &rr) const;
    void ts(ip_opt_data_ts &ts) const;
    uint16_t satid() const { return ntohs(opt_data.satid); }
    uint16_t mtup() const { return ntohs(opt_data.mtu); }
    uint16_t mtur() const { return ntohs(opt_data.mtu); }
    void tr(ip_opt_data_tr &tr) const;
    const uint32_t *addext() const { return &opt_data.addext[0]; }
    uint16_t rtralt() const { return ntohs(opt_data.rtralt); }
    void sdb(std::vector<uint32_t> &vec) const;
};

/*
 * TCP Stuff
 */
struct TcpOpt;
struct TcpHdr : public tcp_hdr
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
    void seq(uint32_t _seq) { th_seq = htonl(_seq); }
    void flags(uint8_t _flags) { th_flags  = _flags; } 

    bool options(std::vector<const TcpOpt *> &vec) const;

    int size() const { return off(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

class TcpPtr
{
  protected:
    EthPacketPtr p;
    int _off;

    void set(const EthPacketPtr &ptr, int offset) { p = ptr; _off = offset; }
    void set(const IpPtr &ptr)
    {
        if (ptr && ptr->proto() == IP_PROTO_TCP)
            set(ptr.p, sizeof(eth_hdr) + ptr->hlen());
        else
            set(0, 0);
    }

  public:
    TcpPtr() : p(0), _off(0) {}
    TcpPtr(const IpPtr &ptr) : p(0), _off(0) { set(ptr); }
    TcpPtr(const TcpPtr &ptr) : p(ptr.p), _off(ptr._off) {}

    TcpHdr *get() { return (TcpHdr *)(p->data + _off); }
    TcpHdr *operator->() { return get(); }
    TcpHdr &operator*() { return *get(); }

    const TcpHdr *get() const { return (const TcpHdr *)(p->data + _off); }
    const TcpHdr *operator->() const { return get(); }
    const TcpHdr &operator*() const { return *get(); }

    const TcpPtr &operator=(const IpPtr &i) { set(i); return *this; }
    const TcpPtr &operator=(const TcpPtr &t) { set(t.p, t._off); return *this; }

    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return p; }
    int off() const { return _off; }
    int pstart() const { return off() + get()->size(); }
};

uint16_t cksum(const TcpPtr &ptr);

typedef Range<uint16_t> SackRange;

struct TcpOpt : public tcp_opt
{
    uint8_t type() const { return opt_type; }
    uint8_t len() const { return TCP_OPT_TYPEONLY(type()) ? 1 : opt_len; }

    bool isopt(int opt) const { return type() == opt; }

    const uint8_t *data() const { return opt_data.data8; }

    uint16_t mss() const { return ntohs(opt_data.mss); }
    uint8_t wscale() const { return opt_data.wscale; }
    bool sack(std::vector<SackRange> &vec) const;
    uint32_t echo() const { return ntohl(opt_data.echo); }
    uint32_t tsval() const { return ntohl(opt_data.timestamp[0]); }
    uint32_t tsecr() const { return ntohl(opt_data.timestamp[1]); }
    uint32_t cc() const { return ntohl(opt_data.cc); }
    uint8_t cksum() const{ return opt_data.cksum; }
    const uint8_t *md5() const { return opt_data.md5; }

    int size() const { return len(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

/*
 * UDP Stuff
 */
struct UdpHdr : public udp_hdr
{
    uint16_t sport() const { return ntohs(uh_sport); }
    uint16_t dport() const { return ntohs(uh_dport); }
    uint16_t len() const { return ntohs(uh_ulen); }
    uint16_t sum() const { return uh_sum; }

    void sum(uint16_t sum) { uh_sum = sum; }
    void len(uint16_t _len) { uh_ulen = htons(_len); }

    int size() const { return sizeof(udp_hdr); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

class UdpPtr
{
  protected:
    EthPacketPtr p;
    int _off;

    void set(const EthPacketPtr &ptr, int offset) { p = ptr; _off = offset; }
    void set(const IpPtr &ptr)
    {
        if (ptr && ptr->proto() == IP_PROTO_UDP)
            set(ptr.p, sizeof(eth_hdr) + ptr->hlen());
        else
            set(0, 0);
    }

  public:
    UdpPtr() : p(0), _off(0) {}
    UdpPtr(const IpPtr &ptr) : p(0), _off(0) { set(ptr); }
    UdpPtr(const UdpPtr &ptr) : p(ptr.p), _off(ptr._off) {}

    UdpHdr *get() { return (UdpHdr *)(p->data + _off); }
    UdpHdr *operator->() { return get(); }
    UdpHdr &operator*() { return *get(); }

    const UdpHdr *get() const { return (const UdpHdr *)(p->data + _off); }
    const UdpHdr *operator->() const { return get(); }
    const UdpHdr &operator*() const { return *get(); }

    const UdpPtr &operator=(const IpPtr &i) { set(i); return *this; }
    const UdpPtr &operator=(const UdpPtr &t) { set(t.p, t._off); return *this; }

    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return p; }
    int off() const { return _off; }
    int pstart() const { return off() + get()->size(); }
};

uint16_t cksum(const UdpPtr &ptr);

int hsplit(const EthPacketPtr &ptr);

} // namespace Net

#endif // __BASE_INET_HH__
