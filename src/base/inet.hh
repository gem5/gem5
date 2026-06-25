/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#ifndef __BASE_INET_HH__
#define __BASE_INET_HH__

#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/ip_icmp.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"
#include "dev/net/etherpkt.hh"

namespace gem5
{

namespace networking
{

constexpr size_t IP6_HDR_LEN = 40;

#define IPOPT_TYPEONLY(o) ((o) == IPOPT_EOL || (o) == IPOPT_NOP)
#define TCPOPT_TYPEONLY(o) ((o) == TCPOPT_EOL || (o) == TCPOPT_NOP)

/*
 * Security option data - RFC 791, 3.1
 */
struct GEM5_PACKED ip_opt_data_sec
{
    uint16_t s;     /* security */
    uint16_t c;     /* compartments */
    uint16_t h;     /* handling restrictions */
    uint8_t tcc[3]; /* transmission control code */
};

/*
 * {Loose Source, Record, Strict Source} Route option data - RFC 791, 3.1
 */
struct GEM5_PACKED ip_opt_data_rr
{
    uint8_t ptr;        /* from start of option, >= 4 */
    uint32_t iplist[0]; /* list of IP addresses */
};

/*
 * Timestamp option data - RFC 791, 3.1
 */
struct GEM5_PACKED ip_opt_data_ts
{
    uint8_t ptr; /* from start of option, >= 5 */
#if BYTE_ORDER == BIG_ENDIAN
    uint8_t oflw : 4, /* number of IPs skipped */
        flg : 4;      /* address[ / timestamp] flag */
#elif BYTE_ORDER == LITTLE_ENDIAN
    uint8_t flg : 4, oflw : 4;
#endif
    uint32_t ipts[0]; /* IP address [/ timestamp] pairs */
};

/*
 * Traceroute option data - RFC 1393, 2.2
 */
struct GEM5_PACKED ip_opt_data_tr
{
    uint16_t id;     /* ID number */
    uint16_t ohc;    /* outbound hop count */
    uint16_t rhc;    /* return hop count */
    uint32_t origip; /* originator IP address */
};

struct GEM5_PACKED ip_opt
{
    uint8_t opt_type;
    uint8_t opt_len;
    union ip_opt_data
    {
        ip_opt_data_sec sec;
        ip_opt_data_rr rr;
        ip_opt_data_ts ts;
        uint16_t satid;
        uint16_t mtu;
        ip_opt_data_tr tr;
        uint32_t addext[2];
        uint16_t rtralt;
        uint32_t sdb[9];
        uint8_t data8[38];
    } opt_data;
};

struct GEM5_PACKED tcp_opt
{
    uint8_t opt_type;
    uint8_t opt_len;
    union
    {
        uint16_t mss;
        uint8_t wscale;
        uint32_t sack[8];
        uint32_t echo;
        uint32_t timestamp[2];
        uint32_t cc;
        uint8_t cksum;
        uint8_t md5[16];
        uint8_t data8[40];
    } opt_data;
};

/*
 * Ethernet Stuff
 */
struct EthAddr : protected ether_addr
{
  protected:
    void parse(const std::string &addr);

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    EthAddr();
    EthAddr(const uint8_t ea[ETHER_ADDR_LEN]);
    EthAddr(const ether_addr &ea);
    EthAddr(const std::string &addr);
    const EthAddr &operator=(const ether_addr &ea);
    const EthAddr &operator=(const std::string &addr);
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     */
    int
    size() const
    {
        return sizeof(ether_addr);
    }

    /**
     * @ingroup api_inet
     * @{
     */
    const uint8_t *
    bytes() const
    {
        return &ether_addr_octet[0];
    }
    uint8_t *
    bytes()
    {
        return &ether_addr_octet[0];
    }
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     * @{
     */
    const uint8_t *
    addr() const
    {
        return &ether_addr_octet[0];
    }
    bool
    unicast() const
    {
        return !(ether_addr_octet[0] & 0x01);
    }
    bool multicast() const { return !unicast() && !broadcast(); }
    bool broadcast() const
    {
        bool isBroadcast = true;
        for (int i = 0; i < ETHER_ADDR_LEN; ++i) {
            isBroadcast = isBroadcast && ether_addr_octet[i] == 0xff;
        }

        return isBroadcast;
    }
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     */
    std::string string() const;

    /**
     * @ingroup api_inet
     */
    operator uint64_t() const
    {
        uint64_t reg = 0;
        reg |= ((uint64_t)ether_addr_octet[0]) << 40;
        reg |= ((uint64_t)ether_addr_octet[1]) << 32;
        reg |= ((uint64_t)ether_addr_octet[2]) << 24;
        reg |= ((uint64_t)ether_addr_octet[3]) << 16;
        reg |= ((uint64_t)ether_addr_octet[4]) << 8;
        reg |= ((uint64_t)ether_addr_octet[5]) << 0;
        return reg;
    }

};

/**
 * @ingroup api_inet
 * @{
 */
std::ostream &operator<<(std::ostream &stream, const EthAddr &ea);
bool operator==(const EthAddr &left, const EthAddr &right);
/** @} */ // end of api_inet

struct EthHdr : public ether_header
{
    bool
    isVlan() const
    {
        return (ntohs(ether_type) == ETHERTYPE_VLAN);
    }
    uint16_t type() const {
        if (!isVlan())
            return ntohs(ether_type);
        else
            // L3 type is now 16 bytes into the hdr with 802.1Q
            // instead of 12.  802.1 is supported.
            return ntohs(*((uint16_t*)(((uint8_t *)this) + 16)));
    }
    uint16_t vlanId() const {
        if (isVlan())
            return ntohs(*((uint16_t*)(((uint8_t *)this) + 14)));
        else
            return 0x0000;
    }

    const EthAddr &
    src() const
    {
        return *(EthAddr *)&ether_shost;
    }
    const EthAddr &
    dst() const
    {
        return *(EthAddr *)&ether_dhost;
    }

    int size() const {
        if (!isVlan())
            return sizeof(ether_header);
        else
            return (sizeof(ether_header) + 4);
    }

    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + size(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + size(); }
};

class EthPtr
{
  protected:
    friend class IpPtr;
    friend class Ip6Ptr;
    EthPacketPtr p;

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    EthPtr() {}
    EthPtr(const EthPacketPtr &ptr) : p(ptr) { }
    /** @} */ // end of api_inet

    EthHdr *operator->() { return (EthHdr *)p->data; }
    EthHdr &operator*() { return *(EthHdr *)p->data; }
    operator EthHdr *() { return (EthHdr *)p->data; }

    const EthHdr *operator->() const { return (const EthHdr *)p->data; }
    const EthHdr &operator*() const { return *(const EthHdr *)p->data; }
    operator const EthHdr *() const { return (const EthHdr *)p->data; }

    /**
     * @ingroup api_inet
     */
    const EthPtr &operator=(const EthPacketPtr &ptr) { p = ptr; return *this; }

    /**
     * @ingroup api_inet
     * @{
     */
    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return (p != nullptr); }
    int off() const { return 0; }
    int pstart() const { return off() + ((const EthHdr*)p->data)->size(); }
    /** @} */ // end of api_inet
};

/*
 * IP Stuff
 */
struct IpAddress
{
  protected:
    uint32_t _ip;

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    IpAddress() : _ip(0)
    {}
    IpAddress(const uint32_t __ip) : _ip(__ip)
    {}
    /** @} */ // end of api_net

    /**
     * @ingroup api_inet
     */
    uint32_t ip() const { return _ip; }

    /**
     * @ingroup api_inet
     */
    std::string string() const;
};

/**
 * @ingroup api_inet
 * @{
 */
std::ostream &operator<<(std::ostream &stream, const IpAddress &ia);
bool operator==(const IpAddress &left, const IpAddress &right);
/** @} */ // end of api_inet

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

    /**
     * @ingroup api_inet
     */
    uint8_t netmask() const { return _netmask; }

    std::string string() const;
};

/**
 * @ingroup api_inet
 * @{
 */
std::ostream &operator<<(std::ostream &stream, const IpNetmask &in);
bool operator==(const IpNetmask &left, const IpNetmask &right);
/** @} */ // end of api_inet

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

    /**
     * @ingroup api_inet
     */
    uint8_t port() const { return _port; }

    std::string string() const;
};

/**
 * @ingroup api_inet
 * @{
 */
std::ostream &operator<<(std::ostream &stream, const IpWithPort &iwp);
bool operator==(const IpWithPort &left, const IpWithPort &right);
/** @} */ // end of api_inet

struct IpOpt;
struct IpHdr : public ip
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
    uint32_t
    src() const
    {
        return ntohl(ip_src.s_addr);
    }
    uint32_t
    dst() const
    {
        return ntohl(ip_dst.s_addr);
    }

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
    bool eth_hdr_vlan;

    void set(const EthPacketPtr &ptr)
    {
        p = 0;
        eth_hdr_vlan = false;

        if (ptr) {
            EthHdr *eth = (EthHdr *)ptr->data;
            if (eth->type() == ETHERTYPE_IP) {
                p = ptr;
            }
            if (eth->isVlan())
                eth_hdr_vlan = true;
        }
    }

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    IpPtr() : p(0), eth_hdr_vlan(false) {}
    IpPtr(const EthPacketPtr &ptr) : p(0), eth_hdr_vlan(false) { set(ptr); }
    IpPtr(const EthPtr &ptr) : p(0), eth_hdr_vlan(false) { set(ptr.p); }
    IpPtr(const IpPtr &ptr) : p(ptr.p), eth_hdr_vlan(ptr.eth_hdr_vlan) { }
    /** @} */ // end of api_inet

    IpHdr *
    get()
    {
        return (IpHdr *)(p->data + sizeof(ether_header) +
                         ((eth_hdr_vlan) ? 4 : 0));
    }
    IpHdr *operator->() { return get(); }
    IpHdr &operator*() { return *get(); }

    /**
     * @ingroup api_inet
     * @{
     */
    const IpHdr *get() const
    {
        return (const IpHdr *)(p->data + sizeof(ether_header) +
                               ((eth_hdr_vlan) ? 4 : 0));
    }
    const IpHdr *operator->() const { return get(); }
    const IpHdr &operator*() const { return *get(); }
    /** @} */ // end of api_inet

    const IpPtr &operator=(const EthPacketPtr &ptr) { set(ptr); return *this; }
    const IpPtr &operator=(const EthPtr &ptr) { set(ptr.p); return *this; }
    const IpPtr &operator=(const IpPtr &ptr) { p = ptr.p; return *this; }

    /**
     * @ingroup api_inet
     * @{
     */
    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return (p != nullptr); }
    int
    off() const
    {
        return (sizeof(ether_header) + ((eth_hdr_vlan) ? 4 : 0));
    }
    int pstart() const { return (off() + get()->size()); }
    /** @} */ // end of api_inet
};

/**
 * @ingroup api_inet
 */
uint16_t cksum(const IpPtr &ptr);

struct IpOpt : public ip_opt
{
    uint8_t type() const { return opt_type; }
    uint8_t
    typeNumber() const
    {
        return IPOPT_NUMBER(opt_type);
    }
    uint8_t
    typeClass() const
    {
        return IPOPT_CLASS(opt_type);
    }
    uint8_t
    typeCopied() const
    {
        return IPOPT_COPIED(opt_type);
    }
    uint8_t
    len() const
    {
        return IPOPT_TYPEONLY(type()) ? 1 : opt_len;
    }

    bool
    isNumber(int num) const
    {
        return typeNumber() == IPOPT_NUMBER(num);
    }
    bool
    isClass(int cls) const
    {
        return typeClass() == IPOPT_CLASS(cls);
    }
    bool
    isCopied(int cpy) const
    {
        return typeCopied() == IPOPT_COPIED(cpy);
    }

    const uint8_t *data() const { return opt_data.data8; }
    void sec(ip_opt_data_sec &sec) const;
    void lsrr(ip_opt_data_rr &rr) const;
    void ssrr(ip_opt_data_rr &rr) const;
    void ts(ip_opt_data_ts &ts) const;
    uint16_t satid() const { return ntohs(opt_data.satid); }
    uint16_t mtup() const { return ntohs(opt_data.mtu); }
    uint16_t mtur() const { return ntohs(opt_data.mtu); }
    void tr(ip_opt_data_tr &tr) const;
    uint16_t rtralt() const { return ntohs(opt_data.rtralt); }
    void sdb(std::vector<uint32_t> &vec) const;
};

/*
 * Ip6 Classes
 */
struct Ip6Opt;
struct Ip6Hdr : public ip6_hdr
{
    uint8_t version() const { return ip6_vfc; }
    uint32_t flow() const { return ntohl(ip6_flow); }
    uint16_t plen() const { return ntohs(ip6_plen); }
    uint16_t hlen() const { return IP6_HDR_LEN; }
    uint8_t nxt() const { return ip6_nxt; }
    uint8_t hlim() const { return ip6_hlim; }

    const uint8_t *
    src() const
    {
        return ip6_src.s6_addr;
    }
    const uint8_t *
    dst() const
    {
        return ip6_dst.s6_addr;
    }

    int extensionLength() const;
    const Ip6Opt* getExt(uint8_t ext) const;
    const Ip6Opt *
    fragmentExt() const
    {
        return getExt(IPPROTO_FRAGMENT);
    }
    const Ip6Opt *
    rtTypeExt() const
    {
        return getExt(IPPROTO_ROUTING);
    }
    const Ip6Opt *
    dstOptExt() const
    {
        return getExt(IPPROTO_DSTOPTS);
    }
    uint8_t proto() const;

    void plen(uint16_t _plen) { ip6_plen = htons(_plen); }

    int size() const { return IP6_HDR_LEN + extensionLength(); }
    const uint8_t *bytes() const { return (const uint8_t *)this; }
    const uint8_t *payload() const { return bytes() + IP6_HDR_LEN
                                            + extensionLength(); }
    uint8_t *bytes() { return (uint8_t *)this; }
    uint8_t *payload() { return bytes() + IP6_HDR_LEN
                                + extensionLength(); }
};

class Ip6Ptr
{
  protected:
    friend class TcpPtr;
    friend class UdpPtr;
    EthPacketPtr p;
    bool eth_hdr_vlan;

    void set(const EthPacketPtr &ptr)
    {
        p = 0;
        eth_hdr_vlan = false;

        if (ptr) {
            EthHdr *eth = (EthHdr *)ptr->data;
            if (eth->type() == ETHERTYPE_IPV6) {
                p = ptr;
            }
            if (eth->isVlan())
                eth_hdr_vlan = true;
        }
    }

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    Ip6Ptr() : p(0), eth_hdr_vlan(false) {}
    Ip6Ptr(const EthPacketPtr &ptr) : p(0), eth_hdr_vlan(false) { set(ptr); }
    Ip6Ptr(const EthPtr &ptr) : p(0), eth_hdr_vlan(false) { set(ptr.p); }
    Ip6Ptr(const Ip6Ptr &ptr) : p(ptr.p), eth_hdr_vlan(ptr.eth_hdr_vlan) { }
    /** @} */ // end of api_inet

    Ip6Hdr *
    get()
    {
        return (Ip6Hdr *)(p->data + sizeof(ether_header) +
                          ((eth_hdr_vlan) ? 4 : 0));
    }
    Ip6Hdr *operator->() { return get(); }
    Ip6Hdr &operator*() { return *get(); }

    const Ip6Hdr *get() const
    {
        return (const Ip6Hdr *)(p->data + sizeof(ether_header) +
                                ((eth_hdr_vlan) ? 4 : 0));
    }
    const Ip6Hdr *operator->() const { return get(); }
    const Ip6Hdr &operator*() const { return *get(); }

    /**
     * @ingroup api_inet
     * @{
     */
    const Ip6Ptr &operator=(const EthPacketPtr &ptr)
    { set(ptr); return *this; }
    const Ip6Ptr &operator=(const EthPtr &ptr)
    { set(ptr.p); return *this; }
    const Ip6Ptr &operator=(const Ip6Ptr &ptr)
    { p = ptr.p; return *this; }
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     * @{
     */
    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return (p != nullptr); }
    int
    off() const
    {
        return sizeof(ether_header) + ((eth_hdr_vlan) ? 4 : 0);
    }
    int pstart() const { return off() + get()->size(); }
    /** @} */ // end of api_inet
};

// The ipv6 opt header is incomplete and
// newer NIC card filters expect a more robust
// ipv6 header option declaration.
struct ip6_opt_fragment
{
    uint16_t offlg;
    uint32_t ident;
};

struct ip6_opt_routing_type2
{
    uint8_t type;
    uint8_t segleft;
    uint32_t reserved;
    in6_addr addr;
};

struct GEM5_PACKED ip6_opt_dstopts
{
    uint8_t type;
    uint8_t length;
    in6_addr addr;
};

struct GEM5_PACKED ip6_opt_hdr
{
    uint8_t ext_nxt;
    uint8_t ext_len;
    union
    {
        struct ip6_opt_fragment fragment;
        struct ip6_opt_routing_type2 rtType2;
        struct ip6_opt_dstopts dstOpts;
    } ext_data;
};

struct Ip6Opt : public ip6_opt_hdr
{
    uint8_t nxt() const { return ext_nxt; }
    uint8_t extlen() const { return ext_len; }
    uint8_t len() const { return extlen() + 8; }

    // Supporting the types of header extensions likely to be encountered:
    // fragment, routing type 2 and dstopts.

    // Routing type 2
    uint8_t  rtType2Type() const { return ext_data.rtType2.type; }
    uint8_t  rtType2SegLft() const { return ext_data.rtType2.segleft; }
    const uint8_t *
    rtType2Addr() const
    {
        return ext_data.rtType2.addr.s6_addr;
    }

    // Fragment
    uint16_t fragmentOfflg() const { return ntohs(ext_data.fragment.offlg); }
    uint32_t fragmentIdent() const { return ntohl(ext_data.fragment.ident); }

    // Dst Options/Home Address Option
    uint8_t dstOptType() const { return ext_data.dstOpts.type; }
    uint8_t dstOptLength() const { return ext_data.dstOpts.length; }
    const uint8_t *
    dstOptAddr() const
    {
        return ext_data.dstOpts.addr.s6_addr;
    }
};


/*
 * TCP Stuff
 */
struct TcpOpt;
struct TcpHdr : public tcphdr
{
    uint16_t sport() const { return ntohs(th_sport); }
    uint16_t dport() const { return ntohs(th_dport); }
    uint32_t seq() const { return ntohl(th_seq); }
    uint32_t ack() const { return ntohl(th_ack); }
    uint8_t  off() const { return th_off*4; }
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
        if (ptr && ptr->proto() == IPPROTO_TCP) {
            set(ptr.p, ptr.pstart());
        } else {
            set(0, 0);
        }
    }
    void set(const Ip6Ptr &ptr)
    {
        if (ptr && ptr->proto() == IPPROTO_TCP) {
            set(ptr.p, ptr.pstart());
        } else {
            set(0, 0);
        }
    }

  public:
    /**
     * @ingroup api_inet
     * @{
     */
    TcpPtr() : p(0), _off(0) {}
    TcpPtr(const IpPtr &ptr) : p(0), _off(0) { set(ptr); }
    TcpPtr(const Ip6Ptr &ptr) : p(0), _off(0) { set(ptr); }
    TcpPtr(const TcpPtr &ptr) : p(ptr.p), _off(ptr._off) {}
    /** @} */ // end of api_inet

    TcpHdr *get() { return (TcpHdr *)(p->data + _off); }
    TcpHdr *operator->() { return get(); }
    TcpHdr &operator*() { return *get(); }

    const TcpHdr *get() const { return (const TcpHdr *)(p->data + _off); }
    const TcpHdr *operator->() const { return get(); }
    const TcpHdr &operator*() const { return *get(); }

    /**
     * @ingroup api_inet
     * @{
     */
    const TcpPtr &operator=(const IpPtr &i)
    { set(i); return *this; }
    const TcpPtr &operator=(const TcpPtr &t)
    { set(t.p, t._off); return *this; }
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     * @{
     */
    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return (p != nullptr); }
    int off() const { return _off; }
    int pstart() const { return off() + get()->size(); }
    /** @} */ // end of api_inet
};

/**
 * @ingroup api_inet
 */
uint16_t cksum(const TcpPtr &ptr);

struct TcpOpt : public tcp_opt
{
    uint8_t type() const { return opt_type; }
    uint8_t
    len() const
    {
        return TCPOPT_TYPEONLY(type()) ? 1 : opt_len;
    }

    bool isopt(int opt) const { return type() == opt; }

    const uint8_t *data() const { return opt_data.data8; }

    uint16_t mss() const { return ntohs(opt_data.mss); }
    uint8_t wscale() const { return opt_data.wscale; }
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
struct UdpHdr : public udphdr
{
    uint16_t sport() const { return ntohs(uh_sport); }
    uint16_t dport() const { return ntohs(uh_dport); }
    uint16_t len() const { return ntohs(uh_ulen); }
    uint16_t sum() const { return uh_sum; }

    void sum(uint16_t sum) { uh_sum = sum; }
    void len(uint16_t _len) { uh_ulen = htons(_len); }

    int
    size() const
    {
        return sizeof(udphdr);
    }
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
        if (ptr && ptr->proto() == IPPROTO_UDP) {
            set(ptr.p, ptr.pstart());
        } else {
            set(0, 0);
        }
    }
    void set(const Ip6Ptr &ptr)
    {
        if (ptr && ptr->proto() == IPPROTO_UDP) {
            set(ptr.p, ptr.pstart());
        } else {
            set(0, 0);
        }
    }

  public:
    /**
     * @ingroup api_inet
     */
    UdpPtr() : p(0), _off(0) {}
    UdpPtr(const IpPtr &ptr) : p(0), _off(0) { set(ptr); }
    UdpPtr(const Ip6Ptr &ptr) : p(0), _off(0) { set(ptr); }
    UdpPtr(const UdpPtr &ptr) : p(ptr.p), _off(ptr._off) {}
    /** @} */ // end of api_inet

    UdpHdr *get() { return (UdpHdr *)(p->data + _off); }
    UdpHdr *operator->() { return get(); }
    UdpHdr &operator*() { return *get(); }

    const UdpHdr *get() const { return (const UdpHdr *)(p->data + _off); }
    const UdpHdr *operator->() const { return get(); }
    const UdpHdr &operator*() const { return *get(); }

    /**
     * @ingroup api_inet
     * @{
     */
    const UdpPtr &operator=(const IpPtr &i) { set(i); return *this; }
    const UdpPtr &operator=(const UdpPtr &t)
    { set(t.p, t._off); return *this; }
    /** @} */ // end of api_inet

    /**
     * @ingroup api_inet
     * @{
     */
    const EthPacketPtr packet() const { return p; }
    EthPacketPtr packet() { return p; }
    bool operator!() const { return !p; }
    operator bool() const { return (p != nullptr); }
    int off() const { return _off; }
    int pstart() const { return off() + get()->size(); }
    /** @} */ // end of api_inet
};

/**
 * @ingroup api_inet
 * @{
 */
uint16_t __tu_cksum6(const Ip6Ptr &ip6);
uint16_t __tu_cksum(const IpPtr &ip);
uint16_t cksum(const UdpPtr &ptr);
/** @} */ // end of api_inet

/**
 * @ingroup api_inet
 */
int hsplit(const EthPacketPtr &ptr);

} // namespace networking
} // namespace gem5

#endif // __BASE_INET_HH__
