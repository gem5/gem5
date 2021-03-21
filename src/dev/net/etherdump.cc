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
 */

/* @file
 * Simple object for creating a simple pcap style packet trace
 */
#include "dev/net/etherdump.hh"

#include <sys/time.h>

#include <algorithm>
#include <string>

#include "base/logging.hh"
#include "base/output.hh"
#include "sim/core.hh"
#include "sim/cur_tick.hh"

using std::string;

namespace gem5
{

EtherDump::EtherDump(const Params &p)
    : SimObject(p), stream(simout.create(p.file, true)->stream()),
      maxlen(p.maxlen)
{
}

#define DLT_EN10MB              1               // Ethernet (10Mb)
#define TCPDUMP_MAGIC           0xa1b2c3d4
#define PCAP_VERSION_MAJOR      2
#define PCAP_VERSION_MINOR      4

struct pcap_file_header
{
    uint32_t magic;
    uint16_t version_major;
    uint16_t version_minor;
    int32_t thiszone;           // gmt to local correction
    uint32_t sigfigs;           // accuracy of timestamps
    uint32_t snaplen;           // max length saved portion of each pkt
    uint32_t linktype;          // data link type (DLT_*)
};

struct pcap_pkthdr
{
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t caplen;            // length of portion present
    uint32_t len;               // length this packet (off wire)
};

void
EtherDump::init()
{
    struct pcap_file_header hdr;
    hdr.magic = TCPDUMP_MAGIC;
    hdr.version_major = PCAP_VERSION_MAJOR;
    hdr.version_minor = PCAP_VERSION_MINOR;

    hdr.thiszone = 0;
    hdr.snaplen = 1500;
    hdr.sigfigs = 0;
    hdr.linktype = DLT_EN10MB;

    stream->write(reinterpret_cast<char *>(&hdr), sizeof(hdr));

    stream->flush();
}

void
EtherDump::dumpPacket(EthPacketPtr &packet)
{
    pcap_pkthdr pkthdr;
    pkthdr.seconds = curTick() / sim_clock::as_int::s;
    pkthdr.microseconds = (curTick() / sim_clock::as_int::us) % 1000000ULL;
    pkthdr.caplen = std::min(packet->length, maxlen);
    pkthdr.len = packet->length;
    stream->write(reinterpret_cast<char *>(&pkthdr), sizeof(pkthdr));
    stream->write(reinterpret_cast<char *>(packet->data), pkthdr.caplen);
    stream->flush();
}

} // namespace gem5
