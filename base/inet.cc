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

#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "sim/host.hh"
#include "base/inet.hh"

using namespace std;
string
eaddr_string(const uint8_t a[6])
{
    stringstream stream;
    ccprintf(stream, "%x:%x:%x:%x:%x:%x", a[0], a[1], a[2], a[3], a[4], a[5]);

    return stream.str();
}

uint16_t
IpHdr::ip_cksum() const
{
    uint16_t sum = ip_cksum_add(this, hlen(), 0);
    return ip_cksum_carry(sum);
}

uint16_t
IpHdr::tu_cksum() const
{
    uint16_t sum = ip_cksum_add(payload(), len() - hlen(), 0);
    sum = ip_cksum_add(&ip_src, 4, sum);
    sum = ip_cksum_add(&ip_dst, 4, sum);
    sum += htons(ip_p + ip_len);
    return ip_cksum_carry(sum);
}
