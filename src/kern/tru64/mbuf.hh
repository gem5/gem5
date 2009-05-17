/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

#ifndef __MBUF_HH__
#define __MBUF_HH__

#include "arch/isa_traits.hh"
#include "base/types.hh"

namespace tru64 {

struct m_hdr {
    Addr        mh_next;        // 0x00
    Addr        mh_nextpkt;     // 0x08
    Addr        mh_data;        // 0x10
    int32_t     mh_len;         // 0x18
    int32_t     mh_type;        // 0x1C
    int32_t     mh_flags;       // 0x20
    int32_t     mh_pad0;        // 0x24
    Addr        mh_foo[4];      // 0x28, 0x30, 0x38, 0x40
};

struct  pkthdr {
    int32_t     len;
    int32_t     protocolSum;
    Addr        rcvif;
};

struct m_ext {
    Addr        ext_buf;        // 0x00
    Addr        ext_free;       // 0x08
    uint32_t    ext_size;       // 0x10
    uint32_t    ext_pad0;       // 0x14
    Addr        ext_arg;        // 0x18
    struct      ext_refq {
        Addr    forw, back;     // 0x20, 0x28
    } ext_ref;
    Addr        uiomove_f;      // 0x30
    int32_t     protocolSum;    // 0x38
    int32_t     bytesSummed;    // 0x3C
    Addr        checksum;       // 0x40
};

struct mbuf {
    struct      m_hdr m_hdr;
    union {
        struct {
            struct      pkthdr MH_pkthdr;
            union {
                struct  m_ext MH_ext;
                char    MH_databuf[1];
            } MH_dat;
        } MH;
        char    M_databuf[1];
    } M_dat;
};

#define m_attr          m_hdr.mh_attr
#define m_next          m_hdr.mh_next
#define m_len           m_hdr.mh_len
#define m_data          m_hdr.mh_data
#define m_type          m_hdr.mh_type
#define m_flags         m_hdr.mh_flags
#define m_nextpkt       m_hdr.mh_nextpkt
#define m_act           m_nextpkt
#define m_pkthdr        M_dat.MH.MH_pkthdr
#define m_ext           M_dat.MH.MH_dat.MH_ext
#define m_pktdat        M_dat.MH.MH_dat.MH_databuf
#define m_dat           M_dat.M_databuf

}

#endif // __MBUF_HH__
