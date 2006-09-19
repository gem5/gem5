/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

/* @file
 * Register and structure descriptions for Intel's 8254x line of gigabit ethernet controllers.
 */

namespace iGbReg {

const uint32_t CTRL     = 0x00000;
const uint32_t STATUS   = 0x00008;
const uint32_t EECD     = 0x00010;
const uint32_t CTRL_EXT = 0x00018;
const uint32_t PBA      = 0x01000;
const uint32_t ICR      = 0x000C0;
const uint32_t ITR      = 0x000C4;
const uint32_t ICS      = 0x000C8;
const uint32_t IMS      = 0x000D0;
const uint32_t IMC      = 0x000D8;
const uint32_t RCTL     = 0x00100;
const uint32_t RDBAL    = 0x02800;
const uint32_t RDBAH    = 0x02804;
const uint32_t RDLEN    = 0x02808;
const uint32_t RDH      = 0x02810;
const uint32_t RDT      = 0x02818;
const uint32_t RDTR     = 0x02820;
const uint32_t RADV     = 0x0282C;
const uint32_t RSRPD    = 0x02C00;
const uint32_t TCTL     = 0x00400;
const uint32_t TDBAL    = 0x03800;
const uint32_t TDBAH    = 0x03804;
const uint32_t TDLEN    = 0x03808;
const uint32_t TDH      = 0x03810;
const uint32_t THT      = 0x03818;
const uint32_t TIDV     = 0x03820;
const uint32_t TXDMAC   = 0x03000;
const uint32_t TXDCTL   = 0x03828;
const uint32_t TADV     = 0x0282C;
const uint32_t TSPMT    = 0x03830;
const uint32_t RXDCTL   = 0x02828;
const uint32_t RXCSUM   = 0x05000;

struct RxDesc {
    Addr buf;
    uint16_t len;
    uint16_t csum;
    union {
        uint8_t status;
        struct { // these may be in the worng order
            uint8_t dd:1;    // descriptor done (hw is done when 1)
            uint8_t eop:1;   // end of packet
            uint8_t xism:1;  // ignore checksum
            uint8_t vp:1;    // packet is vlan packet
            uint8_t rsv:1;   // reserved
            uint8_t tcpcs:1; // TCP checksum done
            uint8_t ipcs:1;  // IP checksum done
            uint8_t pif:1;   // passed in-exact filter
        } st;
    };
    union {
        uint8_t errors;
        struct {
            uint8_t ce:1;   // crc error or alignment error
            uint8_t se:1;   // symbol error
            uint8_t seq:1;  // sequence error
            uint8_t rsv:1;  // reserved
            uint8_t cxe:1;  // carrier extension error
            uint8_t tcpe:1; // tcp checksum error
            uint8_t ipe:1;  // ip checksum error
            uint8_t rxe:1;  // PX data error
        } er;
    };
    union {
        uint16_t special;
        struct {
            uint16_t vlan:12; //vlan id
            uint16_t cfi:1;   // canocial form id
            uint16_t pri:3;   // user priority
        } sp;
    };
};

union TxDesc {
    uint8_t data[16];
    struct {
        Addr buf;
        uint16_t len;
        uint8_t  cso;
        union {
            uint8_t command;
            struct {
                uint8_t eop:1;  // end of packet
                uint8_t ifcs:1; // insert crc
                uint8_t ic:1;   // insert checksum
                uint8_t rs:1;   // report status
                uint8_t rps:1;  // report packet sent
                uint8_t dext:1; // extension
                uint8_t vle:1;  // vlan enable
                uint8_t ide:1;  // interrupt delay enable
            } cmd;
        };
        union {
            uint8_t status:4;
            struct {
                uint8_t dd:1; // descriptor done
                uint8_t ec:1; // excess collisions
                uint8_t lc:1; // late collision
                uint8_t tu:1; // transmit underrun
            } st;
        };
        uint8_t reserved:4;
        uint8_t css;
        union {
            uint16_t special;
            struct {
                uint16_t vlan:12; //vlan id
                uint16_t cfi:1;   // canocial form id
                uint16_t pri:3;   // user priority
            } sp;
        };
    } legacy;

    // Type 0000 descriptor
    struct {
        uint8_t ipcss;
        uint8_t ipcso;
        uint16_t ipcse;
        uint8_t tucss;
        uint8_t tucso;
        uint16_t tucse;
        uint32_t paylen:20;
        uint8_t dtype:4;
        union {
            uint8_t tucommand;
            struct {
                uint8_t tcp:1;  // tcp/udp
                uint8_t ip:1;   // ip ipv4/ipv6
                uint8_t tse:1;  // tcp segment enbale
                uint8_t rs:1;   // report status
                uint8_t rsv0:1; // reserved
                uint8_t dext:1; // descriptor extension
                uint8_t rsv1:1; // reserved
                uint8_t ide:1;  // interrupt delay enable
            } tucmd;
        };
        union {
            uint8_t status:4;
            struct {
                uint8_t dd:1;
                uint8_t rsvd:3;
            } sta;
        };
        uint8_t reserved:4;
        uint8_t hdrlen;
        uint16_t mss;
    } t0;

    // Type 0001 descriptor
    struct {
        Addr buf;
        uint32_t dtalen:20;
        uint8_t dtype:4;
        union {
            uint8_t dcommand;
            struct {
                uint8_t eop:1;  // end of packet
                uint8_t ifcs:1; // insert crc
                uint8_t tse:1;  // segmentation enable
                uint8_t rs:1;   // report status
                uint8_t rps:1;  // report packet sent
                uint8_t dext:1; // extension
                uint8_t vle:1;  // vlan enable
                uint8_t ide:1;  // interrupt delay enable
            } dcmd;
        };
        union {
            uint8_t status:4;
            struct {
                uint8_t dd:1; // descriptor done
                uint8_t ec:1; // excess collisions
                uint8_t lc:1; // late collision
                uint8_t tu:1; // transmit underrun
            } sta;
        };
        union {
            uint8_t pktopts;
            struct {
                uint8_t ixsm:1; // insert ip checksum
                uint8_t txsm:1; // insert tcp checksum
            };
        };
        union {
            uint16_t special;
            struct {
                uint16_t vlan:12; //vlan id
                uint16_t cfi:1;   // canocial form id
                uint16_t pri:3;   // user priority
            } sp;
        };
    } t1;

    // Junk to test descriptor type!
    struct {
        uint64_t junk;
        uint32_t junk1:20;
        uint8_t dtype;
        uint8_t junk2:5;
        uint8_t dext:1;
        uint8_t junk3:2;
        uint8_t junk4:4;
        uint32_t junk5;
    } type;
};

}; // iGbReg namespace
