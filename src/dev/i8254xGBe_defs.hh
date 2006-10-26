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

const uint32_t CTRL     = 0x00000; //*
const uint32_t STATUS   = 0x00008; //*
const uint32_t EECD     = 0x00010; //*
const uint32_t EERD     = 0x00014; //*
const uint32_t CTRL_EXT = 0x00018;
const uint32_t PBA      = 0x01000;
const uint32_t ICR      = 0x000C0; //*
const uint32_t ITR      = 0x000C4;
const uint32_t ICS      = 0x000C8;
const uint32_t IMS      = 0x000D0;
const uint32_t IMC      = 0x000D8; //*
const uint32_t RCTL     = 0x00100; //*
const uint32_t RDBAL    = 0x02800;
const uint32_t RDBAH    = 0x02804;
const uint32_t RDLEN    = 0x02808;
const uint32_t RDH      = 0x02810;
const uint32_t RDT      = 0x02818;
const uint32_t RDTR     = 0x02820;
const uint32_t RADV     = 0x0282C;
const uint32_t RSRPD    = 0x02C00;
const uint32_t TCTL     = 0x00400; //*
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
const uint32_t MANC     = 0x05820;//*

const uint8_t EEPROM_READ_OPCODE_SPI    = 0x03;
const uint8_t EEPROM_RDSR_OPCODE_SPI    = 0x05;
const uint8_t EEPROM_SIZE               = 64;

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

struct Regs {
    union {  // 0x0000 CTRL Register
       uint32_t reg;
       struct {
           uint8_t fd:1;      // full duplex
           uint8_t bem:1;     // big endian mode
           uint8_t pcipr:1;   // PCI priority
           uint8_t lrst:1;    // link reset
           uint8_t tme:1;     // test mode enable
           uint8_t asde:1;    // Auto-speed detection
           uint8_t slu:1;     // Set link up
           uint8_t ilos:1;    // invert los-of-signal
           uint8_t speed:2;   // speed selection bits
           uint8_t be32:1;    // big endian mode 32
           uint8_t frcspd:1;  // force speed
           uint8_t frcdpx:1;  // force duplex
           uint8_t duden:1;   // dock/undock enable
           uint8_t dudpol:1;  // dock/undock polarity
           uint8_t fphyrst:1; // force phy reset
           uint8_t extlen:1;  // external link status enable
           uint8_t rsvd:1;    // reserved
           uint8_t sdp0d:1;   // software controlled pin data
           uint8_t sdp1d:1;   // software controlled pin data
           uint8_t sdp2d:1;   // software controlled pin data
           uint8_t sdp3d:1;   // software controlled pin data
           uint8_t sdp0i:1;   // software controlled pin dir
           uint8_t sdp1i:1;   // software controlled pin dir
           uint8_t sdp2i:1;   // software controlled pin dir
           uint8_t sdp3i:1;   // software controlled pin dir
           uint8_t rst:1;     // reset
           uint8_t rfce:1;    // receive flow control enable
           uint8_t tfce:1;    // transmit flow control enable
           uint8_t rte:1;     // routing tag enable
           uint8_t vme:1;     // vlan enable
           uint8_t phyrst:1;  // phy reset
       } ;
    } ctrl;

    union { // 0x0008 STATUS
        uint32_t reg;
        struct {
            uint8_t fd:1;      // full duplex
            uint8_t lu:1;      // link up
            uint8_t func:2;    // function id
            uint8_t txoff:1;   // transmission paused
            uint8_t tbimode:1; // tbi mode
            uint8_t speed:2;   // link speed
            uint8_t asdv:2;    // auto speed detection value
            uint8_t mtxckok:1; // mtx clock running ok
            uint8_t pci66:1;   // In 66Mhz pci slot
            uint8_t bus64:1;   // in 64 bit slot
            uint8_t pcix:1;    // Pci mode
            uint8_t pcixspd:1; // pci x speed
            uint8_t reserved;  // reserved
        } ;
    } sts;

    union { // 0x0010 EECD
        uint32_t reg;
        struct {
            uint8_t sk:1;      // clack input to the eeprom
            uint8_t cs:1;      // chip select to eeprom
            uint8_t din:1;     // data input to eeprom
            uint8_t dout:1;    // data output bit
            uint8_t fwe:2;     // flash write enable
            uint8_t ee_req:1;  // request eeprom access
            uint8_t ee_gnt:1;  // grant eeprom access
            uint8_t ee_pres:1; // eeprom present
            uint8_t ee_size:1; // eeprom size
            uint8_t ee_sz1:1;  // eeprom size
            uint8_t rsvd:2;    // reserved
            uint8_t ee_type:1; // type of eeprom
        } ;
    } eecd;

    union { // 0x0014 EERD
        uint32_t reg;
        struct {
            uint8_t start:1;  // start read
            uint8_t done:1;   // done read
            uint16_t addr:14; // address
            uint16_t data;    // data
        };
    } eerd;

    union { // 0x00C0 ICR
        uint32_t reg;
        struct {
            uint8_t txdw:1;   // tx descr witten back
            uint8_t txqe:1;   // tx queue empty
            uint8_t lsc:1;    // link status change
            uint8_t rxseq:1;  // rcv sequence error
            uint8_t rxdmt0:1; // rcv descriptor min thresh
            uint8_t rsvd1:1;  // reserved
            uint8_t rxo:1;    // receive overrunn
            uint8_t rxt0:1;   // receiver timer interrupt
            uint8_t rsvd2:1;  // reserved
            uint8_t mdac:1;   // mdi/o access complete
            uint8_t rxcfg:1;  // recv /c/ ordered sets
            uint8_t rsvd3:1;  // reserved
            uint8_t phyint:1; // phy interrupt
            uint8_t gpi1:1;   // gpi int 1
            uint8_t gpi2:1;   // gpi int 2
            uint8_t txdlow:1; // transmit desc low thresh
            uint8_t srpd:1;   // small receive packet detected
            uint16_t rsvd4:15; // reserved
        } ;
    } icd;

    union { // 0x00C0 IMC
        uint32_t reg;
        struct {
            uint8_t txdw:1;   // tx descr witten back
            uint8_t txqe:1;   // tx queue empty
            uint8_t lsc:1;    // link status change
            uint8_t rxseq:1;  // rcv sequence error
            uint8_t rxdmt0:1; // rcv descriptor min thresh
            uint8_t rsvd1:1;  // reserved
            uint8_t rxo:1;    // receive overrunn
            uint8_t rxt0:1;   // receiver timer interrupt
            uint8_t rsvd2:1;  // reserved
            uint8_t mdac:1;   // mdi/o access complete
            uint8_t rxcfg:1;  // recv /c/ ordered sets
            uint8_t rsvd3:1;  // reserved
            uint8_t phyint:1; // phy interrupt
            uint8_t gpi1:1;   // gpi int 1
            uint8_t gpi2:1;   // gpi int 2
            uint8_t txdlow:1; // transmit desc low thresh
            uint8_t srpd:1;   // small receive packet detected
            uint16_t rsvd4:15; // reserved
        } ;
    } imc;

    union { // 0x0100 RCTL
        uint32_t reg;
        struct {
            uint8_t rst:1;   // Reset
            uint8_t en:1;    // Enable
            uint8_t sbp:1;   // Store bad packets
            uint8_t upe:1;   // Unicast Promiscuous enabled
            uint8_t mpe:1;   // Multicast promiscuous enabled
            uint8_t lpe:1;   // long packet reception enabled
            uint8_t lbm:2;   //
            uint8_t rdmts:2; //
            uint8_t rsvd:2;  //
            uint8_t mo:2;    //
            uint8_t mdr:1;   //
            uint8_t bam:1;   //
            uint8_t bsize:2; //
            uint8_t vpe:1;   //
            uint8_t cfien:1; //
            uint8_t cfi:1;   //
            uint8_t rsvd2:1; //
            uint8_t dpf:1;   // discard pause frames
            uint8_t pmcf:1;  // pass mac control  frames
            uint8_t rsvd3:1; // reserved
            uint8_t bsex:1;  // buffer size extension
            uint8_t secrc:1; // strip ethernet crc from incoming packet
            uint8_t rsvd1:5;  // reserved
        } ;
    } rctl;

    union { // 0x0400 TCTL
        uint32_t reg;
        struct {
            uint8_t rst:1;    // Reset
            uint8_t en:1;     // Enable
            uint8_t bce:1;    // busy check enable
            uint8_t psp:1;    // pad short packets
            uint8_t ct:8;     // collision threshold
            uint16_t cold:10; // collision distance
            uint8_t swxoff:1; // software xoff transmission
            uint8_t pbe:1;    // packet burst enable
            uint8_t rtlc:1;   // retransmit late collisions
            uint8_t nrtu:1;   // on underrun no TX
            uint8_t mulr:1;   // multiple request
            uint8_t rsvd:5;   // reserved
        } ;
    } tctl;

    union { // 0x5820 MANC
        uint32_t reg;
        struct {
            uint8_t smbus:1;    // SMBus enabled #####
            uint8_t asf:1;      // ASF enabled #####
            uint8_t ronforce:1; // reset of force
            uint8_t rsvd:5;     // reserved
            uint8_t rmcp1:1;    // rcmp1 filtering
            uint8_t rmcp2:1;    // rcmp2 filtering
            uint8_t ipv4:1;     // enable ipv4
            uint8_t ipv6:1;     // enable ipv6
            uint8_t snap:1;     // accept snap
            uint8_t arp:1;      // filter arp #####
            uint8_t neighbor:1; // neighbor discovery
            uint8_t arp_resp:1; // arp response
            uint8_t tcorst:1;   // tco reset happened
            uint8_t rcvtco:1;   // receive tco enabled ######
            uint8_t blkphyrst:1;// block phy resets ########
            uint8_t rcvall:1;   // receive all
            uint8_t macaddrfltr:1; // mac address filtering ######
            uint8_t mng2host:1; // mng2 host packets #######
            uint8_t ipaddrfltr:1; // ip address filtering
            uint8_t xsumfilter:1; // checksum filtering
            uint8_t brfilter:1; // broadcast filtering
            uint8_t smbreq:1;   // smb request
            uint8_t smbgnt:1;   // smb grant
            uint8_t smbclkin:1; // smbclkin
            uint8_t smbdatain:1; // smbdatain
            uint8_t smbdataout:1; // smb data out
            uint8_t smbclkout:1; // smb clock out
            uint8_t rsvd2:2;
        };
    } manc;
};

}; // iGbReg namespace
