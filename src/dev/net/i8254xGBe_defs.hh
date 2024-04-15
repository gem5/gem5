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
 */

/* @file
 * Register and structure descriptions for Intel's 8254x line of gigabit
 * ethernet controllers.
 */
#include "base/bitfield.hh"
#include "base/compiler.hh"

namespace gem5
{

namespace igbreg
{

// Registers used by the Intel GbE NIC
const uint32_t REG_CTRL = 0x00000;
const uint32_t REG_STATUS = 0x00008;
const uint32_t REG_EECD = 0x00010;
const uint32_t REG_EERD = 0x00014;
const uint32_t REG_CTRL_EXT = 0x00018;
const uint32_t REG_MDIC = 0x00020;
const uint32_t REG_FCAL = 0x00028;
const uint32_t REG_FCAH = 0x0002C;
const uint32_t REG_FCT = 0x00030;
const uint32_t REG_VET = 0x00038;
const uint32_t REG_PBA = 0x01000;
const uint32_t REG_ICR = 0x000C0;
const uint32_t REG_ITR = 0x000C4;
const uint32_t REG_ICS = 0x000C8;
const uint32_t REG_IMS = 0x000D0;
const uint32_t REG_IMC = 0x000D8;
const uint32_t REG_IAM = 0x000E0;
const uint32_t REG_RCTL = 0x00100;
const uint32_t REG_FCTTV = 0x00170;
const uint32_t REG_TIPG = 0x00410;
const uint32_t REG_AIFS = 0x00458;
const uint32_t REG_LEDCTL = 0x00e00;
const uint32_t REG_EICR = 0x01580;
const uint32_t REG_IVAR0 = 0x01700;
const uint32_t REG_FCRTL = 0x02160;
const uint32_t REG_FCRTH = 0x02168;
const uint32_t REG_RDBAL = 0x02800;
const uint32_t REG_RDBAH = 0x02804;
const uint32_t REG_RDLEN = 0x02808;
const uint32_t REG_SRRCTL = 0x0280C;
const uint32_t REG_RDH = 0x02810;
const uint32_t REG_RDT = 0x02818;
const uint32_t REG_RDTR = 0x02820;
const uint32_t REG_RXDCTL = 0x02828;
const uint32_t REG_RADV = 0x0282C;
const uint32_t REG_TCTL = 0x00400;
const uint32_t REG_TDBAL = 0x03800;
const uint32_t REG_TDBAH = 0x03804;
const uint32_t REG_TDLEN = 0x03808;
const uint32_t REG_TDH = 0x03810;
const uint32_t REG_TXDCA_CTL = 0x03814;
const uint32_t REG_TDT = 0x03818;
const uint32_t REG_TIDV = 0x03820;
const uint32_t REG_TXDCTL = 0x03828;
const uint32_t REG_TADV = 0x0382C;
const uint32_t REG_TDWBAL = 0x03838;
const uint32_t REG_TDWBAH = 0x0383C;
const uint32_t REG_CRCERRS = 0x04000;
const uint32_t REG_RXCSUM = 0x05000;
const uint32_t REG_RLPML = 0x05004;
const uint32_t REG_RFCTL = 0x05008;
const uint32_t REG_MTA = 0x05200;
const uint32_t REG_RAL = 0x05400;
const uint32_t REG_RAH = 0x05404;
const uint32_t REG_VFTA = 0x05600;

const uint32_t REG_WUC = 0x05800;
const uint32_t REG_WUFC = 0x05808;
const uint32_t REG_WUS = 0x05810;
const uint32_t REG_MANC = 0x05820;
const uint32_t REG_SWSM = 0x05B50;
const uint32_t REG_FWSM = 0x05B54;
const uint32_t REG_SWFWSYNC = 0x05B5C;

const uint8_t EEPROM_READ_OPCODE_SPI = 0x03;
const uint8_t EEPROM_RDSR_OPCODE_SPI = 0x05;
const uint8_t EEPROM_SIZE = 64;
const uint16_t EEPROM_CSUM = 0xBABA;

const uint8_t VLAN_FILTER_TABLE_SIZE = 128;
const uint8_t RCV_ADDRESS_TABLE_SIZE = 24;
const uint8_t MULTICAST_TABLE_SIZE = 128;
const uint32_t STATS_REGS_SIZE = 0x228;

// Registers in that are accessed in the PHY
const uint8_t PHY_PSTATUS = 0x1;
const uint8_t PHY_PID = 0x2;
const uint8_t PHY_EPID = 0x3;
const uint8_t PHY_GSTATUS = 10;
const uint8_t PHY_EPSTATUS = 15;
const uint8_t PHY_AGC = 18;

// Receive Descriptor Status Flags
const uint16_t RXDS_DYNINT = 0x800;
const uint16_t RXDS_UDPV = 0x400;
const uint16_t RXDS_CRCV = 0x100;
const uint16_t RXDS_PIF = 0x080;
const uint16_t RXDS_IPCS = 0x040;
const uint16_t RXDS_TCPCS = 0x020;
const uint16_t RXDS_UDPCS = 0x010;
const uint16_t RXDS_VP = 0x008;
const uint16_t RXDS_IXSM = 0x004;
const uint16_t RXDS_EOP = 0x002;
const uint16_t RXDS_DD = 0x001;

// Receive Descriptor Error Flags
const uint8_t RXDE_RXE = 0x80;
const uint8_t RXDE_IPE = 0x40;
const uint8_t RXDE_TCPE = 0x20;
const uint8_t RXDE_SEQ = 0x04;
const uint8_t RXDE_SE = 0x02;
const uint8_t RXDE_CE = 0x01;

// Receive Descriptor Extended Error Flags
const uint16_t RXDEE_HBO = 0x008;
const uint16_t RXDEE_CE = 0x010;
const uint16_t RXDEE_LE = 0x020;
const uint16_t RXDEE_PE = 0x080;
const uint16_t RXDEE_OSE = 0x100;
const uint16_t RXDEE_USE = 0x200;
const uint16_t RXDEE_TCPE = 0x400;
const uint16_t RXDEE_IPE = 0x800;

// Receive Descriptor Types
const uint8_t RXDT_LEGACY = 0x00;
const uint8_t RXDT_ADV_ONEBUF = 0x01;
const uint8_t RXDT_ADV_SPLIT_A = 0x05;

// Receive Descriptor Packet Types
const uint16_t RXDP_IPV4 = 0x001;
const uint16_t RXDP_IPV4E = 0x002;
const uint16_t RXDP_IPV6 = 0x004;
const uint16_t RXDP_IPV6E = 0x008;
const uint16_t RXDP_TCP = 0x010;
const uint16_t RXDP_UDP = 0x020;
const uint16_t RXDP_SCTP = 0x040;
const uint16_t RXDP_NFS = 0x080;

// Interrupt types
enum IntTypes
{
    IT_NONE = 0x00000, // dummy value
    IT_TXDW = 0x00001,
    IT_TXQE = 0x00002,
    IT_LSC = 0x00004,
    IT_RXSEQ = 0x00008,
    IT_RXDMT = 0x00010,
    IT_RXO = 0x00040,
    IT_RXT = 0x00080,
    IT_MADC = 0x00200,
    IT_RXCFG = 0x00400,
    IT_GPI0 = 0x02000,
    IT_GPI1 = 0x04000,
    IT_TXDLOW = 0x08000,
    IT_SRPD = 0x10000,
    IT_ACK = 0x20000
};

// Receive Descriptor struct
struct RxDesc
{
    union
    {
        struct
        {
            Addr buf;
            uint16_t len;
            uint16_t csum;
            uint8_t status;
            uint8_t errors;
            uint16_t vlan;
        } legacy;

        struct
        {
            Addr pkt;
            Addr hdr;
        } adv_read;

        struct
        {
            uint16_t rss_type : 4;
            uint16_t pkt_type : 12;
            uint16_t __reserved1 : 5;
            uint16_t header_len : 10;
            uint16_t sph : 1;

            union
            {
                struct
                {
                    uint16_t id;
                    uint16_t csum;
                };

                uint32_t rss_hash;
            };

            uint32_t status : 20;
            uint32_t errors : 12;
            uint16_t pkt_len;
            uint16_t vlan_tag;
        } adv_wb;
    };
};

struct TxDesc
{
    uint64_t d1;
    uint64_t d2;
};

namespace txd_op
{

const uint8_t TXD_CNXT = 0x0;
const uint8_t TXD_DATA = 0x1;
const uint8_t TXD_ADVCNXT = 0x2;
const uint8_t TXD_ADVDATA = 0x3;

inline bool
isLegacy(TxDesc *d)
{
    return !bits(d->d2, 29);
}

inline uint8_t
getType(TxDesc *d)
{
    return bits(d->d2, 23, 20);
}

inline bool
isType(TxDesc *d, uint8_t type)
{
    return getType(d) == type;
}

inline bool
isTypes(TxDesc *d, uint8_t t1, uint8_t t2)
{
    return isType(d, t1) || isType(d, t2);
}

inline bool
isAdvDesc(TxDesc *d)
{
    return !isLegacy(d) && isTypes(d, TXD_ADVDATA, TXD_ADVCNXT);
}

inline bool
isContext(TxDesc *d)
{
    return !isLegacy(d) && isTypes(d, TXD_CNXT, TXD_ADVCNXT);
}

inline bool
isData(TxDesc *d)
{
    return !isLegacy(d) && isTypes(d, TXD_DATA, TXD_ADVDATA);
}

inline Addr
getBuf(TxDesc *d)
{
    assert(isLegacy(d) || isData(d));
    return d->d1;
}

inline Addr
getLen(TxDesc *d)
{
    if (isLegacy(d))
        return bits(d->d2, 15, 0);
    else
        return bits(d->d2, 19, 0);
}

inline void
setDd(TxDesc *d)
{
    replaceBits(d->d2, 35, 32, 1ULL);
}

inline bool
ide(TxDesc *d)
{
    return bits(d->d2, 31, 31) && (getType(d) == TXD_DATA || isLegacy(d));
}

inline bool
vle(TxDesc *d)
{
    assert(isLegacy(d) || isData(d));
    return bits(d->d2, 30);
}

inline bool
rs(TxDesc *d)
{
    return bits(d->d2, 27);
}

inline bool
ic(TxDesc *d)
{
    assert(isLegacy(d) || isData(d));
    return isLegacy(d) && bits(d->d2, 26);
}

inline bool
tse(TxDesc *d)
{
    if (isTypes(d, TXD_CNXT, TXD_DATA))
        return bits(d->d2, 26);
    if (isType(d, TXD_ADVDATA))
        return bits(d->d2, 31);
    return false;
}

inline bool
ifcs(TxDesc *d)
{
    assert(isLegacy(d) || isData(d));
    return bits(d->d2, 25);
}

inline bool
eop(TxDesc *d)
{
    assert(isLegacy(d) || isData(d));
    return bits(d->d2, 24);
}

inline bool
ip(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d2, 25);
}

inline bool
tcp(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d2, 24);
}

inline uint8_t
getCso(TxDesc *d)
{
    assert(isLegacy(d));
    return bits(d->d2, 23, 16);
}

inline uint8_t
getCss(TxDesc *d)
{
    assert(isLegacy(d));
    return bits(d->d2, 47, 40);
}

inline bool
ixsm(TxDesc *d)
{
    return isData(d) && bits(d->d2, 40, 40);
}

inline bool
txsm(TxDesc *d)
{
    return isData(d) && bits(d->d2, 41, 41);
}

inline int
tucse(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 63, 48);
}

inline int
tucso(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 47, 40);
}

inline int
tucss(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 39, 32);
}

inline int
ipcse(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 31, 16);
}

inline int
ipcso(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 15, 8);
}

inline int
ipcss(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d1, 7, 0);
}

inline int
mss(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d2, 63, 48);
}

inline int
hdrlen(TxDesc *d)
{
    assert(isContext(d));
    if (!isAdvDesc(d))
        return bits(d->d2, 47, 40);
    return bits(d->d2, 47, 40) + bits(d->d1, 8, 0) + bits(d->d1, 15, 9);
}

inline int
getTsoLen(TxDesc *d)
{
    assert(isType(d, TXD_ADVDATA));
    return bits(d->d2, 63, 46);
}

inline int
utcmd(TxDesc *d)
{
    assert(isContext(d));
    return bits(d->d2, 24, 31);
}

} // namespace txd_op

#define ADD_FIELD32(NAME, OFFSET, BITS)                                       \
    inline uint32_t NAME() { return bits(_data, OFFSET + BITS - 1, OFFSET); } \
    inline void NAME(uint32_t d)                                              \
    {                                                                         \
        replaceBits(_data, OFFSET + BITS - 1, OFFSET, d);                     \
    }

#define ADD_FIELD64(NAME, OFFSET, BITS)                                       \
    inline uint64_t NAME() { return bits(_data, OFFSET + BITS - 1, OFFSET); } \
    inline void NAME(uint64_t d)                                              \
    {                                                                         \
        replaceBits(_data, OFFSET + BITS - 1, OFFSET, d);                     \
    }

struct Regs : public Serializable
{
    template <class T>
    struct Reg
    {
        T _data;

        T
        operator()()
        {
            return _data;
        }

        const Reg<T> &
        operator=(T d)
        {
            _data = d;
            return *this;
        }

        bool
        operator==(T d)
        {
            return d == _data;
        }

        void
        operator()(T d)
        {
            _data = d;
        }

        Reg() { _data = 0; }

        void
        serialize(CheckpointOut &cp) const
        {
            SERIALIZE_SCALAR(_data);
        }

        void
        unserialize(CheckpointIn &cp)
        {
            UNSERIALIZE_SCALAR(_data);
        }
    };

    struct CTRL : public Reg<uint32_t>
    {
        // 0x0000 CTRL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(fd, 0, 1);       // full duplex
        ADD_FIELD32(bem, 1, 1);      // big endian mode
        ADD_FIELD32(pcipr, 2, 1);    // PCI priority
        ADD_FIELD32(lrst, 3, 1);     // link reset
        ADD_FIELD32(tme, 4, 1);      // test mode enable
        ADD_FIELD32(asde, 5, 1);     // Auto-speed detection
        ADD_FIELD32(slu, 6, 1);      // Set link up
        ADD_FIELD32(ilos, 7, 1);     // invert los-of-signal
        ADD_FIELD32(speed, 8, 2);    // speed selection bits
        ADD_FIELD32(be32, 10, 1);    // big endian mode 32
        ADD_FIELD32(frcspd, 11, 1);  // force speed
        ADD_FIELD32(frcdpx, 12, 1);  // force duplex
        ADD_FIELD32(duden, 13, 1);   // dock/undock enable
        ADD_FIELD32(dudpol, 14, 1);  // dock/undock polarity
        ADD_FIELD32(fphyrst, 15, 1); // force phy reset
        ADD_FIELD32(extlen, 16, 1);  // external link status enable
        ADD_FIELD32(rsvd, 17, 1);    // reserved
        ADD_FIELD32(sdp0d, 18, 1);   // software controlled pin data
        ADD_FIELD32(sdp1d, 19, 1);   // software controlled pin data
        ADD_FIELD32(sdp2d, 20, 1);   // software controlled pin data
        ADD_FIELD32(sdp3d, 21, 1);   // software controlled pin data
        ADD_FIELD32(sdp0i, 22, 1);   // software controlled pin dir
        ADD_FIELD32(sdp1i, 23, 1);   // software controlled pin dir
        ADD_FIELD32(sdp2i, 24, 1);   // software controlled pin dir
        ADD_FIELD32(sdp3i, 25, 1);   // software controlled pin dir
        ADD_FIELD32(rst, 26, 1);     // reset
        ADD_FIELD32(rfce, 27, 1);    // receive flow control enable
        ADD_FIELD32(tfce, 28, 1);    // transmit flow control enable
        ADD_FIELD32(rte, 29, 1);     // routing tag enable
        ADD_FIELD32(vme, 30, 1);     // vlan enable
        ADD_FIELD32(phyrst, 31, 1);  // phy reset
    };

    CTRL ctrl;

    struct STATUS : public Reg<uint32_t>
    {
        // 0x0008 STATUS Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(fd, 0, 1);       // full duplex
        ADD_FIELD32(lu, 1, 1);       // link up
        ADD_FIELD32(func, 2, 2);     // function id
        ADD_FIELD32(txoff, 4, 1);    // transmission paused
        ADD_FIELD32(tbimode, 5, 1);  // tbi mode
        ADD_FIELD32(speed, 6, 2);    // link speed
        ADD_FIELD32(asdv, 8, 2);     // auto speed detection value
        ADD_FIELD32(mtxckok, 10, 1); // mtx clock running ok
        ADD_FIELD32(pci66, 11, 1);   // In 66Mhz pci slot
        ADD_FIELD32(bus64, 12, 1);   // in 64 bit slot
        ADD_FIELD32(pcix, 13, 1);    // Pci mode
        ADD_FIELD32(pcixspd, 14, 2); // pci x speed
    };

    STATUS sts;

    struct EECD : public Reg<uint32_t>
    {
        // 0x0010 EECD Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(sk, 0, 1);       // clack input to the eeprom
        ADD_FIELD32(cs, 1, 1);       // chip select to eeprom
        ADD_FIELD32(din, 2, 1);      // data input to eeprom
        ADD_FIELD32(dout, 3, 1);     // data output bit
        ADD_FIELD32(fwe, 4, 2);      // flash write enable
        ADD_FIELD32(ee_req, 6, 1);   // request eeprom access
        ADD_FIELD32(ee_gnt, 7, 1);   // grant eeprom access
        ADD_FIELD32(ee_pres, 8, 1);  // eeprom present
        ADD_FIELD32(ee_size, 9, 1);  // eeprom size
        ADD_FIELD32(ee_sz1, 10, 1);  // eeprom size
        ADD_FIELD32(rsvd, 11, 2);    // reserved
        ADD_FIELD32(ee_type, 13, 1); // type of eeprom
    };

    EECD eecd;

    struct EERD : public Reg<uint32_t>
    {
        // 0x0014 EERD Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(start, 0, 1);  // start read
        ADD_FIELD32(done, 1, 1);   // done read
        ADD_FIELD32(addr, 2, 14);  // address
        ADD_FIELD32(data, 16, 16); // data
    };

    EERD eerd;

    struct CTRL_EXT : public Reg<uint32_t>
    {
        // 0x0018 CTRL_EXT Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(gpi_en, 0, 4);      // enable interrupts from gpio
        ADD_FIELD32(phyint, 5, 1);      // reads the phy internal int status
        ADD_FIELD32(sdp2_data, 6, 1);   // data from gpio sdp
        ADD_FIELD32(spd3_data, 7, 1);   // data frmo gpio sdp
        ADD_FIELD32(spd2_iodir, 10, 1); // direction of sdp2
        ADD_FIELD32(spd3_iodir, 11, 1); // direction of sdp2
        ADD_FIELD32(asdchk, 12, 1);     // initiate auto-speed-detection
        ADD_FIELD32(eerst, 13, 1);      // reset the eeprom
        ADD_FIELD32(spd_byps, 15, 1);   // bypass speed select
        ADD_FIELD32(ro_dis, 17, 1);     // disable relaxed memory ordering
        ADD_FIELD32(vreg, 21, 1);       // power down the voltage regulator
        ADD_FIELD32(link_mode, 22, 2);  // interface to talk to the link
        ADD_FIELD32(iame, 27, 1);       // interrupt acknowledge auto-mask ??
        ADD_FIELD32(drv_loaded, 28, 1); // driver is loaded and incharge of
                                        //   device
        ADD_FIELD32(timer_clr, 29, 1);  // clear interrupt timers after IMS
                                        //   clear ??
    };

    CTRL_EXT ctrl_ext;

    struct MDIC : public Reg<uint32_t>
    {
        // 0x0020 MDIC Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(data, 0, 16);   // data
        ADD_FIELD32(regadd, 16, 5); // register address
        ADD_FIELD32(phyadd, 21, 5); // phy addresses
        ADD_FIELD32(op, 26, 2);     // opcode
        ADD_FIELD32(r, 28, 1);      // ready
        ADD_FIELD32(i, 29, 1);      // interrupt
        ADD_FIELD32(e, 30, 1);      // error
    };

    MDIC mdic;

    struct ICR : public Reg<uint32_t>
    {
        // 0x00C0 ICR Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(txdw, 0, 1)         // tx descr witten back
        ADD_FIELD32(txqe, 1, 1)         // tx queue empty
        ADD_FIELD32(lsc, 2, 1)          // link status change
        ADD_FIELD32(rxseq, 3, 1)        // rcv sequence error
        ADD_FIELD32(rxdmt0, 4, 1)       // rcv descriptor min thresh
        ADD_FIELD32(rsvd1, 5, 1)        // reserved
        ADD_FIELD32(rxo, 6, 1)          // receive overrunn
        ADD_FIELD32(rxt0, 7, 1)         // receiver timer interrupt
        ADD_FIELD32(mdac, 9, 1)         // mdi/o access complete
        ADD_FIELD32(rxcfg, 10, 1)       // recv /c/ ordered sets
        ADD_FIELD32(phyint, 12, 1)      // phy interrupt
        ADD_FIELD32(gpi1, 13, 1)        // gpi int 1
        ADD_FIELD32(gpi2, 14, 1)        // gpi int 2
        ADD_FIELD32(txdlow, 15, 1)      // transmit desc low thresh
        ADD_FIELD32(srpd, 16, 1)        // small receive packet detected
        ADD_FIELD32(ack, 17, 1);        // receive ack frame
        ADD_FIELD32(int_assert, 31, 1); // interrupt caused a system interrupt
    };

    ICR icr;

    uint32_t imr; // register that contains the current interrupt mask

    struct ITR : public Reg<uint32_t>
    {
        // 0x00C4 ITR Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(interval, 0, 16); // minimum inter-interrutp inteval
                                      // specified in 256ns interrupts
    };

    ITR itr;

    // When CTRL_EXT.IAME and the ICR.INT_ASSERT is 1 an ICR read or write
    // causes the IAM register contents to be written into the IMC
    // automatically clearing all interrupts that have a bit in the IAM set
    uint32_t iam;

    struct RCTL : public Reg<uint32_t>
    {
        // 0x0100 RCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rst, 0, 1);    // Reset
        ADD_FIELD32(en, 1, 1);     // Enable
        ADD_FIELD32(sbp, 2, 1);    // Store bad packets
        ADD_FIELD32(upe, 3, 1);    // Unicast Promiscuous enabled
        ADD_FIELD32(mpe, 4, 1);    // Multicast promiscuous enabled
        ADD_FIELD32(lpe, 5, 1);    // long packet reception enabled
        ADD_FIELD32(lbm, 6, 2);    //
        ADD_FIELD32(rdmts, 8, 2);  //
        ADD_FIELD32(mo, 12, 2);    //
        ADD_FIELD32(mdr, 14, 1);   //
        ADD_FIELD32(bam, 15, 1);   //
        ADD_FIELD32(bsize, 16, 2); //
        ADD_FIELD32(vfe, 18, 1);   //
        ADD_FIELD32(cfien, 19, 1); //
        ADD_FIELD32(cfi, 20, 1);   //
        ADD_FIELD32(dpf, 22, 1);   // discard pause frames
        ADD_FIELD32(pmcf, 23, 1);  // pass mac control  frames
        ADD_FIELD32(bsex, 25, 1);  // buffer size extension
        ADD_FIELD32(secrc, 26, 1); // strip ethernet crc from incoming packet

        unsigned
        descSize()
        {
            switch (bsize()) {
            case 0:
                return bsex() == 0 ? 2048 : 0;
            case 1:
                return bsex() == 0 ? 1024 : 16384;
            case 2:
                return bsex() == 0 ? 512 : 8192;
            case 3:
                return bsex() == 0 ? 256 : 4096;
            default:
                return 0;
            }
        }
    };

    RCTL rctl;

    struct FCTTV : public Reg<uint32_t>
    {
        // 0x0170 FCTTV
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(ttv, 0, 16); // Transmit Timer Value
    };

    FCTTV fcttv;

    struct TCTL : public Reg<uint32_t>
    {
        // 0x0400 TCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rst, 0, 1);     // Reset
        ADD_FIELD32(en, 1, 1);      // Enable
        ADD_FIELD32(bce, 2, 1);     // busy check enable
        ADD_FIELD32(psp, 3, 1);     // pad short packets
        ADD_FIELD32(ct, 4, 8);      // collision threshold
        ADD_FIELD32(cold, 12, 10);  // collision distance
        ADD_FIELD32(swxoff, 22, 1); // software xoff transmission
        ADD_FIELD32(pbe, 23, 1);    // packet burst enable
        ADD_FIELD32(rtlc, 24, 1);   // retransmit late collisions
        ADD_FIELD32(nrtu, 25, 1);   // on underrun no TX
        ADD_FIELD32(mulr, 26, 1);   // multiple request
    };

    TCTL tctl;

    struct PBA : public Reg<uint32_t>
    {
        // 0x1000 PBA Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rxa, 0, 16);
        ADD_FIELD32(txa, 16, 16);
    };

    PBA pba;

    struct FCRTL : public Reg<uint32_t>
    {
        // 0x2160 FCRTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rtl, 3, 28); // make this bigger than the spec so we can
                                 // have a larger buffer
        ADD_FIELD32(xone, 31, 1);
    };

    FCRTL fcrtl;

    struct FCRTH : public Reg<uint32_t>
    {
        // 0x2168 FCRTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rth, 3, 13); // make this bigger than the spec so we can
                                 // have a larger buffer
        ADD_FIELD32(xfce, 31, 1);
    };

    FCRTH fcrth;

    struct RDBA : public Reg<uint64_t>
    {
        // 0x2800 RDBA Register
        using Reg<uint64_t>::operator=;
        ADD_FIELD64(rdbal, 0, 32);  // base address of rx descriptor ring
        ADD_FIELD64(rdbah, 32, 32); // base address of rx descriptor ring
    };

    RDBA rdba;

    struct RDLEN : public Reg<uint32_t>
    {
        // 0x2808 RDLEN Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(len, 7, 13); // number of bytes in the descriptor buffer
    };

    RDLEN rdlen;

    struct SRRCTL : public Reg<uint32_t>
    {
        // 0x280C SRRCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(pktlen, 0, 8);
        ADD_FIELD32(hdrlen, 8, 8);    // guess based on header, not documented
        ADD_FIELD32(desctype, 25, 3); // type of descriptor 000 legacy,
                                      // 001 adv, 101 hdr split

        unsigned
        bufLen()
        {
            return pktlen() << 10;
        }

        unsigned
        hdrLen()
        {
            return hdrlen() << 6;
        }
    };

    SRRCTL srrctl;

    struct RDH : public Reg<uint32_t>
    {
        // 0x2810 RDH Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rdh, 0, 16); // head of the descriptor ring
    };

    RDH rdh;

    struct RDT : public Reg<uint32_t>
    {
        // 0x2818 RDT Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(rdt, 0, 16); // tail of the descriptor ring
    };

    RDT rdt;

    struct RDTR : public Reg<uint32_t>
    {
        // 0x2820 RDTR Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(delay, 0, 16); // receive delay timer
        ADD_FIELD32(fpd, 31, 1);   // flush partial descriptor block ??
    };

    RDTR rdtr;

    struct RXDCTL : public Reg<uint32_t>
    {
        // 0x2828 RXDCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(pthresh, 0, 6);  // prefetch threshold, less that this
                                     // consider prefetch
        ADD_FIELD32(hthresh, 8, 6);  // number of descriptors in host mem to
                                     // consider prefetch
        ADD_FIELD32(wthresh, 16, 6); // writeback threshold
        ADD_FIELD32(gran, 24, 1);    // granularity 0 = desc, 1 = cacheline
    };

    RXDCTL rxdctl;

    struct RADV : public Reg<uint32_t>
    {
        // 0x282C RADV Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(idv, 0, 16); // absolute interrupt delay
    };

    RADV radv;

    struct RSRPD : public Reg<uint32_t>
    {
        // 0x2C00 RSRPD Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(idv, 0, 12); // size to interrutp on small packets
    };

    RSRPD rsrpd;

    struct TDBA : public Reg<uint64_t>
    {
        // 0x3800 TDBAL Register
        using Reg<uint64_t>::operator=;
        ADD_FIELD64(tdbal, 0, 32);  // base address of transmit descriptor ring
        ADD_FIELD64(tdbah, 32, 32); // base address of transmit descriptor ring
    };

    TDBA tdba;

    struct TDLEN : public Reg<uint32_t>
    {
        // 0x3808 TDLEN Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(len, 7, 13); // number of bytes in the descriptor buffer
    };

    TDLEN tdlen;

    struct TDH : public Reg<uint32_t>
    {
        // 0x3810 TDH Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(tdh, 0, 16); // head of the descriptor ring
    };

    TDH tdh;

    struct TXDCA_CTL : public Reg<uint32_t>
    {
        // 0x3814 TXDCA_CTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(cpu_mask, 0, 5);
        ADD_FIELD32(enabled, 5, 1);
        ADD_FIELD32(relax_ordering, 6, 1);
    };

    TXDCA_CTL txdca_ctl;

    struct TDT : public Reg<uint32_t>
    {
        // 0x3818 TDT Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(tdt, 0, 16); // tail of the descriptor ring
    };

    TDT tdt;

    struct TIDV : public Reg<uint32_t>
    {
        // 0x3820 TIDV Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(idv, 0, 16); // interrupt delay
    };

    TIDV tidv;

    struct TXDCTL : public Reg<uint32_t>
    {
        // 0x3828 TXDCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(pthresh, 0, 6);  // if number of descriptors control has is
                                     // below this number, a prefetch is
                                     // considered
        ADD_FIELD32(hthresh, 8, 8);  // number of valid descriptors is host
                                     // memory before a prefetch is considered
        ADD_FIELD32(wthresh, 16, 6); // number of descriptors to keep until
                                     // writeback is considered
        ADD_FIELD32(gran, 24, 1);    // granulatiry of above values
                                     // (0 = cacheline, 1 == desscriptor)
        ADD_FIELD32(lwthresh, 25, 7); // xmit descriptor low thresh, interrupt
                                      // below this level
    };

    TXDCTL txdctl;

    struct TADV : public Reg<uint32_t>
    {
        // 0x382C TADV Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(idv, 0, 16); // absolute interrupt delay
    };

    TADV tadv;
    uint64_t tdwba;

    struct RXCSUM : public Reg<uint32_t>
    {
        // 0x5000 RXCSUM Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(pcss, 0, 8);
        ADD_FIELD32(ipofld, 8, 1);
        ADD_FIELD32(tuofld, 9, 1);
        ADD_FIELD32(pcsd, 13, 1);
    };

    RXCSUM rxcsum;

    uint32_t rlpml; // 0x5004 RLPML probably maximum accepted packet size

    struct RFCTL : public Reg<uint32_t>
    {
        // 0x5008 RFCTL Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(iscsi_dis, 0, 1);
        ADD_FIELD32(iscsi_dwc, 1, 5);
        ADD_FIELD32(nfsw_dis, 6, 1);
        ADD_FIELD32(nfsr_dis, 7, 1);
        ADD_FIELD32(nfs_ver, 8, 2);
        ADD_FIELD32(ipv6_dis, 10, 1);
        ADD_FIELD32(ipv6xsum_dis, 11, 1);
        ADD_FIELD32(ackdis, 13, 1);
        ADD_FIELD32(ipfrsp_dis, 14, 1);
        ADD_FIELD32(exsten, 15, 1);
    };

    RFCTL rfctl;

    struct MANC : public Reg<uint32_t>
    {
        // 0x5820 MANC Register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(smbus, 0, 1);        // SMBus enabled #####
        ADD_FIELD32(asf, 1, 1);          // ASF enabled #####
        ADD_FIELD32(ronforce, 2, 1);     // reset of force
        ADD_FIELD32(rsvd, 3, 5);         // reserved
        ADD_FIELD32(rmcp1, 8, 1);        // rcmp1 filtering
        ADD_FIELD32(rmcp2, 9, 1);        // rcmp2 filtering
        ADD_FIELD32(ipv4, 10, 1);        // enable ipv4
        ADD_FIELD32(ipv6, 11, 1);        // enable ipv6
        ADD_FIELD32(snap, 12, 1);        // accept snap
        ADD_FIELD32(arp, 13, 1);         // filter arp #####
        ADD_FIELD32(neighbor, 14, 1);    // neighbor discovery
        ADD_FIELD32(arp_resp, 15, 1);    // arp response
        ADD_FIELD32(tcorst, 16, 1);      // tco reset happened
        ADD_FIELD32(rcvtco, 17, 1);      // receive tco enabled ######
        ADD_FIELD32(blkphyrst, 18, 1);   // block phy resets ########
        ADD_FIELD32(rcvall, 19, 1);      // receive all
        ADD_FIELD32(macaddrfltr, 20, 1); // mac address filtering ######
        ADD_FIELD32(mng2host, 21, 1);    // mng2 host packets #######
        ADD_FIELD32(ipaddrfltr, 22, 1);  // ip address filtering
        ADD_FIELD32(xsumfilter, 23, 1);  // checksum filtering
        ADD_FIELD32(brfilter, 24, 1);    // broadcast filtering
        ADD_FIELD32(smbreq, 25, 1);      // smb request
        ADD_FIELD32(smbgnt, 26, 1);      // smb grant
        ADD_FIELD32(smbclkin, 27, 1);    // smbclkin
        ADD_FIELD32(smbdatain, 28, 1);   // smbdatain
        ADD_FIELD32(smbdataout, 29, 1);  // smb data out
        ADD_FIELD32(smbclkout, 30, 1);   // smb clock out
    };

    MANC manc;

    struct SWSM : public Reg<uint32_t>
    {
        // 0x5B50 SWSM register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(smbi, 0, 1);    // Semaphone bit
        ADD_FIELD32(swesmbi, 1, 1); // Software eeporm semaphore
        ADD_FIELD32(wmng, 2, 1);    // Wake MNG clock
        ADD_FIELD32(reserved, 3, 29);
    };

    SWSM swsm;

    struct FWSM : public Reg<uint32_t>
    {
        // 0x5B54 FWSM register
        using Reg<uint32_t>::operator=;
        ADD_FIELD32(eep_fw_semaphore, 0, 1);
        ADD_FIELD32(fw_mode, 1, 3);
        ADD_FIELD32(ide, 4, 1);
        ADD_FIELD32(sol, 5, 1);
        ADD_FIELD32(eep_roload, 6, 1);
        ADD_FIELD32(reserved, 7, 8);
        ADD_FIELD32(fw_val_bit, 15, 1);
        ADD_FIELD32(reset_cnt, 16, 3);
        ADD_FIELD32(ext_err_ind, 19, 6);
        ADD_FIELD32(reserved2, 25, 7);
    };

    FWSM fwsm;

    uint32_t sw_fw_sync;

    void
    serialize(CheckpointOut &cp) const override
    {
        paramOut(cp, "ctrl", ctrl._data);
        paramOut(cp, "sts", sts._data);
        paramOut(cp, "eecd", eecd._data);
        paramOut(cp, "eerd", eerd._data);
        paramOut(cp, "ctrl_ext", ctrl_ext._data);
        paramOut(cp, "mdic", mdic._data);
        paramOut(cp, "icr", icr._data);
        SERIALIZE_SCALAR(imr);
        paramOut(cp, "itr", itr._data);
        SERIALIZE_SCALAR(iam);
        paramOut(cp, "rctl", rctl._data);
        paramOut(cp, "fcttv", fcttv._data);
        paramOut(cp, "tctl", tctl._data);
        paramOut(cp, "pba", pba._data);
        paramOut(cp, "fcrtl", fcrtl._data);
        paramOut(cp, "fcrth", fcrth._data);
        paramOut(cp, "rdba", rdba._data);
        paramOut(cp, "rdlen", rdlen._data);
        paramOut(cp, "srrctl", srrctl._data);
        paramOut(cp, "rdh", rdh._data);
        paramOut(cp, "rdt", rdt._data);
        paramOut(cp, "rdtr", rdtr._data);
        paramOut(cp, "rxdctl", rxdctl._data);
        paramOut(cp, "radv", radv._data);
        paramOut(cp, "rsrpd", rsrpd._data);
        paramOut(cp, "tdba", tdba._data);
        paramOut(cp, "tdlen", tdlen._data);
        paramOut(cp, "tdh", tdh._data);
        paramOut(cp, "txdca_ctl", txdca_ctl._data);
        paramOut(cp, "tdt", tdt._data);
        paramOut(cp, "tidv", tidv._data);
        paramOut(cp, "txdctl", txdctl._data);
        paramOut(cp, "tadv", tadv._data);
        // paramOut(cp, "tdwba", tdwba._data);
        SERIALIZE_SCALAR(tdwba);
        paramOut(cp, "rxcsum", rxcsum._data);
        SERIALIZE_SCALAR(rlpml);
        paramOut(cp, "rfctl", rfctl._data);
        paramOut(cp, "manc", manc._data);
        paramOut(cp, "swsm", swsm._data);
        paramOut(cp, "fwsm", fwsm._data);
        SERIALIZE_SCALAR(sw_fw_sync);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        paramIn(cp, "ctrl", ctrl._data);
        paramIn(cp, "sts", sts._data);
        paramIn(cp, "eecd", eecd._data);
        paramIn(cp, "eerd", eerd._data);
        paramIn(cp, "ctrl_ext", ctrl_ext._data);
        paramIn(cp, "mdic", mdic._data);
        paramIn(cp, "icr", icr._data);
        UNSERIALIZE_SCALAR(imr);
        paramIn(cp, "itr", itr._data);
        UNSERIALIZE_SCALAR(iam);
        paramIn(cp, "rctl", rctl._data);
        paramIn(cp, "fcttv", fcttv._data);
        paramIn(cp, "tctl", tctl._data);
        paramIn(cp, "pba", pba._data);
        paramIn(cp, "fcrtl", fcrtl._data);
        paramIn(cp, "fcrth", fcrth._data);
        paramIn(cp, "rdba", rdba._data);
        paramIn(cp, "rdlen", rdlen._data);
        paramIn(cp, "srrctl", srrctl._data);
        paramIn(cp, "rdh", rdh._data);
        paramIn(cp, "rdt", rdt._data);
        paramIn(cp, "rdtr", rdtr._data);
        paramIn(cp, "rxdctl", rxdctl._data);
        paramIn(cp, "radv", radv._data);
        paramIn(cp, "rsrpd", rsrpd._data);
        paramIn(cp, "tdba", tdba._data);
        paramIn(cp, "tdlen", tdlen._data);
        paramIn(cp, "tdh", tdh._data);
        paramIn(cp, "txdca_ctl", txdca_ctl._data);
        paramIn(cp, "tdt", tdt._data);
        paramIn(cp, "tidv", tidv._data);
        paramIn(cp, "txdctl", txdctl._data);
        paramIn(cp, "tadv", tadv._data);
        UNSERIALIZE_SCALAR(tdwba);
        // paramIn(cp, "tdwba", tdwba._data);
        paramIn(cp, "rxcsum", rxcsum._data);
        UNSERIALIZE_SCALAR(rlpml);
        paramIn(cp, "rfctl", rfctl._data);
        paramIn(cp, "manc", manc._data);
        paramIn(cp, "swsm", swsm._data);
        paramIn(cp, "fwsm", fwsm._data);
        UNSERIALIZE_SCALAR(sw_fw_sync);
    }
};

} // namespace igbreg
} // namespace gem5
