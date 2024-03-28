/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __DEV_NET_SINICREG_HH__
#define __DEV_NET_SINICREG_HH__

#include <cstdint>

#include "base/compiler.hh"

#define __SINIC_REG32(NAME, VAL) static const uint32_t NAME = (VAL);
#define __SINIC_REG64(NAME, VAL) static const uint64_t NAME = (VAL);

#define __SINIC_VAL32(NAME, OFFSET, WIDTH) \
        static const uint32_t NAME##_width = WIDTH; \
        static const uint32_t NAME##_offset = OFFSET; \
        static const uint32_t NAME##_mask = (1 << WIDTH) - 1; \
        static const uint32_t NAME = ((1 << WIDTH) - 1) << OFFSET; \
        static inline uint32_t get_##NAME(uint32_t reg) \
        { return (reg & NAME) >> OFFSET; } \
        static inline uint32_t set_##NAME(uint32_t reg, uint32_t val) \
        { return (reg & ~NAME) | ((val << OFFSET) & NAME); }

#define __SINIC_VAL64(NAME, OFFSET, WIDTH) \
        static const uint64_t NAME##_width = WIDTH; \
        static const uint64_t NAME##_offset = OFFSET; \
        static const uint64_t NAME##_mask = (1ULL << WIDTH) - 1; \
        static const uint64_t NAME = ((1ULL << WIDTH) - 1) << OFFSET; \
        static inline uint64_t get_##NAME(uint64_t reg) \
        { return (reg & NAME) >> OFFSET; } \
        static inline uint64_t set_##NAME(uint64_t reg, uint64_t val) \
        { return (reg & ~NAME) | ((val << OFFSET) & NAME); }

namespace gem5
{

namespace sinic
{

namespace registers
{

static const int VirtualShift = 8;
static const int VirtualMask = 0xff;

// Registers
__SINIC_REG32(Config,        0x00) // 32: configuration register
__SINIC_REG32(Command,       0x04) // 32: command register
__SINIC_REG32(IntrStatus,    0x08) // 32: interrupt status
__SINIC_REG32(IntrMask,      0x0c) // 32: interrupt mask
__SINIC_REG32(RxMaxCopy,     0x10) // 32: max bytes per rx copy
__SINIC_REG32(TxMaxCopy,     0x14) // 32: max bytes per tx copy
__SINIC_REG32(ZeroCopySize,  0x18) // 32: bytes to copy if below threshold
__SINIC_REG32(ZeroCopyMark,  0x1c) // 32: only zero-copy above this threshold
__SINIC_REG32(VirtualCount,  0x20) // 32: number of virutal NICs
__SINIC_REG32(RxMaxIntr,     0x24) // 32: max receives per interrupt
__SINIC_REG32(RxFifoSize,    0x28) // 32: rx fifo capacity in bytes
__SINIC_REG32(TxFifoSize,    0x2c) // 32: tx fifo capacity in bytes
__SINIC_REG32(RxFifoLow,     0x30) // 32: rx fifo low watermark
__SINIC_REG32(TxFifoLow,     0x34) // 32: tx fifo low watermark
__SINIC_REG32(RxFifoHigh,    0x38) // 32: rx fifo high watermark
__SINIC_REG32(TxFifoHigh,    0x3c) // 32: tx fifo high watermark
__SINIC_REG32(RxData,        0x40) // 64: receive data
__SINIC_REG32(RxDone,        0x48) // 64: receive done
__SINIC_REG32(RxWait,        0x50) // 64: receive done (busy wait)
__SINIC_REG32(TxData,        0x58) // 64: transmit data
__SINIC_REG32(TxDone,        0x60) // 64: transmit done
__SINIC_REG32(TxWait,        0x68) // 64: transmit done (busy wait)
__SINIC_REG32(HwAddr,        0x70) // 64: mac address
__SINIC_REG32(RxStatus,      0x78)
__SINIC_REG32(Size,          0x80) // register addres space size

// Config register bits
__SINIC_VAL32(Config_ZeroCopy, 12, 1) // enable zero copy
__SINIC_VAL32(Config_DelayCopy,11, 1) // enable delayed copy
__SINIC_VAL32(Config_RSS,      10, 1) // enable receive side scaling
__SINIC_VAL32(Config_RxThread,  9, 1) // enable receive threads
__SINIC_VAL32(Config_TxThread,  8, 1) // enable transmit thread
__SINIC_VAL32(Config_Filter,    7, 1) // enable receive filter
__SINIC_VAL32(Config_Vlan,      6, 1) // enable vlan tagging
__SINIC_VAL32(Config_Vaddr,     5, 1) // enable virtual addressing
__SINIC_VAL32(Config_Desc,      4, 1) // enable tx/rx descriptors
__SINIC_VAL32(Config_Poll,      3, 1) // enable polling
__SINIC_VAL32(Config_IntEn,     2, 1) // enable interrupts
__SINIC_VAL32(Config_TxEn,      1, 1) // enable transmit
__SINIC_VAL32(Config_RxEn,      0, 1) // enable receive

// Command register bits
__SINIC_VAL32(Command_Intr,  1, 1) // software interrupt
__SINIC_VAL32(Command_Reset, 0, 1) // reset chip

// Interrupt register bits
__SINIC_VAL32(Intr_Soft,      8, 1) // software interrupt
__SINIC_VAL32(Intr_TxLow,     7, 1) // tx fifo dropped below watermark
__SINIC_VAL32(Intr_TxFull,    6, 1) // tx fifo full
__SINIC_VAL32(Intr_TxDMA,     5, 1) // tx dma completed w/ interrupt
__SINIC_VAL32(Intr_TxPacket,  4, 1) // packet transmitted
__SINIC_VAL32(Intr_RxHigh,    3, 1) // rx fifo above high watermark
__SINIC_VAL32(Intr_RxEmpty,   2, 1) // rx fifo empty
__SINIC_VAL32(Intr_RxDMA,     1, 1) // rx dma completed w/ interrupt
__SINIC_VAL32(Intr_RxPacket,  0, 1) // packet received
__SINIC_REG32(Intr_All,       0x01ff) // all valid interrupts
__SINIC_REG32(Intr_NoDelay,   0x01cc) // interrupts that aren't coalesced
__SINIC_REG32(Intr_Res,      ~0x01ff) // reserved interrupt bits

// RX Data Description
__SINIC_VAL64(RxData_NoDelay,  61,  1) // Don't Delay this copy
__SINIC_VAL64(RxData_Vaddr,    60,  1) // Addr is virtual
__SINIC_VAL64(RxData_Len,      40, 20) // 0 - 256k
__SINIC_VAL64(RxData_Addr,      0, 40) // Address 1TB

// TX Data Description
__SINIC_VAL64(TxData_More,     63,  1) // Packet not complete (will dma more)
__SINIC_VAL64(TxData_Checksum, 62,  1) // do checksum
__SINIC_VAL64(TxData_Vaddr,    60,  1) // Addr is virtual
__SINIC_VAL64(TxData_Len,      40, 20) // 0 - 256k
__SINIC_VAL64(TxData_Addr,      0, 40) // Address 1TB

// RX Done/Busy Information
__SINIC_VAL64(RxDone_Packets,   32, 16) // number of packets in rx fifo
__SINIC_VAL64(RxDone_Busy,      31,  1) // receive dma busy copying
__SINIC_VAL64(RxDone_Complete,  30,  1) // valid data (packet complete)
__SINIC_VAL64(RxDone_More,      29,  1) // Packet has more data (dma again)
__SINIC_VAL64(RxDone_Empty,     28,  1) // rx fifo is empty
__SINIC_VAL64(RxDone_High,      27,  1) // rx fifo is above the watermark
__SINIC_VAL64(RxDone_NotHigh,   26,  1) // rxfifo never hit the high watermark
__SINIC_VAL64(RxDone_TcpError,  25,  1) // TCP packet error (bad checksum)
__SINIC_VAL64(RxDone_UdpError,  24,  1) // UDP packet error (bad checksum)
__SINIC_VAL64(RxDone_IpError,   23,  1) // IP packet error (bad checksum)
__SINIC_VAL64(RxDone_TcpPacket, 22,  1) // this is a TCP packet
__SINIC_VAL64(RxDone_UdpPacket, 21,  1) // this is a UDP packet
__SINIC_VAL64(RxDone_IpPacket,  20,  1) // this is an IP packet
__SINIC_VAL64(RxDone_CopyLen,    0, 20) // up to 256k

// TX Done/Busy Information
__SINIC_VAL64(TxDone_Packets,   32, 16) // number of packets in tx fifo
__SINIC_VAL64(TxDone_Busy,      31,  1) // transmit dma busy copying
__SINIC_VAL64(TxDone_Complete,  30,  1) // valid data (packet complete)
__SINIC_VAL64(TxDone_Full,      29,  1) // tx fifo is full
__SINIC_VAL64(TxDone_Low,       28,  1) // tx fifo is below the watermark
__SINIC_VAL64(TxDone_Res0,      27,  1) // reserved
__SINIC_VAL64(TxDone_Res1,      26,  1) // reserved
__SINIC_VAL64(TxDone_Res2,      25,  1) // reserved
__SINIC_VAL64(TxDone_Res3,      24,  1) // reserved
__SINIC_VAL64(TxDone_Res4,      23,  1) // reserved
__SINIC_VAL64(TxDone_Res5,      22,  1) // reserved
__SINIC_VAL64(TxDone_Res6,      21,  1) // reserved
__SINIC_VAL64(TxDone_Res7,      20,  1) // reserved
__SINIC_VAL64(TxDone_CopyLen,    0, 20) // up to 256k

__SINIC_VAL64(RxStatus_Dirty,   48, 16)
__SINIC_VAL64(RxStatus_Mapped,  32, 16)
__SINIC_VAL64(RxStatus_Busy,    16, 16)
__SINIC_VAL64(RxStatus_Head,     0, 16)

struct Info
{
    uint8_t size;
    bool read;
    bool write;
    const char *name;
};

} // namespace registers

/* clang-format off */
inline const registers::Info&
regInfo(Addr daddr)
{
    static registers::Info invalid = { 0, false, false, "invalid" };
    static registers::Info info [] = {
        { 4, true,  true,  "Config"       },
        { 4, false, true,  "Command"      },
        { 4, true,  true,  "IntrStatus"   },
        { 4, true,  true,  "IntrMask"     },
        { 4, true,  false, "RxMaxCopy"    },
        { 4, true,  false, "TxMaxCopy"    },
        { 4, true,  false, "ZeroCopySize" },
        { 4, true,  false, "ZeroCopyMark" },
        { 4, true,  false, "VirtualCount" },
        { 4, true,  false, "RxMaxIntr"    },
        { 4, true,  false, "RxFifoSize"   },
        { 4, true,  false, "TxFifoSize"   },
        { 4, true,  false, "RxFifoLow"    },
        { 4, true,  false, "TxFifoLow"    },
        { 4, true,  false, "RxFifoHigh"   },
        { 4, true,  false, "TxFifoHigh"   },
        { 8, true,  true,  "RxData"       },
        invalid,
        { 8, true,  false, "RxDone"       },
        invalid,
        { 8, true,  false, "RxWait"       },
        invalid,
        { 8, true,  true,  "TxData"       },
        invalid,
        { 8, true,  false, "TxDone"       },
        invalid,
        { 8, true,  false, "TxWait"       },
        invalid,
        { 8, true,  false, "HwAddr"       },
        invalid,
        { 8, true,  false, "RxStatus"     },
        invalid,
    };

    return info[daddr / 4];
}
/* clang-format on */

inline bool
regValid(Addr daddr)
{
    if (daddr > registers::Size)
        return false;

    if (regInfo(daddr).size == 0)
        return false;

    return true;
}

} // namespace sinic

} // namespace gem5

#endif // __DEV_NET_SINICREG_HH__
