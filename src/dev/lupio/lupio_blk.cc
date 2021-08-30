/*
 * Copyright (c) 2021 The Regents of the University of California
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

#include "dev/lupio/lupio_blk.hh"

#include "debug/LupioBLK.hh"
#include "dev/dma_device.hh"
#include "mem/packet_access.hh"
#include "params/LupioBLK.hh"

/* Shared fields for CTRL and STAT registers */
#define LUPIO_BLK_TYPE  0x2

/* Specific fields for CTRL */
#define LUPIO_BLK_IRQE  0x1

/* Specific fields for STAT */
#define LUPIO_BLK_BUSY  0x1
#define LUPIO_BLK_ERRR  0x4

/* Fixed sector size of 512 bytes */
#define SECTOR_BITS   9
#define SECTOR_SIZE   (1ULL << SECTOR_BITS)

namespace gem5
{

LupioBLK::LupioBLK(const Params &params) :
    DmaDevice(params),
    platform(params.platform),
    dmaEvent([this]{ dmaEventDone(); }, name()),
    pioAddr(params.pio_addr),
    pioSize(params.pio_size),
    image(*params.image),
    lupioBLKIntID(params.int_id)
{
    static_assert(SECTOR_SIZE == SectorSize, "Sector size of disk image must"
                    " match LupIO device\n");
    nbBlocks = image.size();
    gem5_assert(isPowerOf2(nbBlocks));
    gem5_assert(ceilLog2(nbBlocks) <= 32, "Max number of blocks is 2^32\n");
    DPRINTF(LupioBLK, "LupioBLK initalized\n");
}

void
LupioBLK::dmaEventDone()
{
    int64_t _offset = lba;

    if (writeOp) {
        for (int i = 0; i < reqLen; i += SECTOR_SIZE, _offset++) {
            image.write(&reqData[i], _offset);
        }
    }

    delete[] reqData;
    busy = false;
    DPRINTF(LupioBLK, "Done with DMA event\n");
    platform->postPciInt(lupioBLKIntID);
}

uint64_t
LupioBLK::lupioBLKRead(const uint8_t addr)
{
    uint64_t r = 0;

    switch (addr >> 2) {
        case LUPIO_BLK_CONF:
            r = SECTOR_BITS << 16               // log2(512B / block)
                | floorLog2(nbBlocks);
            DPRINTF(LupioBLK, "Read LUPIO_BLK_CONF: %d\n", r);
            break;
        case LUPIO_BLK_NBLK:
            r = nblk;
            DPRINTF(LupioBLK, "Read LUPIO_BLK_NBLK: %d\n", r);
            break;
        case LUPIO_BLK_BLKA:
            r = lba;
            DPRINTF(LupioBLK, "Read LUPIO_BLK_BLKA: %d\n", r);
            break;
        case LUPIO_BLK_MEMA:
            r = mem;
            DPRINTF(LupioBLK, "Read LUPIO_BLK_MEMA: %d\n", r);
            break;

        case LUPIO_BLK_STAT:
            r = busy;
            if (writeOp) {
                r |= LUPIO_BLK_TYPE;    // Write command
            }
            if (err) {
                r |= LUPIO_BLK_ERRR;    // Error
            }
            DPRINTF(LupioBLK, "Read LUPIO_BLK_STAT: %d\n", r);

            // Acknowledge IRQ
            platform->clearPciInt(lupioBLKIntID);
            break;

        default:
            panic("Unexpected read to the LupioBLK device at address"
                   " %#llx!\n", addr);
            break;
    }
    return r;
}

void
LupioBLK::lupioBLKWrite(const uint8_t addr, uint64_t val64)
{
    uint32_t val = val64;

    switch (addr >> 2) {
        case LUPIO_BLK_NBLK:
            nblk = val;
            DPRINTF(LupioBLK, "Write LUPIO_BLK_NBLK: %d\n", nblk);
            break;
        case LUPIO_BLK_BLKA:
            lba = val;
            DPRINTF(LupioBLK, "Write LUPIO_BLK_BLKA: %d\n", lba);
            break;
        case LUPIO_BLK_MEMA:
            mem = val;
            DPRINTF(LupioBLK, "Write LUPIO_BLK_MEMA: %d\n", mem);
            break;

        case LUPIO_BLK_CTRL:
            // Perform command
            if (!busy) {
                err = false;
                writeOp = val & LUPIO_BLK_TYPE;
                lupioBLKCmd();
            } else {
                panic("Attempting to write to LupioBLK device while transfer"
                        " is ongoing!\n");
            }
            break;

        default:
            panic("Unexpected write to the LupioBLK device at address"
                    " %#llx!\n", addr);
            break;
    }
}

void
LupioBLK::lupioBLKCmd(void)
{
    // Check parameters
    if ((uint64_t)lba + nblk > nbBlocks) {
        panic("Bad LBA range\n");
    }

    // Block device is busy when a transfer occurs
    busy = true;

    // Perform transfer
    reqLen = nblk * SECTOR_SIZE;
    reqData = new uint8_t[reqLen];
    int64_t _offset = lba;

    if (!writeOp) {
        // Read command (block -> mem)
        for (int i = 0; i < reqLen; i += SECTOR_SIZE, _offset++) {
            image.read(&reqData[i], _offset);
        }
        dmaWrite(mem, reqLen, &dmaEvent, reqData, 0);
    } else {
        // Write command (mem -> block)
        dmaRead(mem, reqLen, &dmaEvent, reqData, 0);
    }
}

AddrRangeList
LupioBLK::getAddrRanges() const
{
    AddrRangeList ranges = { RangeSize(pioAddr, pioSize) };
    return ranges;
}

Tick
LupioBLK::read(PacketPtr pkt)
{
    Addr addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioBLK,
        "Read request - addr: %#x, size: %#x\n", addr, pkt->getSize());

    uint64_t read_request = lupioBLKRead(addr);
    DPRINTF(LupioBLK, "Packet Read: %d\n", read_request);
    pkt->setUintX(read_request, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioBLK::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioBLK, "Write register %#x value %#x\n", daddr,
            pkt->getUintX(byteOrder));

    lupioBLKWrite(daddr, pkt->getUintX(byteOrder));
    DPRINTF(LupioBLK, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5
