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

/** @file
 * This device acts as a disk similar to the memory mapped disk device
 * in legion. Any access is translated to an offset in the disk image.
 */

#include "dev/sparc/mm_disk.hh"

#include <cstring>

#include "base/trace.hh"
#include "debug/IdeDisk.hh"
#include "dev/platform.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

MmDisk::MmDisk(const Params *p)
    : BasicPioDevice(p, p->image->size() * SectorSize),
      image(p->image), curSector((off_t)-1), dirty(false)
{
    std::memset(&diskData, 0, SectorSize);
}

Tick
MmDisk::read(PacketPtr pkt)
{
    Addr accessAddr;
    off_t sector;
    uint16_t d16;
    uint32_t d32;
    uint64_t d64;

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    accessAddr = pkt->getAddr() - pioAddr;

    sector = accessAddr / SectorSize;

    if (sector != curSector) {
        if (dirty) {
#ifndef NDEBUG
            off_t bytes_written =
#endif
                image->write(diskData, curSector);
            assert(bytes_written == SectorSize);
        }
#ifndef NDEBUG
        off_t bytes_read =
#endif
            image->read(diskData, sector);
        assert(bytes_read == SectorSize);
        curSector = sector;
    }
    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->set(diskData[accessAddr % SectorSize]);
        DPRINTF(IdeDisk, "reading byte %#x value= %#x\n", accessAddr, diskData[accessAddr %
                SectorSize]);
        break;
      case sizeof(uint16_t):
        memcpy(&d16, diskData + (accessAddr % SectorSize), 2);
        pkt->set(htobe(d16));
        DPRINTF(IdeDisk, "reading word %#x value= %#x\n", accessAddr, d16);
        break;
      case sizeof(uint32_t):
        memcpy(&d32, diskData + (accessAddr % SectorSize), 4);
        pkt->set(htobe(d32));
        DPRINTF(IdeDisk, "reading dword %#x value= %#x\n", accessAddr, d32);
        break;
      case sizeof(uint64_t):
        memcpy(&d64, diskData + (accessAddr % SectorSize), 8);
        pkt->set(htobe(d64));
        DPRINTF(IdeDisk, "reading qword %#x value= %#x\n", accessAddr, d64);
        break;
      default:
        panic("Invalid access size\n");
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
MmDisk::write(PacketPtr pkt)
{
    Addr accessAddr;
    off_t sector;
    uint16_t d16;
    uint32_t d32;
    uint64_t d64;

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    accessAddr = pkt->getAddr() - pioAddr;

    sector = accessAddr / SectorSize;

    if (sector != curSector) {
        if (dirty) {
#ifndef NDEBUG
            off_t bytes_written =
#endif
                image->write(diskData, curSector);
            assert(bytes_written == SectorSize);
        }
#ifndef NDEBUG
        off_t bytes_read =
#endif
            image->read(diskData,  sector);
        assert(bytes_read == SectorSize);
        curSector = sector;
    }
    dirty = true;

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        diskData[accessAddr % SectorSize] = htobe(pkt->get<uint8_t>());
        DPRINTF(IdeDisk, "writing byte %#x value= %#x\n", accessAddr, diskData[accessAddr %
                SectorSize]);
        break;
      case sizeof(uint16_t):
        d16 = htobe(pkt->get<uint16_t>());
        memcpy(diskData + (accessAddr % SectorSize), &d16, 2);
        DPRINTF(IdeDisk, "writing word %#x value= %#x\n", accessAddr, d16);
        break;
      case sizeof(uint32_t):
        d32 = htobe(pkt->get<uint32_t>());
        memcpy(diskData + (accessAddr % SectorSize), &d32, 4);
        DPRINTF(IdeDisk, "writing dword %#x value= %#x\n", accessAddr, d32);
        break;
      case sizeof(uint64_t):
        d64 = htobe(pkt->get<uint64_t>());
        memcpy(diskData + (accessAddr % SectorSize), &d64, 8);
        DPRINTF(IdeDisk, "writing qword %#x value= %#x\n", accessAddr, d64);
        break;
      default:
        panic("Invalid access size\n");
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
MmDisk::serialize(CheckpointOut &cp) const
{
    // just write any dirty changes to the cow layer it will take care of
    // serialization
    if (dirty) {
#ifndef NDEBUG
        int bytes_read =
#endif
            image->write(diskData, curSector);
        assert(bytes_read == SectorSize);
    }
    ClockedObject::serialize(cp);
}

MmDisk *
MmDiskParams::create()
{
    return new MmDisk(this);
}
