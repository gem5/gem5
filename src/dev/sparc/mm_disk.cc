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

#include "base/trace.hh"
#include "dev/sparc/mm_disk.hh"
#include "dev/platform.hh"
#include "mem/port.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

MmDisk::MmDisk(Params *p)
    : BasicPioDevice(p), image(p->image), curSector((uint64_t)-1), dirty(false)
{
    memset(&bytes, 0, SectorSize);
    pioSize = image->size() * SectorSize;
}

Tick
MmDisk::read(PacketPtr pkt)
{
    Addr accessAddr;
    off_t sector;
    off_t bytes_read;
    uint16_t *d16;
    uint32_t *d32;
    uint64_t *d64;

    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    accessAddr = pkt->getAddr() - pioAddr;

    sector = accessAddr / SectorSize;

    if (sector != curSector) {
        if (dirty)
            bytes_read = image->write(bytes, curSector);
        bytes_read = image->read(bytes,  sector);
        curSector = sector;
    }
    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->set(bytes[accessAddr % SectorSize]);
        break;
      case sizeof(uint16_t):
        d16 = (uint16_t*)bytes + (accessAddr % SectorSize)/2;
        pkt->set(htobe(*d16));
        break;
      case sizeof(uint32_t):
        d32 = (uint32_t*)bytes + (accessAddr % SectorSize)/4;
        pkt->set(htobe(*d32));
        break;
      case sizeof(uint64_t):
        d64 = (uint64_t*)bytes + (accessAddr % SectorSize)/8;
        pkt->set(htobe(*d64));
        break;
      default:
        panic("Invalid access size\n");
    }

    pkt->result = Packet::Success;
    return pioDelay;
}

Tick
MmDisk::write(PacketPtr pkt)
{
   panic("need to implement\n");
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(MmDisk)
    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    Param<Addr> pio_size;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;
    SimObjectParam<DiskImage *> image;
END_DECLARE_SIM_OBJECT_PARAMS(MmDisk)

BEGIN_INIT_SIM_OBJECT_PARAMS(MmDisk)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(pio_size, "Size of address range"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(image, "disk image")

END_INIT_SIM_OBJECT_PARAMS(MmDisk)

CREATE_SIM_OBJECT(MmDisk)
{
    MmDisk::Params *p = new MmDisk::Params;
    p->name = getInstanceName();
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    p->image = image;
    return new MmDisk(p);
}

REGISTER_SIM_OBJECT("MmDisk", MmDisk)
