/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 * Simple disk interface for the system console
 */

#include <sys/types.h>
#include <sys/uio.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "base/misc.hh"
#include "base/trace.hh"
#include "dev/disk_image.hh"
#include "dev/simple_disk.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "sim/builder.hh"

using namespace std;

SimpleDisk::SimpleDisk(const string &name, PhysicalMemory *pmem,
                       DiskImage *img)
    : SimObject(name), physmem(pmem), image(img)
{}

SimpleDisk::~SimpleDisk()
{}


void
SimpleDisk::read(Addr addr, baddr_t block, int count) const
{
    uint8_t *data = physmem->dma_addr(addr, count);
    if (!data)
        panic("dma out of range! read addr=%#x count=%d\n", addr, count);

    if (count & (SectorSize - 1))
        panic("Not reading a multiple of a sector (count = %d)", count);

    for (int i = 0, j = 0; i < count; i += SectorSize, j++)
        image->read(data + i, block + j);

    DPRINTF(SimpleDisk, "read  block=%#x len=%d\n", (uint64_t)block, count);
    DDUMP(SimpleDiskData, data, count);
}

void
SimpleDisk::write(Addr addr, baddr_t block, int count)
{
    panic("unimplemented!\n");

#if 0
    uint8_t *data = physmem->dma_addr(addr, count);
    if (!data)
        panic("dma out of range! write addr=%#x count=%d\n", addr, count);

    image->write(data, block, count);
#endif
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(SimpleDisk)

    SimObjectParam<PhysicalMemory *> physmem;
    SimObjectParam<DiskImage *> disk;

END_DECLARE_SIM_OBJECT_PARAMS(SimpleDisk)

BEGIN_INIT_SIM_OBJECT_PARAMS(SimpleDisk)

    INIT_PARAM(physmem, "Physical Memory"),
    INIT_PARAM(disk, "Disk Image")

END_INIT_SIM_OBJECT_PARAMS(SimpleDisk)

CREATE_SIM_OBJECT(SimpleDisk)
{
    return new SimpleDisk(getInstanceName(), physmem, disk);
}

REGISTER_SIM_OBJECT("SimpleDisk", SimpleDisk)
