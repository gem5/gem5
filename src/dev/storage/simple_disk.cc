/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

/* @file
 * Simple disk interface for the system console
 */

#include "dev/storage/simple_disk.hh"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/SimpleDisk.hh"
#include "debug/SimpleDiskData.hh"
#include "dev/storage/disk_image.hh"
#include "mem/port_proxy.hh"
#include "sim/system.hh"

using namespace std;

SimpleDisk::SimpleDisk(const Params *p)
    : SimObject(p), system(p->system), image(p->disk)
{}

SimpleDisk::~SimpleDisk()
{}


void
SimpleDisk::read(Addr addr, baddr_t block, int count) const
{
    uint8_t *data = new uint8_t[SectorSize * count];

    if (count & (SectorSize - 1))
        panic("Not reading a multiple of a sector (count = %d)", count);

    for (int i = 0, j = 0; i < count; i += SectorSize, j++)
        image->read(data + i, block + j);

    system->physProxy.writeBlob(addr, data, count);

    DPRINTF(SimpleDisk, "read  block=%#x len=%d\n", (uint64_t)block, count);
    DDUMP(SimpleDiskData, data, count);

    delete [] data;
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

SimpleDisk *
SimpleDiskParams::create()
{
    return new SimpleDisk(this);
}
