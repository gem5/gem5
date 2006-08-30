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
 * Authors: Ron Dreslinski
 *          Ali Saidi
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <iostream>
#include <string>


#include "base/misc.hh"
#include "config/full_system.hh"
#include "mem/packet_impl.hh"
#include "mem/physical.hh"
#include "sim/host.hh"
#include "sim/builder.hh"
#include "sim/eventq.hh"
#include "arch/isa_traits.hh"


using namespace std;
using namespace TheISA;


PhysicalMemory::PhysicalMemory(Params *p)
    : MemObject(p->name), pmemAddr(NULL), port(NULL), lat(p->latency), _params(p)
{
    if (params()->addrRange.size() % TheISA::PageBytes != 0)
        panic("Memory Size not divisible by page size\n");

    int map_flags = MAP_ANON | MAP_PRIVATE;
    pmemAddr = (uint8_t *)mmap(NULL, params()->addrRange.size(), PROT_READ | PROT_WRITE,
                                map_flags, -1, 0);

    if (pmemAddr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap!\n");
    }

    pagePtr = 0;
}

void
PhysicalMemory::init()
{
    if (!port)
        panic("PhysicalMemory not connected to anything!");
    port->sendStatusChange(Port::RangeChange);
}

PhysicalMemory::~PhysicalMemory()
{
    if (pmemAddr)
        munmap(pmemAddr, params()->addrRange.size());
    //Remove memPorts?
}

Addr
PhysicalMemory::new_page()
{
    Addr return_addr = pagePtr << LogVMPageSize;
    return_addr += params()->addrRange.start;

    ++pagePtr;
    return return_addr;
}

int
PhysicalMemory::deviceBlockSize()
{
    //Can accept anysize request
    return 0;
}

Tick
PhysicalMemory::calculateLatency(Packet *pkt)
{
    return lat;
}

void
PhysicalMemory::doFunctionalAccess(Packet *pkt)
{
    assert(pkt->getAddr() + pkt->getSize() < params()->addrRange.size());

    switch (pkt->cmd) {
      case Packet::ReadReq:
        memcpy(pkt->getPtr<uint8_t>(),
               pmemAddr + pkt->getAddr() - params()->addrRange.start,
               pkt->getSize());
        break;
      case Packet::WriteReq:
        memcpy(pmemAddr + pkt->getAddr() - params()->addrRange.start,
               pkt->getPtr<uint8_t>(),
               pkt->getSize());
        // temporary hack: will need to add real LL/SC implementation
        // for cacheless systems later.
        if (pkt->req->getFlags() & LOCKED) {
            pkt->req->setScResult(1);
        }
        break;
      default:
        panic("unimplemented");
    }

    pkt->result = Packet::Success;
}

Port *
PhysicalMemory::getPort(const std::string &if_name, int idx)
{
    if (if_name == "port" && idx == -1) {
        if (port != NULL)
           panic("PhysicalMemory::getPort: additional port requested to memory!");
        port = new MemoryPort(name() + "-port", this);
        return port;
    } else if (if_name == "functional") {
        /* special port for functional writes at startup. */
        return new MemoryPort(name() + "-funcport", this);
    } else {
        panic("PhysicalMemory::getPort: unknown port %s requested", if_name);
    }
}

void
PhysicalMemory::recvStatusChange(Port::Status status)
{
}

PhysicalMemory::MemoryPort::MemoryPort(const std::string &_name,
                                       PhysicalMemory *_memory)
    : SimpleTimingPort(_name), memory(_memory)
{ }

void
PhysicalMemory::MemoryPort::recvStatusChange(Port::Status status)
{
    memory->recvStatusChange(status);
}

void
PhysicalMemory::MemoryPort::getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
{
    memory->getAddressRanges(resp, snoop);
}

void
PhysicalMemory::getAddressRanges(AddrRangeList &resp, AddrRangeList &snoop)
{
    snoop.clear();
    resp.clear();
    resp.push_back(RangeSize(params()->addrRange.start,
                             params()->addrRange.size()));
}

int
PhysicalMemory::MemoryPort::deviceBlockSize()
{
    return memory->deviceBlockSize();
}

Tick
PhysicalMemory::MemoryPort::recvAtomic(Packet *pkt)
{
    memory->doFunctionalAccess(pkt);
    return memory->calculateLatency(pkt);
}

void
PhysicalMemory::MemoryPort::recvFunctional(Packet *pkt)
{
    // Default implementation of SimpleTimingPort::recvFunctional()
    // calls recvAtomic() and throws away the latency; we can save a
    // little here by just not calculating the latency.
    memory->doFunctionalAccess(pkt);
}

unsigned int
PhysicalMemory::drain(Event *de)
{
    int count = port->drain(de);
    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}

void
PhysicalMemory::serialize(ostream &os)
{
    gzFile compressedMem;
    string filename = name() + ".physmem";

    SERIALIZE_SCALAR(filename);

    // write memory file
    string thefile = Checkpoint::dir() + "/" + filename.c_str();
    int fd = creat(thefile.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open physical memory checkpoint file '%s'\n", filename);
    }

    compressedMem = gzdopen(fd, "wb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

    if (gzwrite(compressedMem, pmemAddr, params()->addrRange.size()) != params()->addrRange.size()) {
        fatal("Write failed on physical memory checkpoint file '%s'\n",
              filename);
    }

    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);
}

void
PhysicalMemory::unserialize(Checkpoint *cp, const string &section)
{
    gzFile compressedMem;
    long *tempPage;
    long *pmem_current;
    uint64_t curSize;
    uint32_t bytesRead;
    const int chunkSize = 16384;


    string filename;

    UNSERIALIZE_SCALAR(filename);

    filename = cp->cptDir + "/" + filename;

    // mmap memoryfile
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    compressedMem = gzdopen(fd, "rb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

    // unmap file that was mmaped in the constructor
    // This is done here to make sure that gzip and open don't muck with our
    // nice large space of memory before we reallocate it
    munmap(pmemAddr, params()->addrRange.size());

    pmemAddr = (uint8_t *)mmap(NULL, params()->addrRange.size(), PROT_READ | PROT_WRITE,
                                MAP_ANON | MAP_PRIVATE, -1, 0);

    if (pmemAddr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap physical memory!\n");
    }

    curSize = 0;
    tempPage = (long*)malloc(chunkSize);
    if (tempPage == NULL)
        fatal("Unable to malloc memory to read file %s\n", filename);

    /* Only copy bytes that are non-zero, so we don't give the VM system hell */
    while (curSize < params()->addrRange.size()) {
        bytesRead = gzread(compressedMem, tempPage, chunkSize);
        if (bytesRead != chunkSize && bytesRead != params()->addrRange.size() - curSize)
            fatal("Read failed on physical memory checkpoint file '%s'"
                  " got %d bytes, expected %d or %d bytes\n",
                  filename, bytesRead, chunkSize, params()->addrRange.size()-curSize);

        assert(bytesRead % sizeof(long) == 0);

        for (int x = 0; x < bytesRead/sizeof(long); x++)
        {
             if (*(tempPage+x) != 0) {
                 pmem_current = (long*)(pmemAddr + curSize + x * sizeof(long));
                 *pmem_current = *(tempPage+x);
             }
        }
        curSize += bytesRead;
    }

    free(tempPage);

    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(PhysicalMemory)

    Param<string> file;
    Param<Range<Addr> > range;
    Param<Tick> latency;

END_DECLARE_SIM_OBJECT_PARAMS(PhysicalMemory)

BEGIN_INIT_SIM_OBJECT_PARAMS(PhysicalMemory)

    INIT_PARAM_DFLT(file, "memory mapped file", ""),
    INIT_PARAM(range, "Device Address Range"),
    INIT_PARAM(latency, "Memory access latency")

END_INIT_SIM_OBJECT_PARAMS(PhysicalMemory)

CREATE_SIM_OBJECT(PhysicalMemory)
{
    PhysicalMemory::Params *p = new PhysicalMemory::Params;
    p->name = getInstanceName();
    p->addrRange = range;
    p->latency = latency;
    return new PhysicalMemory(p);
}

REGISTER_SIM_OBJECT("PhysicalMemory", PhysicalMemory)
