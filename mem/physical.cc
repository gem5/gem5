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
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <cstdio>
#include <iostream>
#include <string>


#include "base/misc.hh"
#include "config/full_system.hh"
#if FULL_SYSTEM
#include "mem/functional/memory_control.hh"
#endif
#include "mem/functional/physical.hh"
#include "sim/host.hh"
#include "sim/builder.hh"
#include "targetarch/isa_traits.hh"

using namespace std;

#if FULL_SYSTEM
PhysicalMemory::PhysicalMemory(const string &n, Range<Addr> range,
                               MemoryController *mmu, const std::string &fname)
    : FunctionalMemory(n), base_addr(range.start), pmem_size(range.size()),
      pmem_addr(NULL)
{
    if (pmem_size % TheISA::PageBytes != 0)
        panic("Memory Size not divisible by page size\n");

    mmu->add_child(this, range);

    int fd = -1;

    if (!fname.empty()) {
        fd = open(fname.c_str(), O_RDWR | O_CREAT, 0644);
        if (fd == -1) {
            perror("open");
            fatal("Could not open physical memory file: %s\n", fname);
        }
        ftruncate(fd, pmem_size);
    }

    int map_flags = (fd == -1) ? (MAP_ANON | MAP_PRIVATE) : MAP_SHARED;
    pmem_addr = (uint8_t *)mmap(NULL, pmem_size, PROT_READ | PROT_WRITE,
                                map_flags, fd, 0);

    if (fd != -1)
        close(fd);

    if (pmem_addr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap!\n");
    }

    page_ptr = 0;
}
#endif

PhysicalMemory::PhysicalMemory(const string &n)
    : FunctionalMemory(n), base_addr(0), pmem_addr(NULL)
{
    // Hardcoded to 128 MB for now.
    pmem_size = 1 << 27;

    if (pmem_size % TheISA::PageBytes != 0)
        panic("Memory Size not divisible by page size\n");

    int map_flags = MAP_ANON | MAP_PRIVATE;
    pmem_addr = (uint8_t *)mmap(NULL, pmem_size, PROT_READ | PROT_WRITE,
                                map_flags, -1, 0);

    if (pmem_addr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap!\n");
    }

    page_ptr = 0;
}

PhysicalMemory::~PhysicalMemory()
{
    if (pmem_addr)
        munmap(pmem_addr, pmem_size);
}

Addr
PhysicalMemory::new_page()
{
    Addr return_addr = page_ptr << LogVMPageSize;
    return_addr += base_addr;

    ++page_ptr;
    return return_addr;
}

//
// little helper for better prot_* error messages
//
void
PhysicalMemory::prot_access_error(Addr addr, int size, const string &func)
{
    panic("invalid physical memory access!\n"
          "%s: %s(addr=%#x, size=%d) out of range (max=%#x)\n",
          name(), func, addr, size, pmem_size - 1);
}

void
PhysicalMemory::prot_read(Addr addr, uint8_t *p, int size)
{
    if (addr + size >= pmem_size)
        prot_access_error(addr, size, "prot_read");

    memcpy(p, pmem_addr + addr - base_addr, size);
}

void
PhysicalMemory::prot_write(Addr addr, const uint8_t *p, int size)
{
    if (addr + size >= pmem_size)
        prot_access_error(addr, size, "prot_write");

    memcpy(pmem_addr + addr - base_addr, p, size);
}

void
PhysicalMemory::prot_memset(Addr addr, uint8_t val, int size)
{
    if (addr + size >= pmem_size)
        prot_access_error(addr, size, "prot_memset");

    memset(pmem_addr + addr - base_addr, val, size);
}

void
PhysicalMemory::serialize(ostream &os)
{
    gzFile compressedMem;
    string filename = name() + ".physmem";

    SERIALIZE_SCALAR(pmem_size);
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

    if (gzwrite(compressedMem, pmem_addr, pmem_size) != pmem_size) {
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


    // unmap file that was mmaped in the constructor
    munmap(pmem_addr, pmem_size);

    string filename;

    UNSERIALIZE_SCALAR(pmem_size);
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


    pmem_addr = (uint8_t *)mmap(NULL, pmem_size, PROT_READ | PROT_WRITE,
                                MAP_ANON | MAP_PRIVATE, -1, 0);

    if (pmem_addr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap physical memory!\n");
    }

    curSize = 0;
    tempPage = (long*)malloc(chunkSize);
    if (tempPage == NULL)
        fatal("Unable to malloc memory to read file %s\n", filename);

    /* Only copy bytes that are non-zero, so we don't give the VM system hell */
    while (curSize < pmem_size) {
        bytesRead = gzread(compressedMem, tempPage, chunkSize);
        if (bytesRead != chunkSize && bytesRead != pmem_size - curSize)
            fatal("Read failed on physical memory checkpoint file '%s'"
                  " got %d bytes, expected %d or %d bytes\n",
                  filename, bytesRead, chunkSize, pmem_size-curSize);

        assert(bytesRead % sizeof(long) == 0);

        for (int x = 0; x < bytesRead/sizeof(long); x++)
        {
             if (*(tempPage+x) != 0) {
                 pmem_current = (long*)(pmem_addr + curSize + x * sizeof(long));
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
#if FULL_SYSTEM
    SimObjectParam<MemoryController *> mmu;
#endif
    Param<Range<Addr> > range;

END_DECLARE_SIM_OBJECT_PARAMS(PhysicalMemory)

BEGIN_INIT_SIM_OBJECT_PARAMS(PhysicalMemory)

    INIT_PARAM_DFLT(file, "memory mapped file", ""),
#if FULL_SYSTEM
    INIT_PARAM(mmu, "Memory Controller"),
#endif
    INIT_PARAM(range, "Device Address Range")

END_INIT_SIM_OBJECT_PARAMS(PhysicalMemory)

CREATE_SIM_OBJECT(PhysicalMemory)
{
#if FULL_SYSTEM
    if (mmu) {
        return new PhysicalMemory(getInstanceName(), range, mmu, file);
    }
#endif

    return new PhysicalMemory(getInstanceName());
}

REGISTER_SIM_OBJECT("PhysicalMemory", PhysicalMemory)
