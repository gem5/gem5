/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Hansson
 */

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/user.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <cerrno>
#include <climits>
#include <cstdio>
#include <iostream>
#include <string>

#include "base/trace.hh"
#include "debug/BusAddrRanges.hh"
#include "debug/Checkpoint.hh"
#include "mem/abstract_mem.hh"
#include "mem/physical.hh"

using namespace std;

PhysicalMemory::PhysicalMemory(const string& _name,
                               const vector<AbstractMemory*>& _memories) :
    _name(_name), size(0)
{
    // add the memories from the system to the address map as
    // appropriate
    for (vector<AbstractMemory*>::const_iterator m = _memories.begin();
         m != _memories.end(); ++m) {
        // only add the memory if it is part of the global address map
        if ((*m)->isInAddrMap()) {
            memories.push_back(*m);

            // calculate the total size once and for all
            size += (*m)->size();

            // add the range to our interval tree and make sure it does not
            // intersect an existing range
            if (addrMap.insert((*m)->getAddrRange(), *m) == addrMap.end())
                fatal("Memory address range for %s is overlapping\n",
                      (*m)->name());
        } else {
            DPRINTF(BusAddrRanges,
                    "Skipping memory %s that is not in global address map\n",
                    (*m)->name());
            // this type of memory is used e.g. as reference memory by
            // Ruby, and they also needs a backing store, but should
            // not be part of the global address map

            // simply do it independently, also note that this kind of
            // memories are allowed to overlap in the logic address
            // map
            vector<AbstractMemory*> unmapped_mems;
            unmapped_mems.push_back(*m);
            createBackingStore((*m)->getAddrRange(), unmapped_mems);
        }
    }

    // iterate over the increasing addresses and chunks of contiguous
    // space to be mapped to backing store, create it and inform the
    // memories
    vector<AddrRange> intlv_ranges;
    vector<AbstractMemory*> curr_memories;
    for (AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.begin();
         r != addrMap.end(); ++r) {
        // simply skip past all memories that are null and hence do
        // not need any backing store
        if (!r->second->isNull()) {
            // if the range is interleaved then save it for now
            if (r->first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r->first)) {
                    AddrRange merged_range(intlv_ranges);
                    createBackingStore(merged_range, curr_memories);
                    intlv_ranges.clear();
                    curr_memories.clear();
                }
                intlv_ranges.push_back(r->first);
                curr_memories.push_back(r->second);
            } else {
                vector<AbstractMemory*> single_memory;
                single_memory.push_back(r->second);
                createBackingStore(r->first, single_memory);
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged, go
    // ahead and do it
    if (!intlv_ranges.empty()) {
        AddrRange merged_range(intlv_ranges);
        createBackingStore(merged_range, curr_memories);
    }
}

void
PhysicalMemory::createBackingStore(AddrRange range,
                                   const vector<AbstractMemory*>& _memories)
{
    if (range.interleaved())
        panic("Cannot create backing store for interleaved range %s\n",
              range.to_string());

    // perform the actual mmap
    DPRINTF(BusAddrRanges, "Creating backing store for range %s with size %d\n",
            range.to_string(), range.size());
    int map_flags = MAP_ANON | MAP_PRIVATE;
    uint8_t* pmem = (uint8_t*) mmap(NULL, range.size(),
                                    PROT_READ | PROT_WRITE,
                                    map_flags, -1, 0);

    if (pmem == (uint8_t*) MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap %d bytes for range %s!\n", range.size(),
              range.to_string());
    }

    // remember this backing store so we can checkpoint it and unmap
    // it appropriately
    backingStore.push_back(make_pair(range, pmem));

    // point the memories to their backing store
    for (vector<AbstractMemory*>::const_iterator m = _memories.begin();
         m != _memories.end(); ++m) {
        DPRINTF(BusAddrRanges, "Mapping memory %s to backing store\n",
                (*m)->name());
        (*m)->setBackingStore(pmem);
    }
}

PhysicalMemory::~PhysicalMemory()
{
    // unmap the backing store
    for (vector<pair<AddrRange, uint8_t*> >::iterator s = backingStore.begin();
         s != backingStore.end(); ++s)
        munmap((char*)s->second, s->first.size());
}

bool
PhysicalMemory::isMemAddr(Addr addr) const
{
    // see if the address is within the last matched range
    if (!rangeCache.contains(addr)) {
        // lookup in the interval tree
        AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.find(addr);
        if (r == addrMap.end()) {
            // not in the cache, and not in the tree
            return false;
        }
        // the range is in the tree, update the cache
        rangeCache = r->first;
    }

    assert(addrMap.find(addr) != addrMap.end());

    // either matched the cache or found in the tree
    return true;
}

AddrRangeList
PhysicalMemory::getConfAddrRanges() const
{
    // this could be done once in the constructor, but since it is unlikely to
    // be called more than once the iteration should not be a problem
    AddrRangeList ranges;
    vector<AddrRange> intlv_ranges;
    for (AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.begin();
         r != addrMap.end(); ++r) {
        if (r->second->isConfReported()) {
            // if the range is interleaved then save it for now
            if (r->first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r->first)) {
                    ranges.push_back(AddrRange(intlv_ranges));
                    intlv_ranges.clear();
                }
                intlv_ranges.push_back(r->first);
            } else {
                // keep the current range
                ranges.push_back(r->first);
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged,
    // go ahead and do it
    if (!intlv_ranges.empty()) {
        ranges.push_back(AddrRange(intlv_ranges));
    }

    return ranges;
}

void
PhysicalMemory::access(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    AddrRangeMap<AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->access(pkt);
}

void
PhysicalMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    AddrRangeMap<AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->functionalAccess(pkt);
}

void
PhysicalMemory::serialize(ostream& os)
{
    // serialize all the locked addresses and their context ids
    vector<Addr> lal_addr;
    vector<int> lal_cid;

    for (vector<AbstractMemory*>::iterator m = memories.begin();
         m != memories.end(); ++m) {
        const list<LockedAddr>& locked_addrs = (*m)->getLockedAddrList();
        for (list<LockedAddr>::const_iterator l = locked_addrs.begin();
             l != locked_addrs.end(); ++l) {
            lal_addr.push_back(l->addr);
            lal_cid.push_back(l->contextId);
        }
    }

    arrayParamOut(os, "lal_addr", lal_addr);
    arrayParamOut(os, "lal_cid", lal_cid);

    // serialize the backing stores
    unsigned int nbr_of_stores = backingStore.size();
    SERIALIZE_SCALAR(nbr_of_stores);

    unsigned int store_id = 0;
    // store each backing store memory segment in a file
    for (vector<pair<AddrRange, uint8_t*> >::iterator s = backingStore.begin();
         s != backingStore.end(); ++s) {
        nameOut(os, csprintf("%s.store%d", name(), store_id));
        serializeStore(os, store_id++, s->first, s->second);
    }
}

void
PhysicalMemory::serializeStore(ostream& os, unsigned int store_id,
                               AddrRange range, uint8_t* pmem)
{
    // we cannot use the address range for the name as the
    // memories that are not part of the address map can overlap
    string filename = name() + ".store" + to_string(store_id) + ".pmem";
    long range_size = range.size();

    DPRINTF(Checkpoint, "Serializing physical memory %s with size %d\n",
            filename, range_size);

    SERIALIZE_SCALAR(store_id);
    SERIALIZE_SCALAR(filename);
    SERIALIZE_SCALAR(range_size);

    // write memory file
    string filepath = Checkpoint::dir() + "/" + filename.c_str();
    int fd = creat(filepath.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open physical memory checkpoint file '%s'\n",
              filename);
    }

    gzFile compressed_mem = gzdopen(fd, "wb");
    if (compressed_mem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);

    uint64_t pass_size = 0;

    // gzwrite fails if (int)len < 0 (gzwrite returns int)
    for (uint64_t written = 0; written < range.size();
         written += pass_size) {
        pass_size = (uint64_t)INT_MAX < (range.size() - written) ?
            (uint64_t)INT_MAX : (range.size() - written);

        if (gzwrite(compressed_mem, pmem + written,
                    (unsigned int) pass_size) != (int) pass_size) {
            fatal("Write failed on physical memory checkpoint file '%s'\n",
                  filename);
        }
    }

    // close the compressed stream and check that the exit status
    // is zero
    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

}

void
PhysicalMemory::unserialize(Checkpoint* cp, const string& section)
{
    // unserialize the locked addresses and map them to the
    // appropriate memory controller
    vector<Addr> lal_addr;
    vector<int> lal_cid;
    arrayParamIn(cp, section, "lal_addr", lal_addr);
    arrayParamIn(cp, section, "lal_cid", lal_cid);
    for(size_t i = 0; i < lal_addr.size(); ++i) {
        AddrRangeMap<AbstractMemory*>::const_iterator m =
            addrMap.find(lal_addr[i]);
        m->second->addLockedAddr(LockedAddr(lal_addr[i], lal_cid[i]));
    }

    // unserialize the backing stores
    unsigned int nbr_of_stores;
    UNSERIALIZE_SCALAR(nbr_of_stores);

    for (unsigned int i = 0; i < nbr_of_stores; ++i) {
        unserializeStore(cp, csprintf("%s.store%d", section, i));
    }

}

void
PhysicalMemory::unserializeStore(Checkpoint* cp, const string& section)
{
    const uint32_t chunk_size = 16384;

    unsigned int store_id;
    UNSERIALIZE_SCALAR(store_id);

    string filename;
    UNSERIALIZE_SCALAR(filename);
    string filepath = cp->cptDir + "/" + filename;

    // mmap memoryfile
    int fd = open(filepath.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    gzFile compressed_mem = gzdopen(fd, "rb");
    if (compressed_mem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);

    // we've already got the actual backing store mapped
    uint8_t* pmem = backingStore[store_id].second;
    AddrRange range = backingStore[store_id].first;

    long range_size;
    UNSERIALIZE_SCALAR(range_size);

    DPRINTF(Checkpoint, "Unserializing physical memory %s with size %d\n",
            filename, range_size);

    if (range_size != range.size())
        fatal("Memory range size has changed! Saw %lld, expected %lld\n",
              range_size, range.size());

    uint64_t curr_size = 0;
    long* temp_page = new long[chunk_size];
    long* pmem_current;
    uint32_t bytes_read;
    while (curr_size < range.size()) {
        bytes_read = gzread(compressed_mem, temp_page, chunk_size);
        if (bytes_read == 0)
            break;

        assert(bytes_read % sizeof(long) == 0);

        for (uint32_t x = 0; x < bytes_read / sizeof(long); x++) {
            // Only copy bytes that are non-zero, so we don't give
            // the VM system hell
            if (*(temp_page + x) != 0) {
                pmem_current = (long*)(pmem + curr_size + x * sizeof(long));
                *pmem_current = *(temp_page + x);
            }
        }
        curr_size += bytes_read;
    }

    delete[] temp_page;

    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);
}
