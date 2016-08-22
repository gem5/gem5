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

#ifndef __MEM_PHYSICAL_HH__
#define __MEM_PHYSICAL_HH__

#include "base/addr_range_map.hh"
#include "mem/packet.hh"

/**
 * Forward declaration to avoid header dependencies.
 */
class AbstractMemory;

/**
 * A single entry for the backing store.
 */
class BackingStoreEntry
{
  public:

    /**
     * Create a backing store entry. Don't worry about managing the memory
     * pointers, because PhysicalMemory is responsible for that.
     */
    BackingStoreEntry(AddrRange range, uint8_t* pmem,
                      bool conf_table_reported, bool in_addr_map, bool kvm_map)
        : range(range), pmem(pmem), confTableReported(conf_table_reported),
          inAddrMap(in_addr_map), kvmMap(kvm_map)
        {}

    /**
     * The address range covered in the guest.
     */
     AddrRange range;

    /**
     * Pointer to the host memory this range maps to. This memory is the same
     * size as the range field.
     */
     uint8_t* pmem;

     /**
      * Whether this memory should be reported to the configuration table
      */
     bool confTableReported;

     /**
      * Whether this memory should appear in the global address map
      */
     bool inAddrMap;

     /**
      * Whether KVM should map this memory into the guest address space during
      * acceleration.
      */
     bool kvmMap;
};

/**
 * The physical memory encapsulates all memories in the system and
 * provides basic functionality for accessing those memories without
 * going through the memory system and interconnect.
 *
 * The physical memory is also responsible for providing the host
 * system backingstore used by the memories in the simulated guest
 * system. When the system is created, the physical memory allocates
 * the backing store based on the address ranges that are populated in
 * the system, and does so independent of how those map to actual
 * memory controllers. Thus, the physical memory completely abstracts
 * the mapping of the backing store of the host system and the address
 * mapping in the guest system. This enables us to arbitrarily change
 * the number of memory controllers, and their address mapping, as
 * long as the ranges stay the same.
 */
class PhysicalMemory : public Serializable
{

  private:

    // Name for debugging
    std::string _name;

    // Global address map
    AddrRangeMap<AbstractMemory*> addrMap;

    // a mutable cache for the last address map iterator that matched
    // an address
    mutable AddrRangeMap<AbstractMemory*>::const_iterator rangeCache;

    // All address-mapped memories
    std::vector<AbstractMemory*> memories;

    // The total memory size
    uint64_t size;

    // Let the user choose if we reserve swap space when calling mmap
    const bool mmapUsingNoReserve;

    // The physical memory used to provide the memory in the simulated
    // system
    std::vector<BackingStoreEntry> backingStore;

    // Prevent copying
    PhysicalMemory(const PhysicalMemory&);

    // Prevent assignment
    PhysicalMemory& operator=(const PhysicalMemory&);

    /**
     * Create the memory region providing the backing store for a
     * given address range that corresponds to a set of memories in
     * the simulated system.
     *
     * @param range The address range covered
     * @param memories The memories this range maps to
     * @param kvm_map Should KVM map this memory for the guest
     */
    void createBackingStore(AddrRange range,
                            const std::vector<AbstractMemory*>& _memories,
                            bool conf_table_reported,
                            bool in_addr_map, bool kvm_map);

  public:

    /**
     * Create a physical memory object, wrapping a number of memories.
     */
    PhysicalMemory(const std::string& _name,
                   const std::vector<AbstractMemory*>& _memories,
                   bool mmap_using_noreserve);

    /**
     * Unmap all the backing store we have used.
     */
    ~PhysicalMemory();

    /**
     * Return the name for debugging and for creation of sections for
     * checkpointing.
     */
    const std::string name() const { return _name; }

    /**
     * Check if a physical address is within a range of a memory that
     * is part of the global address map.
     *
     * @param addr A physical address
     * @return Whether the address corresponds to a memory
     */
    bool isMemAddr(Addr addr) const;

    /**
     * Get the memory ranges for all memories that are to be reported
     * to the configuration table. The ranges are merged before they
     * are returned such that any interleaved ranges appear as a
     * single range.
     *
     * @return All configuration table memory ranges
     */
    AddrRangeList getConfAddrRanges() const;

    /**
     * Get the total physical memory size.
     *
     * @return The sum of all memory sizes
     */
    uint64_t totalSize() const { return size; }

     /**
     * Get the pointers to the backing store for external host
     * access. Note that memory in the guest should be accessed using
     * access() or functionalAccess(). This interface is primarily
     * intended for CPU models using hardware virtualization. Note
     * that memories that are null are not present, and that the
     * backing store may also contain memories that are not part of
     * the OS-visible global address map and thus are allowed to
     * overlap.
     *
     * @return Pointers to the memory backing store
     */
    std::vector<BackingStoreEntry> getBackingStore() const
    { return backingStore; }

    /**
     * Perform an untimed memory access and update all the state
     * (e.g. locked addresses) and statistics accordingly. The packet
     * is turned into a response if required.
     *
     * @param pkt Packet performing the access
     */
    void access(PacketPtr pkt);

    /**
     * Perform an untimed memory read or write without changing
     * anything but the memory itself. No stats are affected by this
     * access. In addition to normal accesses this also facilitates
     * print requests.
     *
     * @param pkt Packet performing the access
     */
    void functionalAccess(PacketPtr pkt);

    /**
     * Serialize all the memories in the system. This is independent
     * of the logical memory layout, and the serialization only sees
     * the contigous backing store, independent of how this maps to
     * logical memories in the guest system.
     *
     * @param os stream to serialize to
     */
    void serialize(CheckpointOut &cp) const override;

    /**
     * Serialize a specific store.
     *
     * @param store_id Unique identifier of this backing store
     * @param range The address range of this backing store
     * @param pmem The host pointer to this backing store
     */
    void serializeStore(CheckpointOut &cp, unsigned int store_id,
                        AddrRange range, uint8_t* pmem) const;

    /**
     * Unserialize the memories in the system. As with the
     * serialization, this action is independent of how the address
     * ranges are mapped to logical memories in the guest system.
     */
    void unserialize(CheckpointIn &cp) override;

    /**
     * Unserialize a specific backing store, identified by a section.
     */
    void unserializeStore(CheckpointIn &cp);

};

#endif //__MEM_PHYSICAL_HH__
