/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_RUBYSLICC_COMPONENTMAPPINGS_HH__
#define __MEM_RUBY_SLICC_INTERFACE_RUBYSLICC_COMPONENTMAPPINGS_HH__

#include "mem/protocol/GenericMachineType.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/common/Set.hh"
#include "mem/ruby/system/DirectoryMemory.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/ruby/system/NodeID.hh"

#ifdef MACHINETYPE_L1Cache
#define MACHINETYPE_L1CACHE_ENUM MachineType_L1Cache
#else
#define MACHINETYPE_L1CACHE_ENUM MachineType_NUM
#endif

#ifdef MACHINETYPE_L2Cache
#define MACHINETYPE_L2CACHE_ENUM MachineType_L2Cache
#else
#define MACHINETYPE_L2CACHE_ENUM MachineType_NUM
#endif

#ifdef MACHINETYPE_L3Cache
#define MACHINETYPE_L3CACHE_ENUM MachineType_L3Cache
#else
#define MACHINETYPE_L3CACHE_ENUM MachineType_NUM
#endif

#ifdef MACHINETYPE_DMA
#define MACHINETYPE_DMA_ENUM MachineType_DMA
#else
#define MACHINETYPE_DMA_ENUM MachineType_NUM
#endif

// used to determine the home directory
// returns a value between 0 and total_directories_within_the_system
inline NodeID
map_Address_to_DirectoryNode(const Address& addr)
{
    return DirectoryMemory::mapAddressToDirectoryVersion(addr);
}

// used to determine the home directory
// returns a value between 0 and total_directories_within_the_system
inline MachineID
map_Address_to_Directory(const Address &addr)
{
    MachineID mach =
        {MachineType_Directory, map_Address_to_DirectoryNode(addr)};
    return mach;
}

inline MachineID
map_Address_to_DMA(const Address & addr)
{
    MachineID dma = {MACHINETYPE_DMA_ENUM, 0};
    return dma;
}

inline NetDest
broadcast(MachineType type)
{
    NetDest dest;
    for (int i = 0; i < MachineType_base_count(type); i++) {
        MachineID mach = {type, i};
        dest.add(mach);
    }
    return dest;
}

inline MachineID
mapAddressToRange(const Address & addr, MachineType type, int low_bit,
                  int num_bits)
{
    MachineID mach = {type, 0};
    if (num_bits == 0)
        return mach;
    mach.num = addr.bitSelect(low_bit, low_bit + num_bits - 1);
    return mach;
}

inline NodeID
machineIDToNodeID(MachineID machID)
{
    return machID.num;
}

inline MachineType
machineIDToMachineType(MachineID machID)
{
    return machID.type;
}

inline NodeID
L1CacheMachIDToProcessorNum(MachineID machID)
{
    assert(machID.type == MACHINETYPE_L1CACHE_ENUM);
    return machID.num;
}

inline MachineID
getL1MachineID(NodeID L1RubyNode)
{
    MachineID mach = {MACHINETYPE_L1CACHE_ENUM, L1RubyNode};
    return mach;
}

inline GenericMachineType
ConvertMachToGenericMach(MachineType machType)
{
    if (machType == MACHINETYPE_L1CACHE_ENUM)
        return GenericMachineType_L1Cache;

    if (machType == MACHINETYPE_L2CACHE_ENUM)
        return GenericMachineType_L2Cache;

    if (machType == MACHINETYPE_L3CACHE_ENUM)
        return GenericMachineType_L3Cache;

    if (machType == MachineType_Directory)
        return GenericMachineType_Directory;

    if (machType == MACHINETYPE_DMA_ENUM)
        return GenericMachineType_DMA;

    panic("cannot convert to a GenericMachineType");
}

inline int
machineCount(MachineType machType)
{
    return MachineType_base_count(machType);
}

#endif  // __MEM_RUBY_SLICC_INTERFACE_COMPONENTMAPPINGS_HH__
