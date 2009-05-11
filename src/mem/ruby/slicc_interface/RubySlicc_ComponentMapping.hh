
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

/*
 * $Id$
 */

#ifndef COMPONENTMAPPINGFNS_H
#define COMPONENTMAPPINGFNS_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/MachineID.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Set.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/protocol/GenericMachineType.hh"

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

#ifdef MACHINETYPE_PersistentArbiter
#define MACHINETYPE_PERSISTENTARBITER_ENUM MachineType_PersistentArbiter
#else
#define MACHINETYPE_PERSISTENTARBITER_ENUM MachineType_NUM
#endif

#ifdef MACHINETYPE_Collector
#define MACHINETYPE_COLLECTOR_ENUM MachineType_Collector
#else
#define MACHINETYPE_COLLECTOR_ENUM MachineType_NUM
#endif


// used to determine the correct L1 set
// input parameters are the address and number of set bits for the L1 cache
// returns a value between 0 and the total number of L1 cache sets
inline
int map_address_to_L1CacheSet(const Address& addr, int cache_num_set_bits)
{
  return addr.bitSelect(RubyConfig::dataBlockBits(),
                        RubyConfig::dataBlockBits()+cache_num_set_bits-1);
}

// used to determine the correct L2 set
// input parameters are the address and number of set bits for the L2 cache
// returns a value between 0 and the total number of L2 cache sets
inline
int map_address_to_L2CacheSet(const Address& addr, int cache_num_set_bits)
{
  assert(cache_num_set_bits == L2_CACHE_NUM_SETS_BITS);  // ensure the l2 bank mapping functions agree with l2 set bits

  if (MAP_L2BANKS_TO_LOWEST_BITS) {
    return addr.bitSelect(RubyConfig::dataBlockBits()+RubyConfig::L2CachePerChipBits(),
                          RubyConfig::dataBlockBits()+RubyConfig::L2CachePerChipBits()+cache_num_set_bits-1);
  } else {
    return addr.bitSelect(RubyConfig::dataBlockBits(),
                          RubyConfig::dataBlockBits()+cache_num_set_bits-1);
  }
}

// input parameter is the base ruby node of the L1 cache
// returns a value between 0 and total_L2_Caches_within_the_system
inline
MachineID map_L1CacheMachId_to_L2Cache(const Address& addr, MachineID L1CacheMachId)
{
  int L2bank = 0;
  MachineID mach = {MACHINETYPE_L2CACHE_ENUM, 0};

  if (RubyConfig::L2CachePerChipBits() > 0) {
    if (MAP_L2BANKS_TO_LOWEST_BITS) {
      L2bank = addr.bitSelect(RubyConfig::dataBlockBits(),
                         RubyConfig::dataBlockBits()+RubyConfig::L2CachePerChipBits()-1);
    } else {
      L2bank = addr.bitSelect(RubyConfig::dataBlockBits()+L2_CACHE_NUM_SETS_BITS,
                         RubyConfig::dataBlockBits()+L2_CACHE_NUM_SETS_BITS+RubyConfig::L2CachePerChipBits()-1);
    }
  }

  assert(L2bank < RubyConfig::numberOfL2CachePerChip());
  assert(L2bank >= 0);

  mach.num = RubyConfig::L1CacheNumToL2Base(L1CacheMachId.num)*RubyConfig::numberOfL2CachePerChip() // base #
    + L2bank;  // bank #
  assert(mach.num < RubyConfig::numberOfL2Cache());
  return mach;
}

// used to determine the correct L2 bank
// input parameter is the base ruby node of the L2 cache
// returns a value between 0 and total_L2_Caches_within_the_system
inline
MachineID map_L2ChipId_to_L2Cache(const Address& addr, NodeID L2ChipId)
{
  assert(L2ChipId < RubyConfig::numberOfChips());

  int L2bank = 0;
  MachineID mach = {MACHINETYPE_L2CACHE_ENUM, 0};

  if (RubyConfig::L2CachePerChipBits() > 0) {
    if (MAP_L2BANKS_TO_LOWEST_BITS) {
      L2bank = addr.bitSelect(RubyConfig::dataBlockBits(),
                         RubyConfig::dataBlockBits()+RubyConfig::L2CachePerChipBits()-1);
    } else {
      L2bank = addr.bitSelect(RubyConfig::dataBlockBits()+L2_CACHE_NUM_SETS_BITS,
                         RubyConfig::dataBlockBits()+L2_CACHE_NUM_SETS_BITS+RubyConfig::L2CachePerChipBits()-1);
    }
  }

  assert(L2bank < RubyConfig::numberOfL2CachePerChip());
  assert(L2bank >= 0);

  mach.num = L2ChipId*RubyConfig::numberOfL2CachePerChip() // base #
    + L2bank; // bank #
  assert(mach.num < RubyConfig::numberOfL2Cache());
  return mach;
}

// used to determine the home directory
// returns a value between 0 and total_directories_within_the_system
inline
NodeID map_Address_to_DirectoryNode(const Address& addr)
{
  NodeID dirNode = 0;

  if (RubyConfig::memoryBits() > 0) {
    dirNode = addr.bitSelect(RubyConfig::dataBlockBits(),
                        RubyConfig::dataBlockBits()+RubyConfig::memoryBits()-1);
  }

  //  Index indexHighPortion = address.bitSelect(MEMORY_SIZE_BITS-1, PAGE_SIZE_BITS+NUMBER_OF_MEMORY_MODULE_BITS);
  //  Index indexLowPortion  = address.bitSelect(DATA_BLOCK_BITS, PAGE_SIZE_BITS-1);

  //Index index = indexLowPortion | (indexHighPortion << (PAGE_SIZE_BITS - DATA_BLOCK_BITS));

/*

ADDRESS_WIDTH    MEMORY_SIZE_BITS        PAGE_SIZE_BITS  DATA_BLOCK_BITS
  |                    |                       |               |
 \ /                  \ /                     \ /             \ /       0
  -----------------------------------------------------------------------
  |       unused        |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  |                     |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  -----------------------------------------------------------------------
                        indexHighPortion         indexLowPortion
                                        <------->
                               NUMBER_OF_MEMORY_MODULE_BITS
  */

  assert(dirNode < RubyConfig::numberOfMemories());
  assert(dirNode >= 0);
  return dirNode;
}

// used to determine the home directory
// returns a value between 0 and total_directories_within_the_system
inline
MachineID map_Address_to_Directory(const Address &addr)
{
  MachineID mach = {MachineType_Directory, map_Address_to_DirectoryNode(addr)};
  return mach;
}

inline
MachineID map_Address_to_CentralArbiterNode(const Address& addr)
{
  MachineType t = MACHINETYPE_PERSISTENTARBITER_ENUM;
  MachineID mach = {t, map_Address_to_DirectoryNode(addr)};

  assert(mach.num < RubyConfig::numberOfMemories());
  assert(mach.num >= 0);
  return mach;
}

inline
NetDest getMultiStaticL2BankNetDest(const Address& addr, const Set& sharers)  // set of L2RubyNodes
{
  NetDest dest;

  for (int i = 0; i < sharers.getSize(); i++) {
    if (sharers.isElement(i)) {
      dest.add(map_L2ChipId_to_L2Cache(addr,i));
    }
  }
  return dest;
}

inline
NetDest getOtherLocalL1IDs(MachineID L1)
{
  int start = (L1.num / RubyConfig::numberOfProcsPerChip()) * RubyConfig::numberOfProcsPerChip();
  NetDest ret;

  assert(MACHINETYPE_L1CACHE_ENUM != MachineType_NUM);

  for (int i = start; i < (start + RubyConfig::numberOfProcsPerChip()); i++) {
    if (i != L1.num) {
      MachineID mach = { MACHINETYPE_L1CACHE_ENUM, i };
      ret.add( mach );
    }
  }

  return ret;
}

inline
NetDest getLocalL1IDs(MachineID mach)
{
  assert(MACHINETYPE_L1CACHE_ENUM != MachineType_NUM);

  NetDest ret;

  if (mach.type == MACHINETYPE_L1CACHE_ENUM) {

    int start = (mach.num / RubyConfig::numberOfL1CachePerChip()) * RubyConfig::numberOfProcsPerChip();

    for (int i = start; i < (start + RubyConfig::numberOfProcsPerChip()); i++) {
      MachineID mach = { MACHINETYPE_L1CACHE_ENUM, i };
      ret.add( mach );
    }
  }
  else if (mach.type == MACHINETYPE_L2CACHE_ENUM) {

    int chip = mach.num/RubyConfig::numberOfL2CachePerChip();
    int start = ( chip*RubyConfig::numberOfL1CachePerChip());
    for (int i = start; i < (start + RubyConfig::numberOfL1CachePerChip()); i++) {
      MachineID mach = { MACHINETYPE_L1CACHE_ENUM, i };
      ret.add( mach );
    }
  }

  return ret;
}

inline
NetDest getExternalL1IDs(MachineID L1)
{
  NetDest ret;

  assert(MACHINETYPE_L1CACHE_ENUM != MachineType_NUM);

  for (int i = 0; i < RubyConfig::numberOfProcessors(); i++) {
    // ret.add( (NodeID) i);
    MachineID mach = { MACHINETYPE_L1CACHE_ENUM, i };
    ret.add( mach );
  }

  ret.removeNetDest(getLocalL1IDs(L1));

  return ret;
}

inline
bool isLocalProcessor(MachineID thisId, MachineID tarID)
{
  int start = (thisId.num / RubyConfig::numberOfProcsPerChip()) * RubyConfig::numberOfProcsPerChip();

  for (int i = start; i < (start + RubyConfig::numberOfProcsPerChip()); i++) {
    if (i == tarID.num) {
      return true;
    }
  }
  return false;
}


inline
NetDest getAllPertinentL2Banks(const Address& addr)  // set of L2RubyNodes
{
  NetDest dest;

  for (int i = 0; i < RubyConfig::numberOfChips(); i++) {
    dest.add(map_L2ChipId_to_L2Cache(addr,i));
  }
  return dest;
}

inline
bool isL1OnChip(MachineID L1machID, NodeID L2NodeID)
{
  if (L1machID.type == MACHINETYPE_L1CACHE_ENUM) {
    return (L1machID.num == L2NodeID);
  } else {
    return false;
  }
}

inline
bool isL2OnChip(MachineID L2machID, NodeID L2NodeID)
{
  if (L2machID.type == MACHINETYPE_L2CACHE_ENUM) {
    return (L2machID.num == L2NodeID);
  } else {
    return false;
  }
}

inline
NodeID closest_clockwise_distance(NodeID this_node, NodeID next_node)
{
  if (this_node <= next_node) {
    return (next_node - this_node);
  } else {
    return (next_node - this_node + RubyConfig::numberOfChips());
  }
}

inline
bool closer_clockwise_processor(NodeID this_node, NodeID newer, NodeID older)
{
  return (closest_clockwise_distance(this_node, newer) < closest_clockwise_distance(this_node, older));
}

extern inline NodeID getChipID(MachineID L2machID)
{
  return (L2machID.num%RubyConfig::numberOfChips())/RubyConfig::numberOfProcsPerChip();
}

extern inline NodeID machineIDToNodeID(MachineID machID)
{
  // return machID.num%RubyConfig::numberOfChips();
  return machID.num;
}

extern inline NodeID machineIDToVersion(MachineID machID)
{
  return machID.num/RubyConfig::numberOfChips();
}

extern inline MachineType machineIDToMachineType(MachineID machID)
{
  return machID.type;
}

extern inline NodeID L1CacheMachIDToProcessorNum(MachineID machID)
{
  assert(machID.type == MachineType_L1Cache);
  return machID.num;
}

extern inline NodeID L2CacheMachIDToChipID(MachineID machID)
{
  assert(machID.type == MACHINETYPE_L2CACHE_ENUM);
  return machID.num/RubyConfig::numberOfL2CachePerChip();
}

extern inline MachineID getCollectorDest(MachineID L1MachID)
{
  MachineID mach = {MACHINETYPE_COLLECTOR_ENUM, L1MachID.num};
  return mach;
}

extern inline MachineID getCollectorL1Cache(MachineID colID)
{
  MachineID mach = {MACHINETYPE_L1CACHE_ENUM, colID.num};
  return mach;
}

extern inline MachineID getL1MachineID(NodeID L1RubyNode)
{
  MachineID mach = {MACHINETYPE_L1CACHE_ENUM, L1RubyNode};
  return mach;
}

extern inline GenericMachineType ConvertMachToGenericMach(MachineType machType) {
  if (machType == MACHINETYPE_L1CACHE_ENUM) {
    return GenericMachineType_L1Cache;
  } else if (machType == MACHINETYPE_L2CACHE_ENUM) {
    return GenericMachineType_L2Cache;
  } else if (machType == MACHINETYPE_L3CACHE_ENUM) {
    return GenericMachineType_L3Cache;
  } else if (machType == MachineType_Directory) {
    return GenericMachineType_Directory;
  } else if (machType == MACHINETYPE_COLLECTOR_ENUM) {
    return GenericMachineType_Collector;
  } else {
    ERROR_MSG("cannot convert to a GenericMachineType");
    return GenericMachineType_NULL;
  }
}


#endif  // COMPONENTMAPPINGFNS_H
