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
 * CacheMemory.hh
 *
 * Description:
 *
 * $Id: CacheMemory.hh,v 3.7 2004/06/18 20:15:15 beckmann Exp $
 *
 */

#ifndef CACHEMEMORY_H
#define CACHEMEMORY_H

#include "mem/ruby/common/Global.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/recorder/CacheRecorder.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "mem/ruby/system/PseudoLRUPolicy.hh"
#include "mem/ruby/system/LRUPolicy.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include <vector>

class CacheMemory {
public:

  // Constructors
  CacheMemory(const string & name);
  void init(const vector<string> & argv);

  // Destructor
  ~CacheMemory();

  // factory
  //  static CacheMemory* createCache(int level, int num, char split_type, AbstractCacheEntry* (*entry_factory)());
  //  static CacheMemory* getCache(int cache_id);

  // Public Methods
  void printConfig(ostream& out);

  // perform a cache access and see if we hit or not.  Return true on a hit.
  bool tryCacheAccess(const Address& address, CacheRequestType type, DataBlock*& data_ptr);

  // similar to above, but doesn't require full access check
  bool testCacheAccess(const Address& address, CacheRequestType type, DataBlock*& data_ptr);

  // tests to see if an address is present in the cache
  bool isTagPresent(const Address& address) const;

  // Returns true if there is:
  //   a) a tag match on this address or there is
  //   b) an unused line in the same cache "way"
  bool cacheAvail(const Address& address) const;

  // find an unused entry and sets the tag appropriate for the address
  void allocate(const Address& address, AbstractCacheEntry* new_entry);

  // Explicitly free up this address
  void deallocate(const Address& address);

  // Returns with the physical address of the conflicting cache line
  Address cacheProbe(const Address& address) const;

  // looks an address up in the cache
  AbstractCacheEntry& lookup(const Address& address);
  const AbstractCacheEntry& lookup(const Address& address) const;

  // Get/Set permission of cache block
  AccessPermission getPermission(const Address& address) const;
  void changePermission(const Address& address, AccessPermission new_perm);

  int getLatency() const { return m_latency; }

  // Hook for checkpointing the contents of the cache
  void recordCacheContents(CacheRecorder& tr) const;
  void setAsInstructionCache(bool is_icache) { m_is_instruction_only_cache = is_icache; }

  // Set this address to most recently used
  void setMRU(const Address& address);

  void getMemoryValue(const Address& addr, char* value,
                      unsigned int size_in_bytes );
  void setMemoryValue(const Address& addr, char* value,
                      unsigned int size_in_bytes );

  void setLocked (const Address& addr, int context);
  void clearLocked (const Address& addr);
  bool isLocked (const Address& addr, int context);
  // Print cache contents
  void print(ostream& out) const;
  void printData(ostream& out) const;

private:
  // Private Methods

  // convert a Address to its location in the cache
  Index addressToCacheSet(const Address& address) const;

  // Given a cache tag: returns the index of the tag in a set.
  // returns -1 if the tag is not found.
  int findTagInSet(Index line, const Address& tag) const;
  int findTagInSetIgnorePermissions(Index cacheSet, const Address& tag) const;

  // Private copy constructor and assignment operator
  CacheMemory(const CacheMemory& obj);
  CacheMemory& operator=(const CacheMemory& obj);

private:
  const string m_cache_name;
  AbstractController* m_controller;
  int m_latency;

  // Data Members (m_prefix)
  bool m_is_instruction_only_cache;
  bool m_is_data_only_cache;

  // The first index is the # of cache lines.
  // The second index is the the amount associativity.
  Vector<Vector<AbstractCacheEntry*> > m_cache;
  Vector<Vector<int> > m_locked;

  AbstractReplacementPolicy *m_replacementPolicy_ptr;

  int m_cache_num_sets;
  int m_cache_num_set_bits;
  int m_cache_assoc;

  static Vector< CacheMemory* > m_all_caches;
};
/*
inline
CacheMemory* CacheMemory::getCache(int cache_id)
{
  assert(cache_id < RubyConfig::getNumberOfCaches());
  if (m_all_caches[cache_id] == NULL) {
    cerr << "ERROR: Tried to obtain CacheMemory that hasn't been created yet." << endl;
    assert(0);
  }
  return m_all_caches[cache_id];
}

inline
CacheMemory* CacheMemory::createCache(int level, int num, char split_type_c, AbstractCacheEntry* (*entry_factory)())
{
  string split_type;
  switch(split_type_c) {
  case 'i':
    split_type = "instruction"; break;
  case 'd':
    split_type = "data"; break;
  default:
    split_type = "unified"; break;
  }
  int cache_id = RubyConfig::getCacheIDFromParams(level, num, split_type);
  assert(cache_id < RubyConfig::getNumberOfCaches());
  if (m_all_caches.size() == 0) {
    m_all_caches.setSize(RubyConfig::getNumberOfCaches());
    for (int i=0; i<m_all_caches.size(); i++)
      m_all_caches[i] = NULL;
  }

  string type = RubyConfig::getCacheType(cache_id);
  if ( type == "SetAssociativeCache" ) {
    m_all_caches[cache_id] = new CacheMemory(cache_id, entry_factory);
  }
  return m_all_caches[cache_id];
}
*/
// Output operator declaration
//ostream& operator<<(ostream& out, const CacheMemory<ENTRY>& obj);

// ******************* Definitions *******************

// Output operator definition
inline
ostream& operator<<(ostream& out, const CacheMemory& obj)
{
  obj.print(out);
  out << flush;
  return out;
}


// ****************************************************************

inline
CacheMemory::CacheMemory(const string & name)
  : m_cache_name(name)
{
}

inline
void CacheMemory::init(const vector<string> & argv)
{
  int cache_size = 0;
  string policy;

  m_controller = NULL;
  for (uint32 i=0; i<argv.size(); i+=2) {
    if (argv[i] == "size_kb") {
      cache_size = atoi(argv[i+1].c_str());
    } else if (argv[i] == "latency") {
      m_latency = atoi(argv[i+1].c_str());
    } else if (argv[i] == "assoc") {
      m_cache_assoc = atoi(argv[i+1].c_str());
    } else if (argv[i] == "replacement_policy") {
      policy = argv[i+1];
    } else if (argv[i] == "controller") {
      m_controller = RubySystem::getController(argv[i+1]);
    } else {
      cerr << "WARNING: CacheMemory: Unknown configuration parameter: " << argv[i] << endl;
    }
  }

  m_cache_num_sets = cache_size / m_cache_assoc;
  m_cache_num_set_bits = log_int(m_cache_num_sets);

  if(policy == "PSEUDO_LRU")
    m_replacementPolicy_ptr = new PseudoLRUPolicy(m_cache_num_sets, m_cache_assoc);
  else if (policy == "LRU")
    m_replacementPolicy_ptr = new LRUPolicy(m_cache_num_sets, m_cache_assoc);
  else
    assert(false);

  m_cache.setSize(m_cache_num_sets);
  m_locked.setSize(m_cache_num_sets);
  for (int i = 0; i < m_cache_num_sets; i++) {
    m_cache[i].setSize(m_cache_assoc);
    m_locked[i].setSize(m_cache_assoc);
    for (int j = 0; j < m_cache_assoc; j++) {
      m_cache[i][j] = NULL;
      m_locked[i][j] = -1;
    }
  }
}
/*
inline
CacheMemory::CacheMemory(int cache_id, AbstractCacheEntry* (*entry_factory)())
{
  string split_type;

  m_cache_id = cache_id;
  m_entry_factory = entry_factory;

  m_cache_num_set_bits = RubyConfig::getNumberOfCacheSetBits(cache_id);
  m_cache_num_sets = RubyConfig::getNumberOfCacheSets(cache_id);
  m_cache_assoc = RubyConfig::getCacheAssoc(cache_id);
  split_type = RubyConfig::getCacheSplitType(cache_id);
  m_is_instruction_only_cache = m_is_data_only_cache = false;
  if (split_type == "instruction")
    m_is_instruction_only_cache = true;
  else if (split_type == "data")
    m_is_data_only_cache = true;
  else
    assert(split_type == "unified");

  if(RubyConfig::getCacheReplacementPolicy(cache_id) == "PSEUDO_LRU")
    m_replacementPolicy_ptr = new PseudoLRUPolicy(m_cache_num_sets, m_cache_assoc);
  else if(RubyConfig::getCacheReplacementPolicy(cache_id) == "LRU")
    m_replacementPolicy_ptr = new LRUPolicy(m_cache_num_sets, m_cache_assoc);
  else
    assert(false);

  m_cache.setSize(m_cache_num_sets);
  for (int i = 0; i < m_cache_num_sets; i++) {
    m_cache[i].setSize(m_cache_assoc);
    for (int j = 0; j < m_cache_assoc; j++) {
      m_cache[i][j] = m_entry_factory();
    }
  }
}
*/
inline
CacheMemory::~CacheMemory()
{
  if(m_replacementPolicy_ptr != NULL)
    delete m_replacementPolicy_ptr;
}

inline
void CacheMemory::printConfig(ostream& out)
{
  out << "Cache config: " << m_cache_name << endl;
  if (m_controller != NULL)
    out << "  controller: " << m_controller->getName() << endl;
  out << "  cache_associativity: " << m_cache_assoc << endl;
  out << "  num_cache_sets_bits: " << m_cache_num_set_bits << endl;
  const int cache_num_sets = 1 << m_cache_num_set_bits;
  out << "  num_cache_sets: " << cache_num_sets << endl;
  out << "  cache_set_size_bytes: " << cache_num_sets * RubySystem::getBlockSizeBytes() << endl;
  out << "  cache_set_size_Kbytes: "
      << double(cache_num_sets * RubySystem::getBlockSizeBytes()) / (1<<10) << endl;
  out << "  cache_set_size_Mbytes: "
      << double(cache_num_sets * RubySystem::getBlockSizeBytes()) / (1<<20) << endl;
  out << "  cache_size_bytes: "
      << cache_num_sets * RubySystem::getBlockSizeBytes() * m_cache_assoc << endl;
  out << "  cache_size_Kbytes: "
      << double(cache_num_sets * RubySystem::getBlockSizeBytes() * m_cache_assoc) / (1<<10) << endl;
  out << "  cache_size_Mbytes: "
      << double(cache_num_sets * RubySystem::getBlockSizeBytes() * m_cache_assoc) / (1<<20) << endl;
}

// PRIVATE METHODS

// convert a Address to its location in the cache
inline
Index CacheMemory::addressToCacheSet(const Address& address) const
{
  assert(address == line_address(address));
  return address.bitSelect(RubySystem::getBlockSizeBits(), RubySystem::getBlockSizeBits() + m_cache_num_set_bits-1);
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
inline
int CacheMemory::findTagInSet(Index cacheSet, const Address& tag) const
{
  assert(tag == line_address(tag));
  // search the set for the tags
  for (int i=0; i < m_cache_assoc; i++) {
    if ((m_cache[cacheSet][i] != NULL) &&
        (m_cache[cacheSet][i]->m_Address == tag) &&
        (m_cache[cacheSet][i]->m_Permission != AccessPermission_NotPresent)) {
      return i;
    }
  }
  return -1; // Not found
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
inline
int CacheMemory::findTagInSetIgnorePermissions(Index cacheSet, const Address& tag) const
{
  assert(tag == line_address(tag));
  // search the set for the tags
  for (int i=0; i < m_cache_assoc; i++) {
    if (m_cache[cacheSet][i] != NULL && m_cache[cacheSet][i]->m_Address == tag)
      return i;
  }
  return -1; // Not found
}

// PUBLIC METHODS
inline
bool CacheMemory::tryCacheAccess(const Address& address,
                                 CacheRequestType type,
                                 DataBlock*& data_ptr)
{
  assert(address == line_address(address));
  DEBUG_EXPR(CACHE_COMP, HighPrio, address);
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  if(loc != -1){ // Do we even have a tag match?
    AbstractCacheEntry* entry = m_cache[cacheSet][loc];
    m_replacementPolicy_ptr->touch(cacheSet, loc, g_eventQueue_ptr->getTime());
    data_ptr = &(entry->getDataBlk());

    if(entry->m_Permission == AccessPermission_Read_Write) {
      return true;
    }
    if ((entry->m_Permission == AccessPermission_Read_Only) &&
        (type == CacheRequestType_LD || type == CacheRequestType_IFETCH)) {
      return true;
    }
    // The line must not be accessible
  }
  data_ptr = NULL;
  return false;
}

inline
bool CacheMemory::testCacheAccess(const Address& address,
                                  CacheRequestType type,
                                  DataBlock*& data_ptr)
{
  assert(address == line_address(address));
  DEBUG_EXPR(CACHE_COMP, HighPrio, address);
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  if(loc != -1){ // Do we even have a tag match?
    AbstractCacheEntry* entry = m_cache[cacheSet][loc];
    m_replacementPolicy_ptr->touch(cacheSet, loc, g_eventQueue_ptr->getTime());
    data_ptr = &(entry->getDataBlk());

    return (m_cache[cacheSet][loc]->m_Permission != AccessPermission_NotPresent);
  }
  data_ptr = NULL;
  return false;
}

// tests to see if an address is present in the cache
inline
bool CacheMemory::isTagPresent(const Address& address) const
{
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int location = findTagInSet(cacheSet, address);

  if (location == -1) {
    // We didn't find the tag
    DEBUG_EXPR(CACHE_COMP, LowPrio, address);
    DEBUG_MSG(CACHE_COMP, LowPrio, "No tag match");
    return false;
  }
  DEBUG_EXPR(CACHE_COMP, LowPrio, address);
  DEBUG_MSG(CACHE_COMP, LowPrio, "found");
  return true;
}

// Returns true if there is:
//   a) a tag match on this address or there is
//   b) an unused line in the same cache "way"
inline
bool CacheMemory::cacheAvail(const Address& address) const
{
  assert(address == line_address(address));

  Index cacheSet = addressToCacheSet(address);

  for (int i=0; i < m_cache_assoc; i++) {
    AbstractCacheEntry* entry = m_cache[cacheSet][i];
    if (entry != NULL) {
      if (entry->m_Address == address ||                         // Already in the cache
          entry->m_Permission == AccessPermission_NotPresent) {  // We found an empty entry
        return true;
      }
    } else {
      return true;
    }
  }
  return false;
}

inline
void CacheMemory::allocate(const Address& address, AbstractCacheEntry* entry)
{
  assert(address == line_address(address));
  assert(!isTagPresent(address));
  assert(cacheAvail(address));
  DEBUG_EXPR(CACHE_COMP, HighPrio, address);

  // Find the first open slot
  Index cacheSet = addressToCacheSet(address);
  for (int i=0; i < m_cache_assoc; i++) {
    if (m_cache[cacheSet][i] == NULL ||
        m_cache[cacheSet][i]->m_Permission == AccessPermission_NotPresent) {
      m_cache[cacheSet][i] = entry;  // Init entry
      m_cache[cacheSet][i]->m_Address = address;
      m_cache[cacheSet][i]->m_Permission = AccessPermission_Invalid;
      m_locked[cacheSet][i] = -1;

      m_replacementPolicy_ptr->touch(cacheSet, i, g_eventQueue_ptr->getTime());

      return;
    }
  }
  ERROR_MSG("Allocate didn't find an available entry");
}

inline
void CacheMemory::deallocate(const Address& address)
{
  assert(address == line_address(address));
  assert(isTagPresent(address));
  DEBUG_EXPR(CACHE_COMP, HighPrio, address);
  Index cacheSet = addressToCacheSet(address);
  int location = findTagInSet(cacheSet, address);
  if (location != -1){
    delete m_cache[cacheSet][location];
    m_cache[cacheSet][location] = NULL;
    m_locked[cacheSet][location] = -1;
  }
}

// Returns with the physical address of the conflicting cache line
inline
Address CacheMemory::cacheProbe(const Address& address) const
{
  assert(address == line_address(address));
  assert(!cacheAvail(address));

  Index cacheSet = addressToCacheSet(address);
  return m_cache[cacheSet][m_replacementPolicy_ptr->getVictim(cacheSet)]->m_Address;
}

// looks an address up in the cache
inline
AbstractCacheEntry& CacheMemory::lookup(const Address& address)
{
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  assert(loc != -1);
  return *m_cache[cacheSet][loc];
}

// looks an address up in the cache
inline
const AbstractCacheEntry& CacheMemory::lookup(const Address& address) const
{
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  assert(loc != -1);
  return *m_cache[cacheSet][loc];
}

inline
AccessPermission CacheMemory::getPermission(const Address& address) const
{
  assert(address == line_address(address));
  return lookup(address).m_Permission;
}

inline
void CacheMemory::changePermission(const Address& address, AccessPermission new_perm)
{
  assert(address == line_address(address));
  lookup(address).m_Permission = new_perm;
  m_locked[cacheSet][loc] = -1; 
  assert(getPermission(address) == new_perm);
}

// Sets the most recently used bit for a cache block
inline
void CacheMemory::setMRU(const Address& address)
{
  Index cacheSet;

  cacheSet = addressToCacheSet(address);
  m_replacementPolicy_ptr->touch(cacheSet,
                                 findTagInSet(cacheSet, address),
                                 g_eventQueue_ptr->getTime());
}

inline
void CacheMemory::recordCacheContents(CacheRecorder& tr) const
{
  for (int i = 0; i < m_cache_num_sets; i++) {
    for (int j = 0; j < m_cache_assoc; j++) {
      AccessPermission perm = m_cache[i][j]->m_Permission;
      CacheRequestType request_type = CacheRequestType_NULL;
      if (perm == AccessPermission_Read_Only) {
        if (m_is_instruction_only_cache) {
          request_type = CacheRequestType_IFETCH;
        } else {
          request_type = CacheRequestType_LD;
        }
      } else if (perm == AccessPermission_Read_Write) {
        request_type = CacheRequestType_ST;
      }

      if (request_type != CacheRequestType_NULL) {
        //        tr.addRecord(m_chip_ptr->getID(), m_cache[i][j].m_Address,
        //                     Address(0), request_type, m_replacementPolicy_ptr->getLastAccess(i, j));
      }
    }
  }
}

inline
void CacheMemory::print(ostream& out) const
{
  out << "Cache dump: " << m_cache_name << endl;
  for (int i = 0; i < m_cache_num_sets; i++) {
    for (int j = 0; j < m_cache_assoc; j++) {
      if (m_cache[i][j] != NULL) {
        out << "  Index: " << i
            << " way: " << j
            << " entry: " << *m_cache[i][j] << endl;
      } else {
        out << "  Index: " << i
            << " way: " << j
            << " entry: NULL" << endl;
      }
    }
  }
}

inline
void CacheMemory::printData(ostream& out) const
{
  out << "printData() not supported" << endl;
}

inline
void CacheMemory::getMemoryValue(const Address& addr, char* value,
                                 unsigned int size_in_bytes ){
  AbstractCacheEntry& entry = lookup(line_address(addr));
  unsigned int startByte = addr.getAddress() - line_address(addr).getAddress();
  for(unsigned int i=0; i<size_in_bytes; ++i){
    value[i] = entry.getDataBlk().getByte(i + startByte);
  }
}

inline
void CacheMemory::setMemoryValue(const Address& addr, char* value,
                                 unsigned int size_in_bytes ){
  AbstractCacheEntry& entry = lookup(line_address(addr));
  unsigned int startByte = addr.getAddress() - line_address(addr).getAddress();
  assert(size_in_bytes > 0);
  for(unsigned int i=0; i<size_in_bytes; ++i){
    entry.getDataBlk().setByte(i + startByte, value[i]);
  }

  //  entry = lookup(line_address(addr));
}

inline
void 
CacheMemory::setLocked(const Address& address, int context) 
{  
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  assert(loc != -1);
  m_locked[cacheSet][loc] = context;
}

inline
void 
CacheMemory::clearLocked(const Address& address) 
{
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  assert(loc != -1);
  m_locked[cacheSet][loc] = -1;
}

inline
bool
CacheMemory::isLocked(const Address& address, int context)
{
  assert(address == line_address(address));
  Index cacheSet = addressToCacheSet(address);
  int loc = findTagInSet(cacheSet, address);
  assert(loc != -1);
  return m_locked[cacheSet][loc] == context; 
}

#endif //CACHEMEMORY_H

