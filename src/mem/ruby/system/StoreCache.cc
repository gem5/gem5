
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
 *
 */

#include "StoreCache.hh"
#include "System.hh"
#include "Driver.hh"
#include "Vector.hh"
#include "DataBlock.hh"
#include "SubBlock.hh"
#include "Map.hh"

// Helper class
struct StoreCacheEntry {
  StoreCacheEntry() {
    m_byte_counters.setSize(RubyConfig::dataBlockBytes());
    for(int i=0; i<m_byte_counters.size(); i++) {
      m_byte_counters[i] = 0;
    }
    m_line_counter = 0;

  }
  Address m_addr;
  DataBlock m_datablock;
  Vector<int> m_byte_counters;
  int m_line_counter;
};

StoreCache::StoreCache()
{
  m_internal_cache_ptr = new Map<Address, StoreCacheEntry>;
}

StoreCache::~StoreCache()
{
  delete m_internal_cache_ptr;
}

bool StoreCache::isEmpty() const
{
  return m_internal_cache_ptr->size() == 0;
}

int StoreCache::size() const { return m_internal_cache_ptr->size(); }

void StoreCache::add(const SubBlock& block)
{
  if (m_internal_cache_ptr->exist(line_address(block.getAddress())) == false) {
    m_internal_cache_ptr->allocate(line_address(block.getAddress()));
  }

  StoreCacheEntry& entry = m_internal_cache_ptr->lookup(line_address(block.getAddress()));

  // For each byte in entry change the bytes and inc. the counters
  int starting_offset = block.getAddress().getOffset();
  int size = block.getSize();
  for (int index=0; index < size; index++) {
    // Update counter
    entry.m_byte_counters[starting_offset+index]++;

    // Record data
    entry.m_datablock.setByte(starting_offset+index, block.getByte(index));

    DEBUG_EXPR(SEQUENCER_COMP, LowPrio, block.getAddress());
    DEBUG_EXPR(SEQUENCER_COMP, LowPrio, int(block.getByte(index)));
    DEBUG_EXPR(SEQUENCER_COMP, LowPrio, starting_offset+index);
  }

  // Increment the counter
  entry.m_line_counter++;
}

void StoreCache::remove(const SubBlock& block)
{
  assert(m_internal_cache_ptr->exist(line_address(block.getAddress())));

  StoreCacheEntry& entry = m_internal_cache_ptr->lookup(line_address(block.getAddress()));

  // Decrement the byte counters
  int starting_offset = block.getAddress().getOffset();
  int size = block.getSize();
  for (int index=0; index < size; index++) {
    // Update counter
    entry.m_byte_counters[starting_offset+index]--;
  }

  // Decrement the line counter
  entry.m_line_counter--;
  assert(entry.m_line_counter >= 0);

  // Check to see if we should de-allocate this entry
  if (entry.m_line_counter == 0) {
    m_internal_cache_ptr->deallocate(line_address(block.getAddress()));
  }
}

bool StoreCache::check(const SubBlock& block) const
{
  if (m_internal_cache_ptr->exist(line_address(block.getAddress())) == false) {
    return false;
  } else {
    // Lookup the entry
    StoreCacheEntry& entry = m_internal_cache_ptr->lookup(line_address(block.getAddress()));

    // See if all the bytes are valid
    int starting_offset = block.getAddress().getOffset();
    int size = block.getSize();
    for (int index=0; index < size; index++) {
      if (entry.m_byte_counters[starting_offset+index] > 0) {
        // So far so good
      } else {
        // not all the bytes were valid
        return false;
      }
    }
  }
  return true;
}

void StoreCache::update(SubBlock& block) const
{
  if (m_internal_cache_ptr->exist(line_address(block.getAddress()))) {
    // Lookup the entry
    StoreCacheEntry& entry = m_internal_cache_ptr->lookup(line_address(block.getAddress()));

    // Copy all appropriate and valid bytes from the store cache to
    // the SubBlock
    int starting_offset = block.getAddress().getOffset();
    int size = block.getSize();
    for (int index=0; index < size; index++) {

      DEBUG_EXPR(SEQUENCER_COMP, LowPrio, block.getAddress());
      DEBUG_EXPR(SEQUENCER_COMP, LowPrio, int(entry.m_datablock.getByte(starting_offset+index)));
      DEBUG_EXPR(SEQUENCER_COMP, LowPrio, starting_offset+index);

      // If this byte is valid, copy the data into the sub-block
      if (entry.m_byte_counters[starting_offset+index] > 0) {
        block.setByte(index, entry.m_datablock.getByte(starting_offset+index));
      }
    }
  }
}

void StoreCache::print(ostream& out) const
{
  out << "[StoreCache]";
}

