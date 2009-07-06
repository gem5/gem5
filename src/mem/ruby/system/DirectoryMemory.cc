
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
 * DirectoryMemory.cc
 *
 * Description: See DirectoryMemory.hh
 *
 * $Id$
 *
 */

#include "mem/ruby/system/System.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/system/DirectoryMemory.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/protocol/Chip.hh"

DirectoryMemory::DirectoryMemory(Chip* chip_ptr, int version)
{
  m_chip_ptr = chip_ptr;
  m_version = version;
  // THIS DOESN'T SEEM TO WORK -- MRM
  // m_size = RubyConfig::memoryModuleBlocks()/RubyConfig::numberOfDirectory();
  m_size = RubyConfig::memoryModuleBlocks();
  assert(m_size > 0);
  /*********************************************************************
  // allocates an array of directory entry pointers & sets them to NULL
  m_entries = new Directory_Entry*[m_size];
  if (m_entries == NULL) {
    ERROR_MSG("Directory Memory: unable to allocate memory.");
  }

  for (int i=0; i < m_size; i++) {
    m_entries[i] = NULL;
  }
  */////////////////////////////////////////////////////////////////////
}

DirectoryMemory::~DirectoryMemory()
{
  /*********************************************************************
  // free up all the directory entries
  for (int i=0; i < m_size; i++) {
    if (m_entries[i] != NULL) {
      delete m_entries[i];
      m_entries[i] = NULL;
    }
  }

  // free up the array of directory entries
  delete[] m_entries;
  *//////////////////////////////////////////////////////////////////////
  m_entries.clear();
}

// Static method
void DirectoryMemory::printConfig(ostream& out)
{
  out << "Memory config:" << endl;
  out << "  memory_bits: " << RubyConfig::memorySizeBits() << endl;
  out << "  memory_size_bytes: " << RubyConfig::memorySizeBytes() << endl;
  out << "  memory_size_Kbytes: " << double(RubyConfig::memorySizeBytes()) / (1<<10) << endl;
  out << "  memory_size_Mbytes: " << double(RubyConfig::memorySizeBytes()) / (1<<20) << endl;
  out << "  memory_size_Gbytes: " << double(RubyConfig::memorySizeBytes()) / (1<<30) << endl;

  out << "  module_bits: " << RubyConfig::memoryModuleBits() << endl;
  out << "  module_size_lines: " << RubyConfig::memoryModuleBlocks() << endl;
  out << "  module_size_bytes: " << RubyConfig::memoryModuleBlocks() * RubyConfig::dataBlockBytes() << endl;
  out << "  module_size_Kbytes: " << double(RubyConfig::memoryModuleBlocks() * RubyConfig::dataBlockBytes()) / (1<<10) << endl;
  out << "  module_size_Mbytes: " << double(RubyConfig::memoryModuleBlocks() * RubyConfig::dataBlockBytes()) / (1<<20) << endl;
}

// Public method
bool DirectoryMemory::isPresent(PhysAddress address)
{
  return (map_Address_to_DirectoryNode(address) == m_chip_ptr->getID()*RubyConfig::numberOfDirectoryPerChip()+m_version);
}

void DirectoryMemory::readPhysMem(uint64 address, int size, void * data)
{
}

Directory_Entry& DirectoryMemory::lookup(PhysAddress address)
{
  assert(isPresent(address));
  Index index = address.memoryModuleIndex();

  if (index < 0 || index > m_size) {
    WARN_EXPR(m_chip_ptr->getID());
    WARN_EXPR(address.getAddress());
    WARN_EXPR(index);
    WARN_EXPR(m_size);
    ERROR_MSG("Directory Memory Assertion: accessing memory out of range.");
  }

  map<Index, Directory_Entry*>::iterator iter =  m_entries.find(index);
  Directory_Entry* entry = m_entries.find(index)->second;

  // allocate the directory entry on demand.
  if (iter == m_entries.end()) {
    entry = new Directory_Entry;

    //    entry->getProcOwner() = m_chip_ptr->getID(); // FIXME - This should not be hard coded
    //    entry->getDirOwner() = true;        // FIXME - This should not be hard-coded

    // load the data from physicalMemory when first initalizing
    physical_address_t physAddr = address.getAddress();
    int8 * dataArray = (int8 * )malloc(RubyConfig::dataBlockBytes() * sizeof(int8));
    readPhysMem(physAddr, RubyConfig::dataBlockBytes(), dataArray);

    for(int j=0; j < RubyConfig::dataBlockBytes(); j++) {
      entry->getDataBlk().setByte(j, dataArray[j]);
    }
    DEBUG_EXPR(NODE_COMP, MedPrio,entry->getDataBlk());
    // store entry to the table
    m_entries.insert(make_pair(index, entry));
  }
  return (*entry);
}

/*
void DirectoryMemory::invalidateBlock(PhysAddress address)
{
  assert(isPresent(address));

  Index index = address.memoryModuleIndex();

  if (index < 0 || index > m_size) {
    ERROR_MSG("Directory Memory Assertion: accessing memory out of range.");
  }

  if(m_entries[index] != NULL){
    delete m_entries[index];
    m_entries[index] = NULL;
  }

}
*/

void DirectoryMemory::print(ostream& out) const
{
  out << "Directory dump: " << endl;
  for(map<Index, Directory_Entry*>::const_iterator it = m_entries.begin(); it != m_entries.end(); ++it) {
      out << it->first << ": ";
      out << *(it->second) << endl;
  }
}

