
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
#include "mem/ruby/system/DirectoryMemory.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/gems_common/util.hh"

int DirectoryMemory::m_num_directories = 0;
int DirectoryMemory::m_num_directories_bits = 0;
int DirectoryMemory::m_total_size_bytes = 0;

DirectoryMemory::DirectoryMemory(const string & name)
 : m_name(name)
{
}

void DirectoryMemory::init(const vector<string> & argv)
{
  m_controller = NULL;
  for (vector<string>::const_iterator it = argv.begin(); it != argv.end(); it++) {
    if ( (*it) == "version" )
      m_version = atoi( (*(++it)).c_str() );
    else if ( (*it) == "size_mb" ) {
      m_size_bytes = atoi((*(++it)).c_str()) * static_cast<uint64>(1<<20);
      m_size_bits = log_int(m_size_bytes);
    } else if ( (*it) == "controller" ) {
      m_controller = RubySystem::getController((*(++it)));
    } else {
      cerr << "DirectoryMemory: Unkown config parameter: " << (*it) << endl;
      assert(0);
    }
  }
  assert(m_controller != NULL);

  m_num_entries = m_size_bytes / RubySystem::getBlockSizeBytes();
  m_entries = new Directory_Entry*[m_num_entries];
  for (int i=0; i < m_num_entries; i++)
    m_entries[i] = NULL;

  m_ram = g_system_ptr->getMemoryVector();

  m_num_directories++;
  m_num_directories_bits = log_int(m_num_directories);
  m_total_size_bytes += m_size_bytes;
}

DirectoryMemory::~DirectoryMemory()
{
  // free up all the directory entries
  for (int i=0;i<m_num_entries;i++)
    if (m_entries[i] != NULL)
      delete m_entries;
  if (m_entries != NULL)
    delete [] m_entries;
}

void DirectoryMemory::printConfig(ostream& out) const
{
  out << "DirectoryMemory module config: " << m_name << endl;
  out << "  controller: " << m_controller->getName() << endl;
  out << "  version: " << m_version << endl;
  out << "  memory_bits: " << m_size_bits << endl;
  out << "  memory_size_bytes: " << m_size_bytes << endl;
  out << "  memory_size_Kbytes: " << double(m_size_bytes) / (1<<10) << endl;
  out << "  memory_size_Mbytes: " << double(m_size_bytes) / (1<<20) << endl;
  out << "  memory_size_Gbytes: " << double(m_size_bytes) / (1<<30) << endl;
}

// Static method
void DirectoryMemory::printGlobalConfig(ostream & out)
{
  out << "DirectoryMemory Global Config: " << endl;
  out << "  number of directory memories: " << m_num_directories << endl;
  if (m_num_directories > 1) {
    out << "  number of selection bits: " << m_num_directories_bits << endl;
    out << "  selection bits: " << RubySystem::getBlockSizeBits()+m_num_directories_bits-1
        << "-" << RubySystem::getBlockSizeBits() << endl;
  }
  out << "  total memory size bytes: " << m_total_size_bytes << endl;
  out << "  total memory size bits: " << log_int(m_total_size_bytes) << endl;

}

int DirectoryMemory::mapAddressToDirectoryVersion(PhysAddress address)
{
  if (m_num_directories_bits == 0) return 0;
  int ret = address.bitSelect(RubySystem::getBlockSizeBits(),
                              RubySystem::getBlockSizeBits()+m_num_directories_bits-1);
  return ret;
}

// Public method
bool DirectoryMemory::isPresent(PhysAddress address)
{
  bool ret = (mapAddressToDirectoryVersion(address) == m_version);
  return ret;
}

int DirectoryMemory::mapAddressToLocalIdx(PhysAddress address)
{
  int ret = address.getAddress() >> (RubySystem::getBlockSizeBits() + m_num_directories_bits);
  return ret;
}

Directory_Entry& DirectoryMemory::lookup(PhysAddress address)
{
  assert(isPresent(address));
  Directory_Entry* entry;
  int idx = mapAddressToLocalIdx(address);
  entry = m_entries[idx];
  if (entry == NULL) {
    entry = new Directory_Entry;
    entry->getDataBlk().assign(m_ram->getBlockPtr(address));
    m_entries[idx] = entry;
  }
  return (*entry);
}
/*
Directory_Entry& DirectoryMemory::lookup(PhysAddress address)
{
  assert(isPresent(address));
  Index index = address.memoryModuleIndex();

  if (index < 0 || index > m_size) {
    WARN_EXPR(address.getAddress());
    WARN_EXPR(index);
    WARN_EXPR(m_size);
    ERROR_MSG("Directory Memory Assertion: accessing memory out of range.");
  }
  Directory_Entry* entry = m_entries[index];

  // allocate the directory entry on demand.
  if (entry == NULL) {
    entry = new Directory_Entry;
    entry->getDataBlk().assign(m_ram->getBlockPtr(address));

    // store entry to the table
    m_entries[index] = entry;
  }

  return (*entry);
}
*/

void DirectoryMemory::invalidateBlock(PhysAddress address)
{
  /*
  assert(isPresent(address));

  Index index = address.memoryModuleIndex();

  if (index < 0 || index > m_size) {
    ERROR_MSG("Directory Memory Assertion: accessing memory out of range.");
  }

  if(m_entries[index] != NULL){
    delete m_entries[index];
    m_entries[index] = NULL;
  }
  */
}

void DirectoryMemory::print(ostream& out) const
{

}

