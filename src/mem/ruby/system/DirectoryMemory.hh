
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
 * DirectoryMemory.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef DIRECTORYMEMORY_H
#define DIRECTORYMEMORY_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/protocol/Directory_Entry.hh"
#include "sim/sim_object.hh"
#include "params/RubyDirectoryMemory.hh"

class AbstractController;

class DirectoryMemory : public SimObject {
public:
  // Constructors
    typedef RubyDirectoryMemoryParams Params;
    DirectoryMemory(const Params *p);
  void init();
  //  DirectoryMemory(int version);

  // Destructor
  ~DirectoryMemory();

  int mapAddressToLocalIdx(PhysAddress address);
  static int mapAddressToDirectoryVersion(PhysAddress address);

  uint64 getSize() { return m_size_bytes; }

  // Public Methods
  void printConfig(ostream& out) const;
  static void printGlobalConfig(ostream & out);
  bool isPresent(PhysAddress address);
  Directory_Entry& lookup(PhysAddress address);

  void invalidateBlock(PhysAddress address);

  void print(ostream& out) const;

private:
  // Private Methods

  // Private copy constructor and assignment operator
  DirectoryMemory(const DirectoryMemory& obj);
  DirectoryMemory& operator=(const DirectoryMemory& obj);

private:
  const string m_name;
  AbstractController* m_controller;
  // Data Members (m_ prefix)
  Directory_Entry **m_entries;
  //  int m_size;  // # of memory module blocks this directory is responsible for
  uint64 m_size_bytes;
  uint64 m_size_bits;
  int m_num_entries;
  int m_version;

  static int m_num_directories;
  static int m_num_directories_bits;
  static uint64_t m_total_size_bytes;

  MemoryVector* m_ram;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DirectoryMemory& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const DirectoryMemory& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DIRECTORYMEMORY_H
