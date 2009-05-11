
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

#ifndef DRIVER_H
#define DRIVER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/packet.hh"

class RubySystem;
class SubBlock;
class Address;
class MachineID;
class SimicsHypervisor;

class Driver {
public:
  // Constructors
  Driver();

  // Destructor
  virtual ~Driver() = 0;

  // Public Methods
  virtual void get_network_config() {}
  virtual void hitCallback(Packet* pkt) = 0;
  virtual integer_t getInstructionCount(int procID) const { return 1; }
  virtual integer_t getCycleCount(int procID) const { return 1; }

  virtual void printDebug(){}  //called by Sequencer

  virtual void printStats(ostream& out) const = 0;
  virtual void clearStats() = 0;

  virtual void printConfig(ostream& out) const = 0;

  virtual integer_t readPhysicalMemory(int procID, physical_address_t address,
                                       int len ){ ASSERT(0); return 0; }

  virtual void writePhysicalMemory( int procID, physical_address_t address,
                                    integer_t value, int len ){ ASSERT(0); }

protected:
  // accessible by subclasses

private:
  // inaccessible by subclasses

};

#endif //DRIVER_H
