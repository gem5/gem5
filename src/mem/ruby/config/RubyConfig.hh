
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
 * RubyConfig.h
 *
 * Description: This class has only static members and class methods,
 * and thus should never need to be instantiated.
 *
 * $Id$
 *
 */

#ifndef RUBYCONFIG_H
#define RUBYCONFIG_H

#include "Global.hh"
#define   CONFIG_VAR_FILENAME "config.include"
#include "vardecl.hh"
#include "NodeID.hh"

#define   MEMORY_LATENCY  RubyConfig::memoryResponseLatency()
#define   ABORT_DELAY  m_chip_ptr->getTransactionManager(m_version)->getAbortDelay()

// Set paramterization
/*
 * This defines the number of longs (32-bits on 32 bit machines,
 * 64-bit on 64-bit AMD machines) to use to hold the set...
 * the default is 4, allowing 128 or 256 different members
 * of the set.
 *
 * This should never need to be changed for correctness reasons,
 * though increasing it will increase performance for larger
 * set sizes at the cost of a (much) larger memory footprint
 *
 */
const int NUMBER_WORDS_PER_SET = 4;

class RubyConfig {
public:

  // CACHE BLOCK CONFIG VARIBLES
  static int dataBlockBits() { return g_DATA_BLOCK_BITS; }
  static int dataBlockBytes() { return g_DATA_BLOCK_BYTES; }

  // SUPPORTED PHYSICAL MEMORY CONFIG VARIABLES
  static int pageSizeBits() { return g_PAGE_SIZE_BITS; }
  static int pageSizeBytes() { return g_PAGE_SIZE_BYTES; }
  static int memorySizeBits() { return g_MEMORY_SIZE_BITS; }
  static int64 memorySizeBytes() { return g_MEMORY_SIZE_BYTES; }
  static int memoryModuleBits() { return g_MEMORY_MODULE_BITS; }
  static int64 memoryModuleBlocks() { return g_MEMORY_MODULE_BLOCKS; }

  // returns number of SMT threads per physical processor
  static int numberofSMTThreads() { return g_NUM_SMT_THREADS; }
  // defines the number of simics processors (power of 2)
  static int numberOfProcessors() { return g_NUM_PROCESSORS; }
  static int procsPerChipBits() { return g_PROCS_PER_CHIP_BITS; }
  static int numberOfProcsPerChip() { return g_PROCS_PER_CHIP; }
  static int numberOfChips() { return g_NUM_CHIPS; }

  // MACHINE INSTANIATION CONFIG VARIABLES
  // -------------------------------------
  // L1 CACHE MACHINES
  // defines the number of L1banks - idependent of ruby chips (power of 2)
  // NOTE - no protocols currently supports L1s != processors, just a placeholder
  static int L1CacheBits() { return g_NUM_PROCESSORS_BITS; }
  static int numberOfL1Cache() { return g_NUM_PROCESSORS; }
  static int L1CachePerChipBits() { return procsPerChipBits() ; } // L1s != processors not currently supported
  static int numberOfL1CachePerChip() { return numberOfProcsPerChip(); } // L1s != processors not currently supported
  static int numberOfL1CachePerChip(NodeID myNodeID) { return numberOfL1CachePerChip(); }
  static int L1CacheTransitionsPerCycle() { return L1CACHE_TRANSITIONS_PER_RUBY_CYCLE; }

  // L2 CACHE MACHINES
  // defines the number of L2banks/L2Caches - idependent of ruby chips (power of 2)
  static int L2CacheBits() { return g_NUM_L2_BANKS_BITS; }
  static int numberOfL2Cache() { return g_NUM_L2_BANKS; }
  static int L1CacheNumToL2Base(NodeID L1RubyNodeID);
  static int L2CachePerChipBits() { return g_NUM_L2_BANKS_PER_CHIP_BITS; }
  static int numberOfL2CachePerChip() { return g_NUM_L2_BANKS_PER_CHIP; }
  static int numberOfL2CachePerChip(NodeID myNodeID) { return numberOfL2CachePerChip(); }
  static int L2CacheTransitionsPerCycle() { return L2CACHE_TRANSITIONS_PER_RUBY_CYCLE; }

  // DIRECTORY/MEMORY MACHINES
  // defines the number of ruby memories - idependent of ruby chips (power of 2)
  static int memoryBits() { return g_NUM_MEMORIES_BITS; }
  static int numberOfDirectory() { return numberOfMemories(); }
  static int numberOfMemories() { return g_NUM_MEMORIES; }
  static int numberOfDirectoryPerChip() { return g_NUM_MEMORIES_PER_CHIP; }
  static int numberOfDirectoryPerChip(NodeID myNodeID) { return g_NUM_MEMORIES_PER_CHIP; }
  static int DirectoryTransitionsPerCycle() { return DIRECTORY_TRANSITIONS_PER_RUBY_CYCLE; }

  // PERSISTENT ARBITER MACHINES
  static int numberOfPersistentArbiter() { return numberOfMemories(); }
  static int numberOfPersistentArbiterPerChip() {return numberOfDirectoryPerChip(); }
  static int numberOfPersistentArbiterPerChip(NodeID myNodeID) {return numberOfDirectoryPerChip(myNodeID); }
  static int PersistentArbiterTransitionsPerCycle() { return L2CACHE_TRANSITIONS_PER_RUBY_CYCLE; }

  // ---- END MACHINE SPECIFIC VARIABLES ----

  // VARIABLE MEMORY RESPONSE LATENCY
  // *** NOTE *** This is where variation is added to the simulation
  // see Alameldeen et al. HPCA 2003 for further details
  static int memoryResponseLatency() { return MEMORY_RESPONSE_LATENCY_MINUS_2+(random() % 5); }

  static void init();
  static void printConfiguration(ostream& out);

  // Memory Controller
  static int memBusCycleMultiplier () { return MEM_BUS_CYCLE_MULTIPLIER; }
  static int banksPerRank () { return BANKS_PER_RANK; }
  static int ranksPerDimm () { return RANKS_PER_DIMM; }
  static int dimmsPerChannel () { return DIMMS_PER_CHANNEL; }
  static int bankBit0 () { return BANK_BIT_0; }
  static int rankBit0 () { return RANK_BIT_0; }
  static int dimmBit0 () { return DIMM_BIT_0; }
  static int bankQueueSize () { return BANK_QUEUE_SIZE; }
  static int bankBusyTime () { return BANK_BUSY_TIME; }
  static int rankRankDelay () { return RANK_RANK_DELAY; }
  static int readWriteDelay () { return READ_WRITE_DELAY; }
  static int basicBusBusyTime () { return BASIC_BUS_BUSY_TIME; }
  static int memCtlLatency () { return MEM_CTL_LATENCY; }
  static int refreshPeriod () { return REFRESH_PERIOD; }
  static int tFaw () { return TFAW; }
  static int memRandomArbitrate () { return MEM_RANDOM_ARBITRATE; }
  static int memFixedDelay () { return MEM_FIXED_DELAY; }

private:
};

#endif //RUBYCONFIG_H
