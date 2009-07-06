
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
 * RubyConfig.hh
 *
 * Description: This class has only static members and class methods,
 * and thus should never need to be instantiated.
 *
 * $Id$
 *
 */

#ifndef RUBYCONFIG_H
#define RUBYCONFIG_H

#include <cstdlib>
#include <string>
#include <ostream>
#include <assert.h>

#include "mem/ruby/common/TypeDefines.hh"

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

using namespace std;

class RubyConfig {
public:

  // CACHE BLOCK CONFIG VARIBLES
  static uint32 dataBlockMask() { return m_data_block_mask; }

  static int numberOfDMA() { return 1; }
  static int numberOfDMAPerChip() { return 1; }
  static int DMATransitionsPerCycle() { return 1; }

  // SUPPORTED PHYSICAL MEMORY CONFIG VARIABLES
  //  static int memoryModuleBits() { return m_MEMORY_MODULE_BITS; }
  //  static int64 memoryModuleBlocks() { return m_MEMORY_MODULE_BLOCKS; }

  // defines the number of simics processors (power of 2)
  //  static int numberOfProcessors() { return m_NUM_PROCESSORS; }
  //  static int procsPerChipBits() { return m_PROCS_PER_CHIP_BITS; }
  //  static int numberOfProcsPerChip() { return m_ProcsPerChip; }
  //  static int numberOfChips() { return m_NUM_CHIPS; }

  // MACHINE INSTANIATION CONFIG VARIABLES
  // -------------------------------------
  // L1 CACHE MACHINES
  // defines the number of L1banks - idependent of ruby chips (power of 2)
  // NOTE - no protocols currently supports L1s != processors, just a placeholder

  // DIRECTORY/MEMORY MACHINES
  // defines the number of ruby memories - idependent of ruby chips (power of 2)
  //  static int memoryBits() { return m_NUM_MEMORIES_BITS; }
  //  static int numberOfDirectory() { return numberOfMemories(); }
  //  static int numberOfMemories() { return m_NUM_MEMORIES; }
  //  static int numberOfDirectoryPerChip() { return m_NUM_MEMORIES_PER_CHIP; }
  //  static int DirectoryTransitionsPerCycle() { return m_DIRECTORY_TRANSITIONS_PER_RUBY_CYCLE; }

  // ---- END MACHINE SPECIFIC VARIABLES ----

  // VARIABLE MEMORY RESPONSE LATENCY
  // *** NOTE *** This is where variation is added to the simulation
  // see Alameldeen et al. HPCA 2003 for further details
//  static int getMemoryLatency() { return m_MEMORY_RESPONSE_LATENCY_MINUS_2+(random() % 5); }

  static void reset();
  static void init();
  static void printConfiguration(std::ostream& out);

  // Memory Controller

//  static int memBusCycleMultiplier () { return m_MEM_BUS_CYCLE_MULTIPLIER; }
/*  static int banksPerRank () { return m_BANKS_PER_RANK; }
  static int ranksPerDimm () { return m_RANKS_PER_DIMM; }
  static int dimmsPerChannel () { return m_DIMMS_PER_CHANNEL; }
  static int bankBit0 () { return m_BANK_BIT_0; }
  static int rankBit0 () { return m_RANK_BIT_0; }
  static int dimmBit0 () { return m_DIMM_BIT_0; }
  static int bankQueueSize () { return m_BANK_QUEUE_SIZE; }
  static int bankBusyTime () { return m_BankBusyTime; }
  static int rankRankDelay () { return m_RANK_RANK_DELAY; }
  static int readWriteDelay () { return m_READ_WRITE_DELAY; }
  static int basicBusBusyTime () { return m_BASIC_BUS_BUSY_TIME; }
  static int memCtlLatency () { return m_MEM_CTL_LATENCY; }
  static int refreshPeriod () { return m_REFRESH_PERIOD; }
  static int tFaw () { return m_TFAW; }
  static int memRandomArbitrate () { return m_MEM_RANDOM_ARBITRATE; }
  static int memFixedDelay () { return m_MEM_FIXED_DELAY; }
*/
  /* cache accessors */
  static int getCacheIDFromParams(int level, int num, string split_type) {
    // TODO:  this function
    return 0;
  }

#define accessor_true( TYPE, NAME )
#define accessor_false( TYPE, NAME )                    \
  static TYPE get##NAME() { return m_##NAME; }          \
  static void set##NAME(TYPE val) { m_##NAME = val; }

#define array_accessor_true( TYPE, NAME, DEFAULT_ARRAY_SIZE )
#define array_accessor_false( TYPE, NAME, DEFAULT_ARRAY_SIZE )          \
  static TYPE get##NAME(int idx) {                                      \
    assert(m_##NAME != NULL);                                           \
    return m_##NAME[idx];                                               \
  }                                                                     \
  static void set##NAME(int idx, TYPE val) {                            \
    if(m_##NAME == NULL) {                                              \
      assert(DEFAULT_ARRAY_SIZE > 0);                                   \
      m_##NAME = new TYPE[DEFAULT_ARRAY_SIZE];                          \
    }                                                                   \
    m_##NAME[idx] = val;                                                \
  }

#define array2d_accessor_true( TYPE, NAME )
#define array2d_accessor_false( TYPE, NAME )                            \
  static TYPE get##NAME(int idx1, int idx2) { return m_##NAME[idx1][idx2]; } \
  static void set##NAME(int idx1, int idx2, TYPE val) { m_##NAME[idx1][idx2] = val; }

#define array3d_accessor_true( TYPE, NAME )
#define array3d_accessor_false( TYPE, NAME )                            \
  static TYPE get##NAME(int idx1, int idx2, int idx3) { return m_##NAME[idx1][idx2][idx3]; } \
  static void set##NAME(int idx1, int idx2, int idx3, TYPE val) { m_##NAME[idx1][idx2][idx3] = val; }

#define PARAM( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )   \
  accessor_##CUSTOM_ACCESSOR(int32,NAME)
#define PARAM_UINT( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )      \
  accessor_##CUSTOM_ACCESSOR(uint32,NAME)
#define PARAM_ULONG( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )     \
  accessor_##CUSTOM_ACCESSOR(uint64,NAME)
#define PARAM_BOOL( NAME, DEFAULT_VALUE,CUSTOM_ACCESSOR )       \
  accessor_##CUSTOM_ACCESSOR(bool,NAME)
#define PARAM_DOUBLE( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )    \
  accessor_##CUSTOM_ACCESSOR(double,NAME)
#define PARAM_STRING( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )    \
  accessor_##CUSTOM_ACCESSOR(const char*,NAME)
#define PARAM_ARRAY( NAME, TYPE, DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  array_accessor_##CUSTOM_ACCESSOR(TYPE, NAME, DEFAULT_ARRAY_SIZE)
#define PARAM_ARRAY2D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  array2d_accessor_##CUSTOM_ACCESSOR(TYPE, NAME)
#define PARAM_ARRAY3D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, D3_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  array3d_accessor_##CUSTOM_ACCESSOR(TYPE, NAME)
#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY
#undef PARAM_ARRAY2D
#undef PARAM_ARRAY3D

private:
  static uint32 m_data_block_mask;

#define PARAM( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )   \
  static int32  m_##NAME;
#define PARAM_UINT( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )      \
  static uint32 m_##NAME;
#define PARAM_ULONG( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )     \
  static uint64 m_##NAME;
#define PARAM_BOOL( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )      \
  static bool   m_##NAME;
#define PARAM_DOUBLE( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )    \
  static double m_##NAME;
#define PARAM_STRING( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR )    \
  static const char  *m_##NAME;
#define PARAM_ARRAY( NAME, TYPE, DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  static TYPE* m_##NAME;
#define PARAM_ARRAY2D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  static TYPE** m_##NAME;
#define PARAM_ARRAY3D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, D3_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  static TYPE*** m_##NAME;
#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY
#undef PARAM_ARRAY2D
#undef PARAM_ARRAY3D

};

#endif //RUBYCONFIG_H
