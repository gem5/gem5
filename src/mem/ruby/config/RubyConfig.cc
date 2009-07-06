
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
 * RubyConfig.cc
 *
 * Description: See RubyConfig.hh
 *
 * $Id$
 *
 */

#include "mem/ruby/config/RubyConfig.hh"
//#include "mem/protocol/protocol_name.hh"
#include "mem/gems_common/util.hh"

#define CONFIG_DEF_FILE "mem/ruby/config/config.hh"

#define ERROR_MSG(MESSAGE)\
{\
    cerr << "Fatal Error: in fn "\
         << __PRETTY_FUNCTION__ << " in "\
         << __FILE__ << ":"\
         << __LINE__ << ": "\
         << (MESSAGE) << endl << flush;\
    abort();\
}

// declare all configuration variables
#define PARAM_BOOL( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  bool RubyConfig::m_##NAME = DEFAULT_VALUE;
#define PARAM_STRING( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  const char* RubyConfig::m_##NAME = DEFAULT_VALUE;
#define PARAM_ULONG( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  uint64 RubyConfig::m_##NAME = DEFAULT_VALUE;
#define PARAM( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  int RubyConfig::m_##NAME = DEFAULT_VALUE;
#define PARAM_ARRAY( NAME, TYPE, DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  TYPE* RubyConfig::m_##NAME = NULL;
#define PARAM_ARRAY2D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  TYPE** RubyConfig::m_##NAME = NULL;
#define PARAM_ARRAY3D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, D3_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  TYPE*** RubyConfig::m_##NAME = NULL;
#include CONFIG_DEF_FILE
#undef PARAM_BOOL
#undef PARAM_STRING
#undef PARAM_ULONG
#undef PARAM
#undef PARAM_ARRAY
#undef PARAM_ARRAY2D
#undef PARAM_ARRAY3D

#define CHECK_POWER_OF_2(N) { if (!is_power_of_2(N)) { ERROR_MSG(#N " must be a power of 2."); }}
#define CHECK_ZERO(N) { if (N != 0) { ERROR_MSG(#N " must be zero at initialization."); }}
#define CHECK_NON_ZERO(N) { if (N == 0) { ERROR_MSG(#N " must be non-zero."); }}

uint32 RubyConfig::m_data_block_mask;

void RubyConfig::reset()
{
 #define PARAM_BOOL( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = DEFAULT_VALUE;
#define PARAM_STRING( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = DEFAULT_VALUE;
#define PARAM_ULONG( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = DEFAULT_VALUE;
#define PARAM( NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = DEFAULT_VALUE;
#define PARAM_ARRAY( NAME, TYPE, DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = new TYPE[DEFAULT_ARRAY_SIZE]; \
  for (int i=0; i<DEFAULT_ARRAY_SIZE; i++) \
    m_##NAME[i] = DEFAULT_VALUE;
#define PARAM_ARRAY2D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = new TYPE*[D1_DEFAULT_ARRAY_SIZE]; \
  for (int i=0; i<D1_DEFAULT_ARRAY_SIZE; i++) {       \
    m_##NAME[i] = new TYPE[D2_DEFAULT_ARRAY_SIZE]; \
    for (int j=0; j<D2_DEFAULT_ARRAY_SIZE; j++) \
      m_##NAME[i][j] = DEFAULT_VALUE; \
  }
#define PARAM_ARRAY3D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, D3_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) \
  m_##NAME = new TYPE**[D1_DEFAULT_ARRAY_SIZE];                         \
  for (int i=0; i<D1_DEFAULT_ARRAY_SIZE; i++) {                         \
    m_##NAME[i] = new TYPE*[D2_DEFAULT_ARRAY_SIZE];                     \
    for (int j=0; j<D2_DEFAULT_ARRAY_SIZE; j++) {                       \
      m_##NAME[i][j] = new TYPE[D3_DEFAULT_ARRAY_SIZE];                 \
      for (int k=0; k<D3_DEFAULT_ARRAY_SIZE; k++)                       \
        m_##NAME[i][j][k] = DEFUALT_VALUE;                              \
    }                                                                   \
  }
#include CONFIG_DEF_FILE
#undef PARAM_BOOL
#undef PARAM_STRING
#undef PARAM_ULONG
#undef PARAM
#undef PARAM_ARRAY
#undef PARAM_ARRAY2D
#undef PARAM_ARRAY3D
}

void RubyConfig::init()
{
  /*
  // MemoryControl:
  CHECK_NON_ZERO(m_MEM_BUS_CYCLE_MULTIPLIER);
  CHECK_NON_ZERO(m_BANKS_PER_RANK);
  CHECK_NON_ZERO(m_RANKS_PER_DIMM);
  CHECK_NON_ZERO(m_DIMMS_PER_CHANNEL);
  CHECK_NON_ZERO(m_BANK_QUEUE_SIZE);
  CHECK_NON_ZERO(m_BankBusyTime);
  CHECK_NON_ZERO(m_MEM_CTL_LATENCY);
  CHECK_NON_ZERO(m_REFRESH_PERIOD);
  CHECK_NON_ZERO(m_BASIC_BUS_BUSY_TIME);

  CHECK_POWER_OF_2(m_BANKS_PER_RANK);
  CHECK_POWER_OF_2(m_RANKS_PER_DIMM);
  CHECK_POWER_OF_2(m_DIMMS_PER_CHANNEL);

  CHECK_NON_ZERO(m_MemorySizeBytes);
  //  CHECK_NON_ZERO(m_DATA_BLOCK_BYTES);
  CHECK_NON_ZERO(m_NUM_PROCESSORS);
  CHECK_NON_ZERO(m_ProcsPerChip);

  if (m_NUM_L2_BANKS == 0) {  // defaults to number of ruby nodes
    m_NUM_L2_BANKS = m_NUM_PROCESSORS;
  }
  if (m_NUM_MEMORIES == 0) {  // defaults to number of ruby nodes
    m_NUM_MEMORIES = m_NUM_PROCESSORS;
  }

  CHECK_ZERO(m_MEMORY_SIZE_BITS);
  CHECK_ZERO(m_NUM_PROCESSORS_BITS);
  CHECK_ZERO(m_NUM_CHIP_BITS);
  CHECK_ZERO(m_NUM_L2_BANKS_BITS);
  CHECK_ZERO(m_NUM_MEMORIES_BITS);
  CHECK_ZERO(m_PROCS_PER_CHIP_BITS);
  CHECK_ZERO(m_NUM_L2_BANKS_PER_CHIP);
  CHECK_ZERO(m_NUM_L2_BANKS_PER_CHIP_BITS);
  CHECK_ZERO(m_NUM_MEMORIES_BITS);
  CHECK_ZERO(m_MEMORY_MODULE_BLOCKS);
  CHECK_ZERO(m_MEMORY_MODULE_BITS);
  CHECK_ZERO(m_NUM_MEMORIES_PER_CHIP);

  CHECK_POWER_OF_2(m_MemorySizeBytes);
  CHECK_POWER_OF_2(m_NUM_PROCESSORS);
  CHECK_POWER_OF_2(m_NUM_L2_BANKS);
  CHECK_POWER_OF_2(m_NUM_MEMORIES);
  CHECK_POWER_OF_2(m_ProcsPerChip);

  assert(m_NUM_PROCESSORS >= m_ProcsPerChip);  // obviously can't have less processors than procs/chip
  m_NUM_CHIPS = m_NUM_PROCESSORS/m_ProcsPerChip;
  assert(m_NUM_L2_BANKS >= m_NUM_CHIPS);  // cannot have a single L2cache across multiple chips

  m_NUM_L2_BANKS_PER_CHIP = m_NUM_L2_BANKS/m_NUM_CHIPS;

  if (m_NUM_CHIPS > m_NUM_MEMORIES) {
    m_NUM_MEMORIES_PER_CHIP = 1;  // some chips have a memory, others don't
  } else {
    m_NUM_MEMORIES_PER_CHIP = m_NUM_MEMORIES/m_NUM_CHIPS;
  }

  m_NUM_CHIP_BITS = log_int(m_NUM_CHIPS);
  m_MEMORY_SIZE_BITS = log_int(m_MemorySizeBytes);

  m_data_block_mask = ~ (~0 << m_DATA_BLOCK_BITS);

  m_NUM_PROCESSORS_BITS = log_int(m_NUM_PROCESSORS);
  m_NUM_L2_BANKS_BITS = log_int(m_NUM_L2_BANKS);
  m_NUM_L2_BANKS_PER_CHIP_BITS = log_int(m_NUM_L2_BANKS_PER_CHIP);
  m_NUM_MEMORIES_BITS = log_int(m_NUM_MEMORIES);
  m_PROCS_PER_CHIP_BITS = log_int(m_ProcsPerChip);

  m_MEMORY_MODULE_BITS = m_MEMORY_SIZE_BITS - m_DATA_BLOCK_BITS - m_NUM_MEMORIES_BITS;
  m_MEMORY_MODULE_BLOCKS = (int64(1) << m_MEMORY_MODULE_BITS);

  */

  // Randomize the execution
  //  srandom(m_RandomSeed);
}

static void print_parameters(ostream& out)
{

#define print_true(NAME)
#define print_false(NAME)                                         \
  out << #NAME << ": " << RubyConfig::get##NAME () << endl

#define PARAM(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_UINT(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_ULONG(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_BOOL(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_DOUBLE(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_STRING(NAME, DEFAULT_VALUE, CUSTOM_ACCESSOR) { print_##CUSTOM_ACCESSOR(NAME); }
#define PARAM_ARRAY( NAME, TYPE, DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) { out << #NAME << ": ARRAY" << endl; }
#define PARAM_ARRAY2D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) { out << #NAME << ": ARRAY2D" << endl; }
#define PARAM_ARRAY3D( NAME, TYPE, D1_DEFAULT_ARRAY_SIZE, D2_DEFAULT_ARRAY_SIZE, D3_DEFAULT_ARRAY_SIZE, DEFAULT_VALUE, CUSTOM_ACCESSOR ) { out << #NAME << ": ARRAY3D" << endl; }
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
}

void RubyConfig::printConfiguration(ostream& out) {
  out << "Ruby Configuration" << endl;
  out << "------------------" << endl;

  //out << "protocol: " << CURRENT_PROTOCOL << endl;
  out << "compiled_at: " << __TIME__ << ", " << __DATE__ << endl;
  //  out << "RUBY_DEBUG: " << bool_to_string(RUBY_DEBUG) << endl;

  char buffer[100];
  gethostname(buffer, 50);
  out << "hostname: " << buffer << endl;

  print_parameters(out);
}


