
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
 * RubyConfig.C
 *
 * Description: See RubyConfig.h
 *
 * $Id$
 *
 */

#include "mem/ruby/config/RubyConfig.hh"
#include "mem/protocol/protocol_name.hh"
#include "mem/gems_common/util.hh"
#include "mem/protocol/Protocol.hh"

#define CHECK_POWER_OF_2(N) { if (!is_power_of_2(N)) { ERROR_MSG(#N " must be a power of 2."); }}
#define CHECK_ZERO(N) { if (N != 0) { ERROR_MSG(#N " must be zero at initialization."); }}
#define CHECK_NON_ZERO(N) { if (N == 0) { ERROR_MSG(#N " must be non-zero."); }}


void RubyConfig::init()
{
  // MemoryControl:
  CHECK_NON_ZERO(MEM_BUS_CYCLE_MULTIPLIER);
  CHECK_NON_ZERO(BANKS_PER_RANK);
  CHECK_NON_ZERO(RANKS_PER_DIMM);
  CHECK_NON_ZERO(DIMMS_PER_CHANNEL);
  CHECK_NON_ZERO(BANK_QUEUE_SIZE);
  CHECK_NON_ZERO(BANK_BUSY_TIME);
  CHECK_NON_ZERO(MEM_CTL_LATENCY);
  CHECK_NON_ZERO(REFRESH_PERIOD);
  CHECK_NON_ZERO(BASIC_BUS_BUSY_TIME);

  CHECK_POWER_OF_2(BANKS_PER_RANK);
  CHECK_POWER_OF_2(RANKS_PER_DIMM);
  CHECK_POWER_OF_2(DIMMS_PER_CHANNEL);

  CHECK_NON_ZERO(g_MEMORY_SIZE_BYTES);
  CHECK_NON_ZERO(g_DATA_BLOCK_BYTES);
  CHECK_NON_ZERO(g_PAGE_SIZE_BYTES);
  CHECK_NON_ZERO(g_NUM_PROCESSORS);
  CHECK_NON_ZERO(g_PROCS_PER_CHIP);
  if(g_NUM_SMT_THREADS == 0){ //defaults to single-threaded
    g_NUM_SMT_THREADS = 1;
  }
  if (g_NUM_L2_BANKS == 0) {  // defaults to number of ruby nodes
    g_NUM_L2_BANKS = g_NUM_PROCESSORS;
  }
  if (g_NUM_MEMORIES == 0) {  // defaults to number of ruby nodes
    g_NUM_MEMORIES = g_NUM_PROCESSORS;
  }

  CHECK_ZERO(g_MEMORY_SIZE_BITS);
  CHECK_ZERO(g_DATA_BLOCK_BITS);
  CHECK_ZERO(g_PAGE_SIZE_BITS);
  CHECK_ZERO(g_NUM_PROCESSORS_BITS);
  CHECK_ZERO(g_NUM_CHIP_BITS);
  CHECK_ZERO(g_NUM_L2_BANKS_BITS);
  CHECK_ZERO(g_NUM_MEMORIES_BITS);
  CHECK_ZERO(g_PROCS_PER_CHIP_BITS);
  CHECK_ZERO(g_NUM_L2_BANKS_PER_CHIP);
  CHECK_ZERO(g_NUM_L2_BANKS_PER_CHIP_BITS);
  CHECK_ZERO(g_NUM_MEMORIES_BITS);
  CHECK_ZERO(g_MEMORY_MODULE_BLOCKS);
  CHECK_ZERO(g_MEMORY_MODULE_BITS);
  CHECK_ZERO(g_NUM_MEMORIES_PER_CHIP);

  CHECK_POWER_OF_2(g_MEMORY_SIZE_BYTES);
  CHECK_POWER_OF_2(g_DATA_BLOCK_BYTES);
  CHECK_POWER_OF_2(g_NUM_PROCESSORS);
  CHECK_POWER_OF_2(g_NUM_L2_BANKS);
  CHECK_POWER_OF_2(g_NUM_MEMORIES);
  CHECK_POWER_OF_2(g_PROCS_PER_CHIP);

  ASSERT(g_NUM_PROCESSORS >= g_PROCS_PER_CHIP);  // obviously can't have less processors than procs/chip
  g_NUM_CHIPS = g_NUM_PROCESSORS/g_PROCS_PER_CHIP;
  ASSERT(g_NUM_L2_BANKS >= g_NUM_CHIPS);  // cannot have a single L2cache across multiple chips

  g_NUM_L2_BANKS_PER_CHIP = g_NUM_L2_BANKS/g_NUM_CHIPS;

  ASSERT(L2_CACHE_NUM_SETS_BITS > log_int(g_NUM_L2_BANKS_PER_CHIP));  // cannot have less than one set per bank
  L2_CACHE_NUM_SETS_BITS = L2_CACHE_NUM_SETS_BITS - log_int(g_NUM_L2_BANKS_PER_CHIP);

  if (g_NUM_CHIPS > g_NUM_MEMORIES) {
    g_NUM_MEMORIES_PER_CHIP = 1;  // some chips have a memory, others don't
  } else {
    g_NUM_MEMORIES_PER_CHIP = g_NUM_MEMORIES/g_NUM_CHIPS;
  }

  g_NUM_CHIP_BITS = log_int(g_NUM_CHIPS);
  g_MEMORY_SIZE_BITS = log_int(g_MEMORY_SIZE_BYTES);
  g_DATA_BLOCK_BITS = log_int(g_DATA_BLOCK_BYTES);
  g_PAGE_SIZE_BITS = log_int(g_PAGE_SIZE_BYTES);
  g_NUM_PROCESSORS_BITS = log_int(g_NUM_PROCESSORS);
  g_NUM_L2_BANKS_BITS = log_int(g_NUM_L2_BANKS);
  g_NUM_L2_BANKS_PER_CHIP_BITS = log_int(g_NUM_L2_BANKS_PER_CHIP);
  g_NUM_MEMORIES_BITS = log_int(g_NUM_MEMORIES);
  g_PROCS_PER_CHIP_BITS = log_int(g_PROCS_PER_CHIP);

  g_MEMORY_MODULE_BITS = g_MEMORY_SIZE_BITS - g_DATA_BLOCK_BITS - g_NUM_MEMORIES_BITS;
  g_MEMORY_MODULE_BLOCKS = (int64(1) << g_MEMORY_MODULE_BITS);

  if ((!Protocol::m_CMP) && (g_PROCS_PER_CHIP > 1)) {
    ERROR_MSG("Non-CMP protocol should set g_PROCS_PER_CHIP to 1");
  }

  // Randomize the execution
  srandom(g_RANDOM_SEED);
}

int RubyConfig::L1CacheNumToL2Base(NodeID L1CacheNum)
{
  return L1CacheNum/g_PROCS_PER_CHIP;
}

static void print_parameters(ostream& out)
{

#define PARAM(NAME) { out << #NAME << ": " << NAME << endl; }
#define PARAM_UINT(NAME) { out << #NAME << ": " << NAME << endl; }
#define PARAM_ULONG(NAME) { out << #NAME << ": " << NAME << endl; }
#define PARAM_BOOL(NAME) { out << #NAME << ": " << bool_to_string(NAME) << endl; }
#define PARAM_DOUBLE(NAME) { out << #NAME << ": " << NAME << endl; }
#define PARAM_STRING(NAME) { assert(NAME != NULL); out << #NAME << ": " << string(NAME) << endl; }
#define PARAM_ARRAY(PTYPE, NAME, ARRAY_SIZE)   \
  {                                            \
    out << #NAME << ": (";                     \
    for (int i = 0; i < ARRAY_SIZE; i++) {     \
      if (i != 0) {                            \
        out << ", ";                           \
      }                                        \
      out << NAME[i];                          \
    }                                          \
    out << ")" << endl;                        \
  }                                            \


#include "mem/ruby/config/config.hh"
#undef PARAM
#undef PARAM_UINT
#undef PARAM_ULONG
#undef PARAM_BOOL
#undef PARAM_DOUBLE
#undef PARAM_STRING
#undef PARAM_ARRAY
}

void RubyConfig::printConfiguration(ostream& out) {
  out << "Ruby Configuration" << endl;
  out << "------------------" << endl;

  out << "protocol: " << CURRENT_PROTOCOL << endl;
  out << "compiled_at: " << __TIME__ << ", " << __DATE__ << endl;
  out << "RUBY_DEBUG: " << bool_to_string(RUBY_DEBUG) << endl;

  char buffer[100];
  gethostname(buffer, 50);
  out << "hostname: " << buffer << endl;

  print_parameters(out);
}


