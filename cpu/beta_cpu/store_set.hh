/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_BETA_CPU_STORE_SET_HH__
#define __CPU_BETA_CPU_STORE_SET_HH__

#include <vector>

#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"

class StoreSet
{
  public:
    typedef unsigned SSID;

  public:
    StoreSet(int SSIT_size, int LFST_size);

    void violation(Addr store_PC, Addr load_PC);

    void insertLoad(Addr load_PC, InstSeqNum load_seq_num);

    void insertStore(Addr store_PC, InstSeqNum store_seq_num);

    InstSeqNum checkInst(Addr PC);

    void issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store);

    void squash(InstSeqNum squashed_num);

    void clear();

  private:
    inline int calcIndex(Addr PC)
    { return (PC >> offset_bits) & index_mask; }

    inline SSID calcSSID(Addr PC)
    { return ((PC ^ (PC >> 10)) % LFST_size); }

    SSID *SSIT;

    std::vector<bool> validSSIT;

    InstSeqNum *LFST;

    std::vector<bool> validLFST;

    int *SSCounters;

    int SSIT_size;

    int LFST_size;

    int index_mask;

    // HACK: Hardcoded for now.
    int offset_bits;
};

#endif // __CPU_BETA_CPU_STORE_SET_HH__
