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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_STORE_SET_HH__
#define __CPU_O3_STORE_SET_HH__

#include <list>
#include <map>
#include <utility>
#include <vector>

#include "arch/isa_traits.hh"
#include "cpu/inst_seq.hh"

struct ltseqnum {
    bool operator()(const InstSeqNum &lhs, const InstSeqNum &rhs) const
    {
        return lhs > rhs;
    }
};

class StoreSet
{
  public:
    typedef unsigned SSID;

  public:
    StoreSet() { };

    StoreSet(int SSIT_size, int LFST_size);

    ~StoreSet();

    void init(int SSIT_size, int LFST_size);

    void violation(Addr store_PC, Addr load_PC);

    void insertLoad(Addr load_PC, InstSeqNum load_seq_num);

    void insertStore(Addr store_PC, InstSeqNum store_seq_num,
                     unsigned tid);

    InstSeqNum checkInst(Addr PC);

    void issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store);

    void squash(InstSeqNum squashed_num, unsigned tid);

    void clear();

  private:
    inline int calcIndex(Addr PC)
    { return (PC >> offsetBits) & indexMask; }

    inline SSID calcSSID(Addr PC)
    { return ((PC ^ (PC >> 10)) % LFSTSize); }

    std::vector<SSID> SSIT;

    std::vector<bool> validSSIT;

    std::vector<InstSeqNum> LFST;

    std::vector<bool> validLFST;

    std::map<InstSeqNum, int, ltseqnum> storeList;

    typedef std::map<InstSeqNum, int, ltseqnum>::iterator SeqNumMapIt;

    int SSITSize;

    int LFSTSize;

    int indexMask;

    // HACK: Hardcoded for now.
    int offsetBits;
};

#endif // __CPU_O3_STORE_SET_HH__
