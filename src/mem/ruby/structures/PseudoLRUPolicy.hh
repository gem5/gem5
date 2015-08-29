/*
 * Copyright (c) 2007 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc
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

#ifndef __MEM_RUBY_STRUCTURES_PSEUDOLRUPOLICY_HH__
#define __MEM_RUBY_STRUCTURES_PSEUDOLRUPOLICY_HH__

#include "mem/ruby/structures/AbstractReplacementPolicy.hh"
#include "params/PseudoLRUReplacementPolicy.hh"

/**
 * Implementation of tree-based pseudo-LRU replacement
 *
 * Works for any associativity between 1 and 128.
 *
 * Also implements associativities that are not a power of 2 by
 * ignoring paths that lead to a larger index (i.e. truncating the
 * tree).  Note that when this occurs, the algorithm becomes less
 * fair, as it will favor indicies in the larger (by index) half of
 * the associative set. This is most unfair when the nearest power of
 * 2 is one below the associativy, and most fair when it is one above.
 */

class PseudoLRUPolicy : public AbstractReplacementPolicy
{
  public:
    typedef PseudoLRUReplacementPolicyParams Params;
    PseudoLRUPolicy(const Params * p);
    ~PseudoLRUPolicy();

    void touch(int64_t set, int64_t way, Tick time);
    int64_t getVictim(int64_t set) const;

  private:
    unsigned int m_effective_assoc;    /** nearest (to ceiling) power of 2 */
    unsigned int m_num_levels;         /** number of levels in the tree */
    uint64_t* m_trees;                   /** bit representation of the
                                        * trees, one for each set */
};

#endif // __MEM_RUBY_STRUCTURES_PSEUDOLRUPOLICY_HH__
