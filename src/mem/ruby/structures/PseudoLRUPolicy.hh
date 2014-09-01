/*
 * Copyright (c) 2007 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SYSTEM_PSEUDOLRUPOLICY_HH__
#define __MEM_RUBY_SYSTEM_PSEUDOLRUPOLICY_HH__

#include "mem/ruby/structures/AbstractReplacementPolicy.hh"

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
    PseudoLRUPolicy(Index num_sets, Index assoc);
    ~PseudoLRUPolicy();

    void touch(Index set, Index way, Tick time);
    Index getVictim(Index set) const;

  private:
    unsigned int m_effective_assoc;    /** nearest (to ceiling) power of 2 */
    unsigned int m_num_levels;         /** number of levels in the tree */
    uint64* m_trees;                   /** bit representation of the
                                        * trees, one for each set */
};

inline
PseudoLRUPolicy::PseudoLRUPolicy(Index num_sets, Index assoc)
    : AbstractReplacementPolicy(num_sets, assoc)
{
    // associativity cannot exceed capacity of tree representation
    assert(num_sets > 0 && assoc > 1 && assoc <= (Index) sizeof(uint64)*4);

    m_trees = NULL;
    m_num_levels = 0;

    m_effective_assoc = 1;
    while (m_effective_assoc < assoc) {
        // effective associativity is ceiling power of 2
        m_effective_assoc <<= 1;
    }
    assoc = m_effective_assoc;
    while (true) {
        assoc /= 2;
        if(!assoc) break;
        m_num_levels++;
    }
    assert(m_num_levels < sizeof(unsigned int)*4);
    m_trees = new uint64[m_num_sets];
    for (unsigned i = 0; i < m_num_sets; i++) {
        m_trees[i] = 0;
    }
}

inline
PseudoLRUPolicy::~PseudoLRUPolicy()
{
    if (m_trees != NULL)
        delete[] m_trees;
}

inline void
PseudoLRUPolicy::touch(Index set, Index index, Tick time)
{
    assert(index >= 0 && index < m_assoc);
    assert(set >= 0 && set < m_num_sets);

    int tree_index = 0;
    int node_val;
    for (int i = m_num_levels - 1; i >= 0; i--) {
        node_val = (index >> i)&1;
        if (node_val)
            m_trees[set] |= node_val << tree_index;
        else
            m_trees[set] &= ~(1 << tree_index);
        tree_index = node_val ? (tree_index*2)+2 : (tree_index*2)+1;
    }
    m_last_ref_ptr[set][index] = time;
}

inline Index
PseudoLRUPolicy::getVictim(Index set) const
{
    // assert(m_assoc != 0);
    Index index = 0;

    int tree_index = 0;
    int node_val;
    for (unsigned i = 0; i < m_num_levels; i++){
        node_val = (m_trees[set] >> tree_index) & 1;
        index += node_val ? 0 : (m_effective_assoc >> (i + 1));
        tree_index = node_val ? (tree_index * 2) + 1 : (tree_index * 2) + 2;
    }
    assert(index >= 0 && index < m_effective_assoc);

    /* return either the found index or the max possible index */
    /* NOTE: this is not a fair replacement when assoc is not a power of 2 */
    return (index > (m_assoc - 1)) ? m_assoc - 1 : index;
}

#endif // __MEM_RUBY_SYSTEM_PSEUDOLRUPOLICY_HH__
