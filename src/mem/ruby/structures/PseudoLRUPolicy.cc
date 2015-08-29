/*
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
 *
 * Author: Derek Hower
 */

#include "mem/ruby/structures/PseudoLRUPolicy.hh"



PseudoLRUPolicy::PseudoLRUPolicy(const Params * p)
    : AbstractReplacementPolicy(p)
{
    // associativity cannot exceed capacity of tree representation
    assert(m_num_sets > 0 &&
           m_assoc > 1 &&
           m_assoc <= (int64_t) sizeof(uint64_t)*4);

    m_trees = NULL;
    m_num_levels = 0;

    m_effective_assoc = 1;
    while (m_effective_assoc < m_assoc) {
        // effective associativity is ceiling power of 2
        m_effective_assoc <<= 1;
    }
    int tmp_assoc = m_effective_assoc;
    while (true) {
        tmp_assoc /= 2;
        if(!tmp_assoc) break;
        m_num_levels++;
    }
    assert(m_num_levels < sizeof(unsigned int)*4);
    m_trees = new uint64_t[m_num_sets];
    for (unsigned i = 0; i < m_num_sets; i++) {
        m_trees[i] = 0;
    }
}

PseudoLRUPolicy *
PseudoLRUReplacementPolicyParams::create()
{
    return new PseudoLRUPolicy(this);
}


PseudoLRUPolicy::~PseudoLRUPolicy()
{
    if (m_trees != NULL)
        delete[] m_trees;
}

void
PseudoLRUPolicy::touch(int64_t set, int64_t index, Tick time)
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

int64_t
PseudoLRUPolicy::getVictim(int64_t set) const
{
    int64_t index = 0;

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
