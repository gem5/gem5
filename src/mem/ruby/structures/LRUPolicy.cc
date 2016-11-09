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

#include "mem/ruby/structures/LRUPolicy.hh"

LRUPolicy::LRUPolicy(const Params * p)
    : AbstractReplacementPolicy(p)
{
}


LRUPolicy::~LRUPolicy()
{
}

LRUPolicy *
LRUReplacementPolicyParams::create()
{
    return new LRUPolicy(this);
}


void
LRUPolicy::touch(int64_t set, int64_t index, Tick time)
{
    assert(index >= 0 && index < m_assoc);
    assert(set >= 0 && set < m_num_sets);

    m_last_ref_ptr[set][index] = time;
}

int64_t
LRUPolicy::getVictim(int64_t set) const
{
    Tick time, smallest_time;
    int64_t smallest_index = 0;
    smallest_time = m_last_ref_ptr[set][0];

    for (unsigned i = 0; i < m_assoc; i++) {
        time = m_last_ref_ptr[set][i];

        if (time < smallest_time) {
            smallest_index = i;
            smallest_time = time;
        }
    }

    return smallest_index;
}
