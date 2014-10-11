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

#ifndef __MEM_RUBY_STRUCTURES_ABSTRACTREPLACEMENTPOLICY_HH__
#define __MEM_RUBY_STRUCTURES_ABSTRACTREPLACEMENTPOLICY_HH__

#include "base/types.hh"

class AbstractReplacementPolicy
{
  public:
    AbstractReplacementPolicy(int64 num_sets, int64 assoc);
    virtual ~AbstractReplacementPolicy();

    /* touch a block. a.k.a. update timestamp */
    virtual void touch(int64 set, int64 way, Tick time) = 0;

    /* returns the way to replace */
    virtual int64 getVictim(int64 set) const = 0;

    /* get the time of the last access */
    Tick getLastAccess(int64 set, int64 way);

  protected:
    unsigned m_num_sets;       /** total number of sets */
    unsigned m_assoc;          /** set associativity */
    Tick **m_last_ref_ptr;         /** timestamp of last reference */
};

inline
AbstractReplacementPolicy::AbstractReplacementPolicy(int64 num_sets,
                                                     int64 assoc)
{
    m_num_sets = num_sets;
    m_assoc = assoc;
    m_last_ref_ptr = new Tick*[m_num_sets];
    for(unsigned i = 0; i < m_num_sets; i++){
        m_last_ref_ptr[i] = new Tick[m_assoc];
        for(unsigned j = 0; j < m_assoc; j++){
            m_last_ref_ptr[i][j] = 0;
        }
    }
}

inline
AbstractReplacementPolicy::~AbstractReplacementPolicy()
{
    if (m_last_ref_ptr != NULL){
        for (unsigned i = 0; i < m_num_sets; i++){
            if (m_last_ref_ptr[i] != NULL){
                delete[] m_last_ref_ptr[i];
            }
        }
        delete[] m_last_ref_ptr;
    }
}

inline Tick
AbstractReplacementPolicy::getLastAccess(int64 set, int64 way)
{
    return m_last_ref_ptr[set][way];
}

#endif // __MEM_RUBY_STRUCTURES_ABSTRACTREPLACEMENTPOLICY_HH__
