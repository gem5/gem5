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
#include "mem/ruby/common/TypeDefines.hh"
#include "params/ReplacementPolicy.hh"
#include "sim/sim_object.hh"

class CacheMemory;

class AbstractReplacementPolicy : public SimObject
{
  public:
    typedef ReplacementPolicyParams Params;
    AbstractReplacementPolicy(const Params * p);
    virtual ~AbstractReplacementPolicy();

    /* touch a block. a.k.a. update timestamp */
    virtual void touch(int64_t set, int64_t way, Tick time) = 0;

    /* returns the way to replace */
    virtual int64_t getVictim(int64_t set) const = 0;

    /* get the time of the last access */
    Tick getLastAccess(int64_t set, int64_t way);

    virtual bool useOccupancy() const { return false; }

    void setCache(CacheMemory * pCache) {m_cache = pCache;}
    CacheMemory * m_cache;

  protected:
    unsigned m_num_sets;       /** total number of sets */
    unsigned m_assoc;          /** set associativity */
    Tick **m_last_ref_ptr;         /** timestamp of last reference */
};

#endif // __MEM_RUBY_STRUCTURES_ABSTRACTREPLACEMENTPOLICY_HH__
