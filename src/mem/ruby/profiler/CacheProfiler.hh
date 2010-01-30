
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
 * CacheProfiler.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef CACHEPROFILER_H
#define CACHEPROFILER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/common/Histogram.hh"
#include "mem/protocol/AccessModeType.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/CacheRequestType.hh"

#include "params/CacheProfiler.hh"

template <class TYPE> class Vector;

class CacheProfiler : public SimObject {
public:
  // Constructors
  typedef CacheProfilerParams Params;
  CacheProfiler(const Params *);

  // Destructor
  ~CacheProfiler();

  // Public Methods
  void printStats(ostream& out) const;
  void clearStats();

  void addStatSample(CacheRequestType requestType, AccessModeType type, int msgSize, PrefetchBit pfBit);

  void print(ostream& out) const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  CacheProfiler(const CacheProfiler& obj);
  CacheProfiler& operator=(const CacheProfiler& obj);

  // Data Members (m_ prefix)
  string m_description;
  Histogram m_requestSize;
  int64 m_misses;
  int64 m_demand_misses;
  int64 m_prefetches;
  int64 m_sw_prefetches;
  int64 m_hw_prefetches;
  int64 m_accessModeTypeHistogram[AccessModeType_NUM];

  Vector < int >* m_requestTypeVec_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const CacheProfiler& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const CacheProfiler& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //CACHEPROFILER_H
