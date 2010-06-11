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

#include "mem/gems_common/PrioHeap.hh"
#include "mem/ruby/profiler/CacheProfiler.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

CacheProfiler::CacheProfiler(const string& description)
    : m_requestTypeVec(int(CacheRequestType_NUM))
{
    m_description = description;

    clearStats();
}

CacheProfiler::~CacheProfiler()
{
}

void
CacheProfiler::printStats(ostream& out) const
{
    out << "Cache Stats: " << m_description << endl;
    string description = "  " + m_description;

    out << description << "_total_misses: " << m_misses << endl;
    out << description << "_total_demand_misses: " << m_demand_misses << endl;
    out << description << "_total_prefetches: " << m_prefetches << endl;
    out << description << "_total_sw_prefetches: " << m_sw_prefetches << endl;
    out << description << "_total_hw_prefetches: " << m_hw_prefetches << endl;
    out << endl;

    int requests = 0;

    for (int i = 0; i < int(CacheRequestType_NUM); i++) {
        requests += m_requestTypeVec[i];
    }

    assert(m_misses == requests);

    if (requests > 0) {
        for (int i = 0; i < int(CacheRequestType_NUM); i++) {
            if (m_requestTypeVec[i] > 0) {
                out << description << "_request_type_"
                    << CacheRequestType_to_string(CacheRequestType(i))
                    << ":   "
                    << 100.0 * (double)m_requestTypeVec[i] /
                    (double)requests
                    << "%" << endl;
            }
        }

        out << endl;

        for (int i = 0; i < AccessModeType_NUM; i++){
            if (m_accessModeTypeHistogram[i] > 0) {
                out << description << "_access_mode_type_"
                    << (AccessModeType) i << ":   "
                    << m_accessModeTypeHistogram[i] << "    "
                    << 100.0 * m_accessModeTypeHistogram[i] / requests
                    << "%" << endl;
            }
        }
    }

    out << description << "_request_size: " << m_requestSize << endl;
    out << endl;
}

void
CacheProfiler::clearStats()
{
    for (int i = 0; i < int(CacheRequestType_NUM); i++) {
        m_requestTypeVec[i] = 0;
    }
    m_requestSize.clear();
    m_misses = 0;
    m_demand_misses = 0;
    m_prefetches = 0;
    m_sw_prefetches = 0;
    m_hw_prefetches = 0;
    for (int i = 0; i < AccessModeType_NUM; i++) {
        m_accessModeTypeHistogram[i] = 0;
    }
}

void
CacheProfiler::addStatSample(CacheRequestType requestType,
                             AccessModeType type, int msgSize,
                             PrefetchBit pfBit)
{
    m_misses++;

    m_requestTypeVec[requestType]++;

    m_accessModeTypeHistogram[type]++;
    m_requestSize.add(msgSize);
    if (pfBit == PrefetchBit_No) {
        m_demand_misses++;
    } else if (pfBit == PrefetchBit_Yes) {
        m_prefetches++;
        m_sw_prefetches++;
    } else {
        // must be L1_HW || L2_HW prefetch
        m_prefetches++;
        m_hw_prefetches++;
    }
}
