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

#ifndef __MEM_RUBY_PROFILER_STORETRACE_HH__
#define __MEM_RUBY_PROFILER_STORETRACE_HH__

#include <iostream>

#include "base/types.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Histogram.hh"

namespace gem5
{

namespace ruby
{

class StoreTrace
{
  public:
    StoreTrace() {}

    explicit StoreTrace(Addr addr);
    ~StoreTrace();

    void store(NodeID node);
    void downgrade(NodeID node);

    int
    getTotal() const
    {
        return m_total_samples;
    }

    static void initSummary();
    static void printSummary(std::ostream &out);
    static void clearSummary();

    void print(std::ostream &out) const;

  private:
    static bool s_init;
    static int64_t s_total_samples; // Total number of store lifetimes
                                    // of all lines
    static Histogram *s_store_count_ptr;
    static Histogram *s_store_first_to_stolen_ptr;
    static Histogram *s_store_last_to_stolen_ptr;
    static Histogram *s_store_first_to_last_ptr;

    Addr m_addr;
    NodeID m_last_writer;
    Tick m_first_store;
    Tick m_last_store;
    int m_stores_this_interval;

    int64_t m_total_samples; // Total number of store lifetimes of this line
    Histogram m_store_count;
    Histogram m_store_first_to_stolen;
    Histogram m_store_last_to_stolen;
    Histogram m_store_first_to_last;
};

inline bool
node_less_then_eq(const StoreTrace *n1, const StoreTrace *n2)
{
    return n1->getTotal() > n2->getTotal();
}

inline std::ostream &
operator<<(std::ostream &out, const StoreTrace &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_PROFILER_STORETRACE_HH__
