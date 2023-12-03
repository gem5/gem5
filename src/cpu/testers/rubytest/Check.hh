/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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

#ifndef __CPU_RUBYTEST_CHECK_HH__
#define __CPU_RUBYTEST_CHECK_HH__

#include <iostream>

#include "cpu/testers/rubytest/RubyTester.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/TesterStatus.hh"

namespace gem5
{

namespace ruby
{
class SubBlock;
} // namespace ruby

const int CHECK_SIZE_BITS = 2;
const int CHECK_SIZE = (1 << CHECK_SIZE_BITS);

class Check
{
  public:
    Check(Addr address, Addr pc, int _num_writers, int _num_readers,
          RubyTester *_tester);

    void initiate(); // Does Action or Check or nether
    void performCallback(ruby::NodeID proc, ruby::SubBlock *data,
                         Cycles curTime);

    Addr
    getAddress() const
    {
        return m_address;
    }

    void changeAddress(Addr address);

    void print(std::ostream &out) const;

  private:
    void initiateFlush();
    void initiatePrefetch();
    void initiateAction();
    void initiateCheck();

    void pickValue();
    void pickInitiatingNode();

    void debugPrint();

    ruby::TesterStatus m_status;
    uint8_t m_value;
    int m_store_count;
    ruby::NodeID m_initiatingNode;
    Addr m_address;
    Addr m_pc;
    ruby::RubyAccessMode m_access_mode;
    int m_num_writers;
    int m_num_readers;
    RubyTester *m_tester_ptr;
};

inline std::ostream &
operator<<(std::ostream &out, const Check &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace gem5

#endif // __CPU_RUBYTEST_CHECK_HH__
