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

#ifndef __CPU_RUBYTEST_CHECKTABLE_HH__
#define __CPU_RUBYTEST_CHECKTABLE_HH__

#include <iostream>
#include <unordered_map>
#include <vector>

#include "base/random.hh"
#include "mem/ruby/common/Address.hh"

namespace gem5
{

class Check;
class RubyTester;

class CheckTable
{
  public:
    CheckTable(int _num_writers, int _num_readers, RubyTester* _tester);
    ~CheckTable();

    Check* getRandomCheck();
    Check* getCheck(Addr address);

    //  bool isPresent(const Address& address) const;
    //  void removeCheckFromTable(const Address& address);
    //  bool isTableFull() const;
    // Need a method to select a check or retrieve a check

    void print(std::ostream& out) const;

  private:
    void addCheck(Addr address);

    // Private copy constructor and assignment operator
    CheckTable(const CheckTable& obj);
    CheckTable& operator=(const CheckTable& obj);

    std::vector<Check*> m_check_vector;
    std::unordered_map<Addr, Check*> m_lookup_map;

    int m_num_writers;
    int m_num_readers;
    RubyTester* m_tester_ptr;
    Random::RandomPtr rng = Random::genRandom();
};

inline std::ostream&
operator<<(std::ostream& out, const CheckTable& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace gem5

#endif // __CPU_RUBYTEST_CHECKTABLE_HH__
