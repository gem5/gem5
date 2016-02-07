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

//
// This Deterministic Generator generates GETX requests for all nodes in the
// system.  The GETX requests are generated one at a time in round-robin fashion
// 0...1...2...etc.
//

#ifndef __CPU_DIRECTEDTEST_SERIESREQUESTGENERATOR_HH__
#define __CPU_DIRECTEDTEST_SERIESREQUESTGENERATOR_HH__

#include "cpu/testers/directedtest/DirectedGenerator.hh"
#include "cpu/testers/directedtest/RubyDirectedTester.hh"
#include "mem/protocol/SeriesRequestGeneratorStatus.hh"
#include "params/SeriesRequestGenerator.hh"

class SeriesRequestGenerator : public DirectedGenerator
{
  public:
    typedef SeriesRequestGeneratorParams Params;
    SeriesRequestGenerator(const Params *p);

    ~SeriesRequestGenerator();

    bool initiate();
    void performCallback(uint32_t proc, Addr address);

  private:
    SeriesRequestGeneratorStatus m_status;
    Addr m_address;
    uint32_t m_active_node;
    uint32_t m_addr_increment_size;
    uint32_t m_percent_writes;
};

#endif //__CPU_DIRECTEDTEST_SERIESREQUESTGENERATOR_HH__

