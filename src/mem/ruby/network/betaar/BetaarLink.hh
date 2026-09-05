/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
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

#ifndef __MEM_RUBY_NETWORK_BETAAR_0_BETAARLINK_HH__
#define __MEM_RUBY_NETWORK_BETAAR_0_BETAARLINK_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/betaar/BetaarCreditLink.hh"
#include "mem/ruby/network/betaar/BetaarNetworkBridge.hh"
#include "mem/ruby/network/betaar/BetaarNetworkLink.hh"
#include "params/BetaarExtLink.hh"
#include "params/BetaarIntLink.hh"

namespace gem5
{

namespace ruby
{

namespace betaar
{

class BetaarIntLink : public BasicIntLink
{
  public:
    typedef BetaarIntLinkParams Params;
    BetaarIntLink(const Params &p);

    void init();

    void print(std::ostream &out) const;

    friend class BetaarNetwork;

  protected:
    BetaarNetworkLink *m_network_link;
    BetaarCreditLink *m_credit_link;

    bool srcBridgeEn;
    bool dstBridgeEn;

    bool srcSerdesEn;
    bool dstSerdesEn;

    bool srcCdcEn;
    bool dstCdcEn;

    BetaarNetworkBridge *srcNetBridge;
    BetaarNetworkBridge *dstNetBridge;

    BetaarNetworkBridge *srcCredBridge;
    BetaarNetworkBridge *dstCredBridge;
};

inline std::ostream &
operator<<(std::ostream &out, const BetaarIntLink &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

class BetaarExtLink : public BasicExtLink
{
  public:
    typedef BetaarExtLinkParams Params;
    BetaarExtLink(const Params &p);

    void init();

    void print(std::ostream &out) const;

    friend class BetaarNetwork;

  protected:
    bool extBridgeEn;
    bool intBridgeEn;

    bool extSerdesEn;
    bool intSerdesEn;

    bool extCdcEn;
    bool intCdcEn;

    BetaarNetworkLink *m_network_links[2];
    BetaarCreditLink *m_credit_links[2];

    BetaarNetworkBridge *extNetBridge[2];
    BetaarNetworkBridge *intNetBridge[2];

    BetaarNetworkBridge *extCredBridge[2];
    BetaarNetworkBridge *intCredBridge[2];
};

inline std::ostream &
operator<<(std::ostream &out, const BetaarExtLink &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace betaar
} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_NETWORK_BETAAR_0_BETAARLINK_HH__
