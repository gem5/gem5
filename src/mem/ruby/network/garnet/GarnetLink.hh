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


#ifndef __MEM_RUBY_NETWORK_GARNET_0_GARNETLINK_HH__
#define __MEM_RUBY_NETWORK_GARNET_0_GARNETLINK_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/NetworkBridge.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "params/GarnetExtLink.hh"
#include "params/GarnetIntLink.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

class GarnetIntLink : public BasicIntLink
{
  public:
    typedef GarnetIntLinkParams Params;
    GarnetIntLink(const Params &p);

    void init();

    void print(std::ostream& out) const;

    friend class GarnetNetwork;

  protected:
    NetworkLink* m_network_link;
    CreditLink* m_credit_link;

    bool srcBridgeEn;
    bool dstBridgeEn;

    bool srcSerdesEn;
    bool dstSerdesEn;

    bool srcCdcEn;
    bool dstCdcEn;

    NetworkBridge* srcNetBridge;
    NetworkBridge* dstNetBridge;

    NetworkBridge* srcCredBridge;
    NetworkBridge* dstCredBridge;
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetIntLink& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

class GarnetExtLink : public BasicExtLink
{
  public:
    typedef GarnetExtLinkParams Params;
    GarnetExtLink(const Params &p);

    void init();

    void print(std::ostream& out) const;

    friend class GarnetNetwork;

  protected:
    bool extBridgeEn;
    bool intBridgeEn;

    bool extSerdesEn;
    bool intSerdesEn;

    bool extCdcEn;
    bool intCdcEn;

    NetworkLink* m_network_links[2];
    CreditLink* m_credit_links[2];

    NetworkBridge* extNetBridge[2];
    NetworkBridge* intNetBridge[2];

    NetworkBridge* extCredBridge[2];
    NetworkBridge* intCredBridge[2];

};

inline std::ostream&
operator<<(std::ostream& out, const GarnetExtLink& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace garnet
} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_NETWORK_GARNET_0_GARNETLINK_HH__
