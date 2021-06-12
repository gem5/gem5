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


#include "mem/ruby/network/garnet/GarnetLink.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/NetworkBridge.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

GarnetIntLink::GarnetIntLink(const Params &p)
    : BasicIntLink(p)
{
    // Uni-directional

    m_network_link = p.network_link;
    m_credit_link = p.credit_link;

    srcCdcEn = p.src_cdc;
    dstCdcEn = p.dst_cdc;

    srcSerdesEn = p.src_serdes;
    dstSerdesEn = p.dst_serdes;

    srcBridgeEn = false;
    dstBridgeEn = false;

    if (srcCdcEn || srcSerdesEn) {
        srcBridgeEn = true;
        srcNetBridge = p.src_net_bridge;
        srcCredBridge = p.src_cred_bridge;
    }
    if (dstCdcEn || dstSerdesEn) {
        dstBridgeEn = true;
        dstNetBridge = p.dst_net_bridge;
        dstCredBridge = p.dst_cred_bridge;
    }
}

void
GarnetIntLink::init()
{
    if (srcBridgeEn) {
        assert(srcNetBridge && srcCredBridge);
        srcNetBridge->initBridge(srcCredBridge, srcCdcEn, srcSerdesEn);
        srcCredBridge->initBridge(srcNetBridge, srcCdcEn, srcSerdesEn);
    }

    if (dstBridgeEn) {
        assert(dstNetBridge && dstCredBridge);
        dstNetBridge->initBridge(dstCredBridge, dstCdcEn, dstSerdesEn);
        dstCredBridge->initBridge(dstNetBridge, dstCdcEn, dstSerdesEn);
    }
}

void
GarnetIntLink::print(std::ostream& out) const
{
    out << name();
}

GarnetExtLink::GarnetExtLink(const Params &p)
    : BasicExtLink(p)
{
    // Bi-directional

    // In
    m_network_links[0] = p.network_links[0];
    m_credit_links[0] = p.credit_links[0];

    // Out
    m_network_links[1] = p.network_links[1];
    m_credit_links[1] = p.credit_links[1];


    extCdcEn = p.ext_cdc;
    intCdcEn = p.int_cdc;

    extSerdesEn = p.ext_serdes;
    intSerdesEn = p.int_serdes;

    extBridgeEn = false;
    intBridgeEn = false;

    if (extCdcEn || extSerdesEn) {
        extBridgeEn = true;
        extNetBridge[0] = p.ext_net_bridge[0];
        extCredBridge[0] = p.ext_cred_bridge[0];
        extNetBridge[1] = p.ext_net_bridge[1];
        extCredBridge[1] = p.ext_cred_bridge[1];
    }

    if (intCdcEn || intSerdesEn) {
        intBridgeEn = true;
        intNetBridge[0] = p.int_net_bridge[0];
        intNetBridge[1] = p.int_net_bridge[1];
        intCredBridge[0] = p.int_cred_bridge[0];
        intCredBridge[1] = p.int_cred_bridge[1];
    }
}

void
GarnetExtLink::init()
{
    if (extBridgeEn) {
        assert(extNetBridge[0] && extCredBridge[0] &&
           extNetBridge[1] && extCredBridge[1]);
        extNetBridge[0]->initBridge(extCredBridge[0], extCdcEn, extSerdesEn);
        extCredBridge[0]->initBridge(extNetBridge[0], extCdcEn, extSerdesEn);
        extNetBridge[1]->initBridge(extCredBridge[1], extCdcEn, extSerdesEn);
        extCredBridge[1]->initBridge(extNetBridge[1], extCdcEn, extSerdesEn);
    }

    if (intBridgeEn) {
        assert(intNetBridge[0] && intCredBridge[0] &&
           intNetBridge[1] && intCredBridge[1]);
        intNetBridge[0]->initBridge(intCredBridge[0], intCdcEn, intSerdesEn);
        intCredBridge[0]->initBridge(intNetBridge[0], intCdcEn, intSerdesEn);
        intNetBridge[1]->initBridge(intCredBridge[1], intCdcEn, intSerdesEn);
        intCredBridge[1]->initBridge(intNetBridge[1], intCdcEn, intSerdesEn);
    }
}

void
GarnetExtLink::print(std::ostream& out) const
{
    out << name();
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
