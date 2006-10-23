/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Erik Hallnor
 *          Steve Reinhardt
 *          Ron Dreslinski
 */

/**
 * @file
 * Definitions of CoherenceProtocol.
 */

#include <string>

#include "base/misc.hh"
#include "mem/cache/miss/mshr.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/coherence/coherence_protocol.hh"
#include "sim/builder.hh"

using namespace std;


CoherenceProtocol::StateTransition::StateTransition()
    : busCmd(Packet::InvalidCmd), newState(-1), snoopFunc(invalidTransition)
{
}


void
CoherenceProtocol::regStats()
{
    // Even though we count all the possible transitions in the
    // requestCount and snoopCount arrays, most of these are invalid,
    // so we just select the interesting ones to print here.

    requestCount[Invalid][Packet::ReadReq]
        .name(name() + ".read_invalid")
        .desc("read misses to invalid blocks")
        ;

    requestCount[Invalid][Packet::WriteReq]
        .name(name() +".write_invalid")
        .desc("write misses to invalid blocks")
        ;

    requestCount[Invalid][Packet::SoftPFReq]
        .name(name() +".swpf_invalid")
        .desc("soft prefetch misses to invalid blocks")
        ;

    requestCount[Invalid][Packet::HardPFReq]
        .name(name() +".hwpf_invalid")
        .desc("hard prefetch misses to invalid blocks")
        ;

    requestCount[Shared][Packet::WriteReq]
        .name(name() + ".write_shared")
        .desc("write misses to shared blocks")
        ;

    requestCount[Owned][Packet::WriteReq]
        .name(name() + ".write_owned")
        .desc("write misses to owned blocks")
        ;

    snoopCount[Shared][Packet::ReadReq]
        .name(name() + ".snoop_read_shared")
        .desc("read snoops on shared blocks")
        ;

    snoopCount[Shared][Packet::ReadExReq]
        .name(name() + ".snoop_readex_shared")
        .desc("readEx snoops on shared blocks")
        ;

    snoopCount[Shared][Packet::UpgradeReq]
        .name(name() + ".snoop_upgrade_shared")
        .desc("upgradee snoops on shared blocks")
        ;

    snoopCount[Modified][Packet::ReadReq]
        .name(name() + ".snoop_read_modified")
        .desc("read snoops on modified blocks")
        ;

    snoopCount[Modified][Packet::ReadExReq]
        .name(name() + ".snoop_readex_modified")
        .desc("readEx snoops on modified blocks")
        ;

    snoopCount[Owned][Packet::ReadReq]
        .name(name() + ".snoop_read_owned")
        .desc("read snoops on owned blocks")
        ;

    snoopCount[Owned][Packet::ReadExReq]
        .name(name() + ".snoop_readex_owned")
        .desc("readEx snoops on owned blocks")
        ;

    snoopCount[Owned][Packet::UpgradeReq]
        .name(name() + ".snoop_upgrade_owned")
        .desc("upgrade snoops on owned blocks")
        ;

    snoopCount[Exclusive][Packet::ReadReq]
        .name(name() + ".snoop_read_exclusive")
        .desc("read snoops on exclusive blocks")
        ;

    snoopCount[Exclusive][Packet::ReadExReq]
        .name(name() + ".snoop_readex_exclusive")
        .desc("readEx snoops on exclusive blocks")
        ;

    snoopCount[Shared][Packet::InvalidateReq]
        .name(name() + ".snoop_inv_shared")
        .desc("Invalidate snoops on shared blocks")
        ;

    snoopCount[Owned][Packet::InvalidateReq]
        .name(name() + ".snoop_inv_owned")
        .desc("Invalidate snoops on owned blocks")
        ;

    snoopCount[Exclusive][Packet::InvalidateReq]
        .name(name() + ".snoop_inv_exclusive")
        .desc("Invalidate snoops on exclusive blocks")
        ;

    snoopCount[Modified][Packet::InvalidateReq]
        .name(name() + ".snoop_inv_modified")
        .desc("Invalidate snoops on modified blocks")
        ;

    snoopCount[Invalid][Packet::InvalidateReq]
        .name(name() + ".snoop_inv_invalid")
        .desc("Invalidate snoops on invalid blocks")
        ;

    snoopCount[Shared][Packet::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_shared")
        .desc("WriteInvalidate snoops on shared blocks")
        ;

    snoopCount[Owned][Packet::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_owned")
        .desc("WriteInvalidate snoops on owned blocks")
        ;

    snoopCount[Exclusive][Packet::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_exclusive")
        .desc("WriteInvalidate snoops on exclusive blocks")
        ;

    snoopCount[Modified][Packet::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_modified")
        .desc("WriteInvalidate snoops on modified blocks")
        ;

    snoopCount[Invalid][Packet::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_invalid")
        .desc("WriteInvalidate snoops on invalid blocks")
        ;
}


bool
CoherenceProtocol::invalidateTrans(BaseCache *cache, PacketPtr &pkt,
                                   CacheBlk *blk, MSHR *mshr,
                                   CacheBlk::State & new_state)
{
    // invalidate the block
    new_state = (blk->status & ~stateMask) | Invalid;
    return false;
}


bool
CoherenceProtocol::supplyTrans(BaseCache *cache, PacketPtr &pkt,
                               CacheBlk *blk,
                               MSHR *mshr,
                               CacheBlk::State & new_state)
{
    return true;
}


bool
CoherenceProtocol::supplyAndGotoSharedTrans(BaseCache *cache, PacketPtr &pkt,
                                            CacheBlk *blk,
                                            MSHR *mshr,
                                            CacheBlk::State & new_state)
{
    new_state = (blk->status & ~stateMask) | Shared;
    pkt->flags |= SHARED_LINE;
    return supplyTrans(cache, pkt, blk, mshr, new_state);
}


bool
CoherenceProtocol::supplyAndGotoOwnedTrans(BaseCache *cache, PacketPtr &pkt,
                                           CacheBlk *blk,
                                           MSHR *mshr,
                                           CacheBlk::State & new_state)
{
    new_state = (blk->status & ~stateMask) | Owned;
    pkt->flags |= SHARED_LINE;
    return supplyTrans(cache, pkt, blk, mshr, new_state);
}


bool
CoherenceProtocol::supplyAndInvalidateTrans(BaseCache *cache, PacketPtr &pkt,
                                            CacheBlk *blk,
                                            MSHR *mshr,
                                            CacheBlk::State & new_state)
{
    new_state = (blk->status & ~stateMask) | Invalid;
    return supplyTrans(cache, pkt, blk, mshr, new_state);
}

bool
CoherenceProtocol::assertShared(BaseCache *cache, PacketPtr &pkt,
                                            CacheBlk *blk,
                                            MSHR *mshr,
                                            CacheBlk::State & new_state)
{
    new_state = (blk->status & ~stateMask) | Shared;
    pkt->flags |= SHARED_LINE;
    return false;
}

CoherenceProtocol::CoherenceProtocol(const string &name,
                                     const string &protocol,
                                     const bool doUpgrades)
    : SimObject(name)
{
    // Python should catch this, but in case it doesn't...
    if (!(protocol == "msi"  || protocol == "mesi" ||
          protocol == "mosi" || protocol == "moesi")) {
        fatal("CoherenceProtocol: unrecognized protocol %s\n",  protocol);
    }

    bool hasOwned = (protocol == "mosi" || protocol == "moesi");
    bool hasExclusive = (protocol == "mesi" || protocol == "moesi");

    if (hasOwned && !doUpgrades) {
        fatal("CoherenceProtocol: ownership protocols require upgrade "
              "transactions\n(write miss on owned block generates ReadExcl, "
              "which will clobber dirty block)\n");
    }

    // set up a few shortcuts to save typing & visual clutter
    typedef Packet P;
    StateTransition (&tt)[stateMax+1][NUM_MEM_CMDS] = transitionTable;

    P::Command writeToSharedCmd =  doUpgrades ? P::UpgradeReq : P::ReadExReq;
    P::Command writeToSharedResp = doUpgrades ? P::UpgradeReq : P::ReadExResp;

    // Note that all transitions by default cause a panic.
    // Override the valid transitions with the appropriate actions here.

    //
    // ----- incoming requests: specify outgoing bus request -----
    //
    tt[Invalid][P::ReadReq].onRequest(P::ReadReq);
    // we only support write allocate right now
    tt[Invalid][P::WriteReq].onRequest(P::ReadExReq);
    tt[Shared][P::WriteReq].onRequest(writeToSharedCmd);
    if (hasOwned) {
        tt[Owned][P::WriteReq].onRequest(writeToSharedCmd);
    }

    // Prefetching causes a read
    tt[Invalid][P::SoftPFReq].onRequest(P::ReadReq);
    tt[Invalid][P::HardPFReq].onRequest(P::ReadReq);

    //
    // ----- on response to given request: specify new state -----
    //
    tt[Invalid][P::ReadExResp].onResponse(Modified);
    tt[Shared][writeToSharedResp].onResponse(Modified);
    // Go to Exclusive state on read response if we have one (will
    // move into shared if the shared line is asserted in the
    // getNewState function)
    //
    // originally had this as:
    // tt[Invalid][P::ReadResp].onResponse(hasExclusive ? Exclusive: Shared);
    // ...but for some reason that caused a link error...
    if (hasExclusive) {
        tt[Invalid][P::ReadResp].onResponse(Exclusive);
    } else {
        tt[Invalid][P::ReadResp].onResponse(Shared);
    }
    if (hasOwned) {
        tt[Owned][writeToSharedResp].onResponse(Modified);
    }

    //
    // ----- bus snoop transition functions -----
    //
    tt[Invalid][P::ReadReq].onSnoop(nullTransition);
    tt[Invalid][P::ReadExReq].onSnoop(nullTransition);
    tt[Invalid][P::InvalidateReq].onSnoop(invalidateTrans);
    tt[Invalid][P::WriteInvalidateReq].onSnoop(invalidateTrans);
    tt[Shared][P::ReadReq].onSnoop(hasExclusive
                                   ? assertShared : nullTransition);
    tt[Shared][P::ReadExReq].onSnoop(invalidateTrans);
    tt[Shared][P::InvalidateReq].onSnoop(invalidateTrans);
    tt[Shared][P::WriteInvalidateReq].onSnoop(invalidateTrans);
    if (doUpgrades) {
        tt[Invalid][P::UpgradeReq].onSnoop(nullTransition);
        tt[Shared][P::UpgradeReq].onSnoop(invalidateTrans);
    }
    tt[Modified][P::ReadExReq].onSnoop(supplyAndInvalidateTrans);
    tt[Modified][P::ReadReq].onSnoop(hasOwned
                                     ? supplyAndGotoOwnedTrans
                                     : supplyAndGotoSharedTrans);
    tt[Modified][P::InvalidateReq].onSnoop(invalidateTrans);
    tt[Modified][P::WriteInvalidateReq].onSnoop(invalidateTrans);

    if (hasExclusive) {
        tt[Exclusive][P::ReadReq].onSnoop(assertShared);
        tt[Exclusive][P::ReadExReq].onSnoop(invalidateTrans);
        tt[Exclusive][P::InvalidateReq].onSnoop(invalidateTrans);
        tt[Exclusive][P::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    if (hasOwned) {
        tt[Owned][P::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        tt[Owned][P::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        tt[Owned][P::UpgradeReq].onSnoop(invalidateTrans);
        tt[Owned][P::InvalidateReq].onSnoop(invalidateTrans);
        tt[Owned][P::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    // @todo add in hardware prefetch to this list
}


Packet::Command
CoherenceProtocol::getBusCmd(Packet::Command cmdIn, CacheBlk::State state,
                             MSHR *mshr)
{
    state &= stateMask;
    int cmd_idx = (int) cmdIn;

    assert(0 <= state && state <= stateMax);
    assert(0 <= cmd_idx && cmd_idx < NUM_MEM_CMDS);

    Packet::Command cmdOut = transitionTable[state][cmd_idx].busCmd;

    assert(cmdOut != Packet::InvalidCmd);

    ++requestCount[state][cmd_idx];

    return cmdOut;
}


CacheBlk::State
CoherenceProtocol::getNewState(PacketPtr &pkt, CacheBlk::State oldState)
{
    CacheBlk::State state = oldState & stateMask;
    int cmd_idx = pkt->cmdToIndex();

    assert(0 <= state && state <= stateMax);
    assert(0 <= cmd_idx && cmd_idx < NUM_MEM_CMDS);

    CacheBlk::State newState = transitionTable[state][cmd_idx].newState;

    //Check if it's exclusive and the shared line was asserted,
    //then  goto shared instead
    if (newState == Exclusive && (pkt->flags & SHARED_LINE)) {
        newState = Shared;
    }

    assert(newState != -1);

    //Make sure not to loose any other state information
    newState = (oldState & ~stateMask) | newState;
    return newState;
}


bool
CoherenceProtocol::handleBusRequest(BaseCache *cache, PacketPtr &pkt,
                                    CacheBlk *blk,
                                    MSHR *mshr,
                                    CacheBlk::State & new_state)
{
    if (blk == NULL) {
        // nothing to do if we don't have a block
        return false;
    }

    CacheBlk::State state = blk->status & stateMask;
    int cmd_idx = pkt->cmdToIndex();

    assert(0 <= state && state <= stateMax);
    assert(0 <= cmd_idx && cmd_idx < NUM_MEM_CMDS);

//    assert(mshr == NULL); // can't currently handle outstanding requests
    //Check first if MSHR, and also insure, if there is one, that it is not in service
    assert(!mshr || mshr->inService == 0);
    ++snoopCount[state][cmd_idx];

    bool ret = transitionTable[state][cmd_idx].snoopFunc(cache, pkt, blk, mshr,
                                                     new_state);



    return ret;
}

bool
CoherenceProtocol::nullTransition(BaseCache *cache, PacketPtr &pkt,
                                  CacheBlk *blk, MSHR *mshr,
                                  CacheBlk::State & new_state)
{
    // do nothing
    if (blk)
        new_state = blk->status;
    return false;
}


bool
CoherenceProtocol::invalidTransition(BaseCache *cache, PacketPtr &pkt,
                                     CacheBlk *blk, MSHR *mshr,
                                     CacheBlk::State & new_state)
{
    panic("Invalid transition");
    return false;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(CoherenceProtocol)

    Param<string> protocol;
    Param<bool> do_upgrades;

END_DECLARE_SIM_OBJECT_PARAMS(CoherenceProtocol)


BEGIN_INIT_SIM_OBJECT_PARAMS(CoherenceProtocol)

    INIT_PARAM(protocol, "name of coherence protocol"),
    INIT_PARAM_DFLT(do_upgrades, "use upgrade transactions?", true)

END_INIT_SIM_OBJECT_PARAMS(CoherenceProtocol)


CREATE_SIM_OBJECT(CoherenceProtocol)
{
    return new CoherenceProtocol(getInstanceName(), protocol,
                                 do_upgrades);
}

REGISTER_SIM_OBJECT("CoherenceProtocol", CoherenceProtocol)

#endif // DOXYGEN_SHOULD_SKIP_THIS
