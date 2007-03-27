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
    : busCmd(MemCmd::InvalidCmd), newState(-1), snoopFunc(invalidTransition)
{
}


void
CoherenceProtocol::regStats()
{
    // Even though we count all the possible transitions in the
    // requestCount and snoopCount arrays, most of these are invalid,
    // so we just select the interesting ones to print here.

    requestCount[Invalid][MemCmd::ReadReq]
        .name(name() + ".read_invalid")
        .desc("read misses to invalid blocks")
        ;

    requestCount[Invalid][MemCmd::WriteReq]
        .name(name() +".write_invalid")
        .desc("write misses to invalid blocks")
        ;

    requestCount[Invalid][MemCmd::SoftPFReq]
        .name(name() +".swpf_invalid")
        .desc("soft prefetch misses to invalid blocks")
        ;

    requestCount[Invalid][MemCmd::HardPFReq]
        .name(name() +".hwpf_invalid")
        .desc("hard prefetch misses to invalid blocks")
        ;

    requestCount[Shared][MemCmd::WriteReq]
        .name(name() + ".write_shared")
        .desc("write misses to shared blocks")
        ;

    requestCount[Owned][MemCmd::WriteReq]
        .name(name() + ".write_owned")
        .desc("write misses to owned blocks")
        ;

    snoopCount[Shared][MemCmd::ReadReq]
        .name(name() + ".snoop_read_shared")
        .desc("read snoops on shared blocks")
        ;

    snoopCount[Shared][MemCmd::ReadExReq]
        .name(name() + ".snoop_readex_shared")
        .desc("readEx snoops on shared blocks")
        ;

    snoopCount[Shared][MemCmd::UpgradeReq]
        .name(name() + ".snoop_upgrade_shared")
        .desc("upgradee snoops on shared blocks")
        ;

    snoopCount[Modified][MemCmd::ReadReq]
        .name(name() + ".snoop_read_modified")
        .desc("read snoops on modified blocks")
        ;

    snoopCount[Modified][MemCmd::ReadExReq]
        .name(name() + ".snoop_readex_modified")
        .desc("readEx snoops on modified blocks")
        ;

    snoopCount[Owned][MemCmd::ReadReq]
        .name(name() + ".snoop_read_owned")
        .desc("read snoops on owned blocks")
        ;

    snoopCount[Owned][MemCmd::ReadExReq]
        .name(name() + ".snoop_readex_owned")
        .desc("readEx snoops on owned blocks")
        ;

    snoopCount[Owned][MemCmd::UpgradeReq]
        .name(name() + ".snoop_upgrade_owned")
        .desc("upgrade snoops on owned blocks")
        ;

    snoopCount[Exclusive][MemCmd::ReadReq]
        .name(name() + ".snoop_read_exclusive")
        .desc("read snoops on exclusive blocks")
        ;

    snoopCount[Exclusive][MemCmd::ReadExReq]
        .name(name() + ".snoop_readex_exclusive")
        .desc("readEx snoops on exclusive blocks")
        ;

    snoopCount[Shared][MemCmd::InvalidateReq]
        .name(name() + ".snoop_inv_shared")
        .desc("Invalidate snoops on shared blocks")
        ;

    snoopCount[Owned][MemCmd::InvalidateReq]
        .name(name() + ".snoop_inv_owned")
        .desc("Invalidate snoops on owned blocks")
        ;

    snoopCount[Exclusive][MemCmd::InvalidateReq]
        .name(name() + ".snoop_inv_exclusive")
        .desc("Invalidate snoops on exclusive blocks")
        ;

    snoopCount[Modified][MemCmd::InvalidateReq]
        .name(name() + ".snoop_inv_modified")
        .desc("Invalidate snoops on modified blocks")
        ;

    snoopCount[Invalid][MemCmd::InvalidateReq]
        .name(name() + ".snoop_inv_invalid")
        .desc("Invalidate snoops on invalid blocks")
        ;

    snoopCount[Shared][MemCmd::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_shared")
        .desc("WriteInvalidate snoops on shared blocks")
        ;

    snoopCount[Owned][MemCmd::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_owned")
        .desc("WriteInvalidate snoops on owned blocks")
        ;

    snoopCount[Exclusive][MemCmd::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_exclusive")
        .desc("WriteInvalidate snoops on exclusive blocks")
        ;

    snoopCount[Modified][MemCmd::WriteInvalidateReq]
        .name(name() + ".snoop_writeinv_modified")
        .desc("WriteInvalidate snoops on modified blocks")
        ;

    snoopCount[Invalid][MemCmd::WriteInvalidateReq]
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
    typedef MemCmd MC;
    StateTransition (&tt)[stateMax+1][MC::NUM_MEM_CMDS] = transitionTable;

    MC::Command writeToSharedCmd =
        doUpgrades ? MC::UpgradeReq : MC::ReadExReq;
    MC::Command writeToSharedResp =
        doUpgrades ? MC::UpgradeReq : MC::ReadExResp;

    // Note that all transitions by default cause a panic.
    // Override the valid transitions with the appropriate actions here.

    //
    // ----- incoming requests: specify outgoing bus request -----
    //
    tt[Invalid][MC::ReadReq].onRequest(MC::ReadReq);
    // we only support write allocate right now
    tt[Invalid][MC::WriteReq].onRequest(MC::ReadExReq);
    tt[Invalid][MC::SwapReq].onRequest(MC::ReadExReq);
    tt[Shared][MC::WriteReq].onRequest(writeToSharedCmd);
    tt[Shared][MC::SwapReq].onRequest(writeToSharedCmd);
    if (hasOwned) {
        tt[Owned][MC::WriteReq].onRequest(writeToSharedCmd);
        tt[Owned][MC::SwapReq].onRequest(writeToSharedCmd);
    }

    // Prefetching causes a read
    tt[Invalid][MC::SoftPFReq].onRequest(MC::ReadReq);
    tt[Invalid][MC::HardPFReq].onRequest(MC::ReadReq);

    //
    // ----- on response to given request: specify new state -----
    //
    tt[Invalid][MC::ReadExResp].onResponse(Modified);
    tt[Shared][writeToSharedResp].onResponse(Modified);
    // Go to Exclusive state on read response if we have one (will
    // move into shared if the shared line is asserted in the
    // getNewState function)
    //
    // originally had this as:
    // tt[Invalid][MC::ReadResp].onResponse(hasExclusive ? Exclusive: Shared);
    // ...but for some reason that caused a link error...
    if (hasExclusive) {
        tt[Invalid][MC::ReadResp].onResponse(Exclusive);
    } else {
        tt[Invalid][MC::ReadResp].onResponse(Shared);
    }
    if (hasOwned) {
        tt[Owned][writeToSharedResp].onResponse(Modified);
    }

    //
    // ----- bus snoop transition functions -----
    //
    tt[Invalid][MC::ReadReq].onSnoop(nullTransition);
    tt[Invalid][MC::ReadExReq].onSnoop(nullTransition);
    tt[Invalid][MC::InvalidateReq].onSnoop(invalidateTrans);
    tt[Invalid][MC::WriteInvalidateReq].onSnoop(invalidateTrans);
    tt[Shared][MC::ReadReq].onSnoop(hasExclusive
                                   ? assertShared : nullTransition);
    tt[Shared][MC::ReadExReq].onSnoop(invalidateTrans);
    tt[Shared][MC::InvalidateReq].onSnoop(invalidateTrans);
    tt[Shared][MC::WriteInvalidateReq].onSnoop(invalidateTrans);
    if (doUpgrades) {
        tt[Invalid][MC::UpgradeReq].onSnoop(nullTransition);
        tt[Shared][MC::UpgradeReq].onSnoop(invalidateTrans);
    }
    tt[Modified][MC::ReadExReq].onSnoop(supplyAndInvalidateTrans);
    tt[Modified][MC::ReadReq].onSnoop(hasOwned
                                     ? supplyAndGotoOwnedTrans
                                     : supplyAndGotoSharedTrans);
    tt[Modified][MC::InvalidateReq].onSnoop(invalidateTrans);
    tt[Modified][MC::WriteInvalidateReq].onSnoop(invalidateTrans);

    if (hasExclusive) {
        tt[Exclusive][MC::ReadReq].onSnoop(assertShared);
        tt[Exclusive][MC::ReadExReq].onSnoop(invalidateTrans);
        tt[Exclusive][MC::InvalidateReq].onSnoop(invalidateTrans);
        tt[Exclusive][MC::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    if (hasOwned) {
        tt[Owned][MC::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        tt[Owned][MC::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        tt[Owned][MC::UpgradeReq].onSnoop(invalidateTrans);
        tt[Owned][MC::InvalidateReq].onSnoop(invalidateTrans);
        tt[Owned][MC::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    // @todo add in hardware prefetch to this list
}


MemCmd
CoherenceProtocol::getBusCmd(MemCmd cmdIn, CacheBlk::State state,
                             MSHR *mshr)
{
    state &= stateMask;
    int cmd_idx = cmdIn.toInt();

    assert(0 <= state && state <= stateMax);
    assert(0 <= cmd_idx && cmd_idx < MemCmd::NUM_MEM_CMDS);

    MemCmd::Command cmdOut = transitionTable[state][cmd_idx].busCmd;

    assert(cmdOut != MemCmd::InvalidCmd);

    ++requestCount[state][cmd_idx];

    return cmdOut;
}


CacheBlk::State
CoherenceProtocol::getNewState(PacketPtr &pkt, CacheBlk::State oldState)
{
    CacheBlk::State state = oldState & stateMask;
    int cmd_idx = pkt->cmdToIndex();

    assert(0 <= state && state <= stateMax);
    assert(0 <= cmd_idx && cmd_idx < MemCmd::NUM_MEM_CMDS);

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
    assert(0 <= cmd_idx && cmd_idx < MemCmd::NUM_MEM_CMDS);

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
