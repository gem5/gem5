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
                               CacheBlk::State & new_state
                               )
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
    if ((protocol == "mosi" || protocol == "moesi") && !doUpgrades) {
        cerr << "CoherenceProtocol: ownership protocols require upgrade transactions"
             << "(write miss on owned block generates ReadExcl, which will clobber dirty block)"
             << endl;
        fatal("");
    }

    Packet::Command writeToSharedCmd = doUpgrades ? Packet::UpgradeReq : Packet::ReadExReq;
    Packet::Command writeToSharedResp = doUpgrades ? Packet::UpgradeReq : Packet::ReadExResp;

//@todo add in hardware prefetch to this list
    if (protocol == "msi") {
        // incoming requests: specify outgoing bus request
        transitionTable[Invalid][Packet::ReadReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::WriteReq].onRequest(Packet::ReadExReq);
        transitionTable[Shared][Packet::WriteReq].onRequest(writeToSharedCmd);
        //Prefetching causes a read
        transitionTable[Invalid][Packet::SoftPFReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::HardPFReq].onRequest(Packet::ReadReq);

        // on response to given request: specify new state
        transitionTable[Invalid][Packet::ReadResp].onResponse(Shared);
        transitionTable[Invalid][Packet::ReadExResp].onResponse(Modified);
        transitionTable[Shared][writeToSharedResp].onResponse(Modified);

        // bus snoop transition functions
        transitionTable[Invalid][Packet::ReadReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::ReadExReq].onSnoop(nullTransition);
        transitionTable[Shared][Packet::ReadReq].onSnoop(nullTransition);
        transitionTable[Shared][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Modified][Packet::ReadReq].onSnoop(supplyAndGotoSharedTrans);
        //Tansitions on seeing a DMA (writeInv(samelevel) or DMAInv)
        transitionTable[Invalid][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Invalid][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);

        if (doUpgrades) {
            transitionTable[Invalid][Packet::UpgradeReq].onSnoop(nullTransition);
            transitionTable[Shared][Packet::UpgradeReq].onSnoop(invalidateTrans);
        }
    }

    else if(protocol == "mesi") {
        // incoming requests: specify outgoing bus request
        transitionTable[Invalid][Packet::ReadReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::WriteReq].onRequest(Packet::ReadExReq);
        transitionTable[Shared][Packet::WriteReq].onRequest(writeToSharedCmd);
        //Prefetching causes a read
        transitionTable[Invalid][Packet::SoftPFReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::HardPFReq].onRequest(Packet::ReadReq);

        // on response to given request: specify new state
        transitionTable[Invalid][Packet::ReadResp].onResponse(Exclusive);
        //It will move into shared if the shared line is asserted in the
        //getNewState function
        transitionTable[Invalid][Packet::ReadExResp].onResponse(Modified);
        transitionTable[Shared][writeToSharedResp].onResponse(Modified);

        // bus snoop transition functions
        transitionTable[Invalid][Packet::ReadReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::ReadExReq].onSnoop(nullTransition);
        transitionTable[Shared][Packet::ReadReq].onSnoop(assertShared);
        transitionTable[Shared][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::ReadReq].onSnoop(assertShared);
        transitionTable[Exclusive][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Modified][Packet::ReadReq].onSnoop(supplyAndGotoSharedTrans);
        //Tansitions on seeing a DMA (writeInv(samelevel) or DMAInv)
        transitionTable[Invalid][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Invalid][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);

        if (doUpgrades) {
            transitionTable[Invalid][Packet::UpgradeReq].onSnoop(nullTransition);
            transitionTable[Shared][Packet::UpgradeReq].onSnoop(invalidateTrans);
        }
    }

    else if(protocol == "mosi") {
        // incoming requests: specify outgoing bus request
        transitionTable[Invalid][Packet::ReadReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::WriteReq].onRequest(Packet::ReadExReq);
        transitionTable[Shared][Packet::WriteReq].onRequest(writeToSharedCmd);
        transitionTable[Owned][Packet::WriteReq].onRequest(writeToSharedCmd);
        //Prefetching causes a read
        transitionTable[Invalid][Packet::SoftPFReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::HardPFReq].onRequest(Packet::ReadReq);

        // on response to given request: specify new state
        transitionTable[Invalid][Packet::ReadResp].onResponse(Shared);
        transitionTable[Invalid][Packet::ReadExResp].onResponse(Modified);
        transitionTable[Shared][writeToSharedResp].onResponse(Modified);
        transitionTable[Owned][writeToSharedResp].onResponse(Modified);

        // bus snoop transition functions
        transitionTable[Invalid][Packet::ReadReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::ReadExReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::UpgradeReq].onSnoop(nullTransition);
        transitionTable[Shared][Packet::ReadReq].onSnoop(assertShared);
        transitionTable[Shared][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::UpgradeReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Modified][Packet::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        transitionTable[Owned][Packet::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        transitionTable[Owned][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Owned][Packet::UpgradeReq].onSnoop(invalidateTrans);
        //Tansitions on seeing a DMA (writeInv(samelevel) or DMAInv)
        transitionTable[Invalid][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Owned][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Invalid][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Owned][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    else if(protocol == "moesi") {
        // incoming requests: specify outgoing bus request
        transitionTable[Invalid][Packet::ReadReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::WriteReq].onRequest(Packet::ReadExReq);
        transitionTable[Shared][Packet::WriteReq].onRequest(writeToSharedCmd);
        transitionTable[Owned][Packet::WriteReq].onRequest(writeToSharedCmd);
        //Prefetching causes a read
        transitionTable[Invalid][Packet::SoftPFReq].onRequest(Packet::ReadReq);
        transitionTable[Invalid][Packet::HardPFReq].onRequest(Packet::ReadReq);

        // on response to given request: specify new state
        transitionTable[Invalid][Packet::ReadResp].onResponse(Exclusive);
        //It will move into shared if the shared line is asserted in the
        //getNewState function
        transitionTable[Invalid][Packet::ReadExResp].onResponse(Modified);
        transitionTable[Shared][writeToSharedResp].onResponse(Modified);
        transitionTable[Owned][writeToSharedResp].onResponse(Modified);

        // bus snoop transition functions
        transitionTable[Invalid][Packet::ReadReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::ReadExReq].onSnoop(nullTransition);
        transitionTable[Invalid][Packet::UpgradeReq].onSnoop(nullTransition);
        transitionTable[Shared][Packet::ReadReq].onSnoop(assertShared);
        transitionTable[Shared][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::UpgradeReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::ReadReq].onSnoop(assertShared);
        transitionTable[Exclusive][Packet::ReadExReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        transitionTable[Modified][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Owned][Packet::ReadReq].onSnoop(supplyAndGotoOwnedTrans);
        transitionTable[Owned][Packet::ReadExReq].onSnoop(supplyAndInvalidateTrans);
        transitionTable[Owned][Packet::UpgradeReq].onSnoop(invalidateTrans);
        //Transitions on seeing a DMA (writeInv(samelevel) or DMAInv)
        transitionTable[Invalid][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Owned][Packet::InvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Invalid][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Shared][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Exclusive][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Modified][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
        transitionTable[Owned][Packet::WriteInvalidateReq].onSnoop(invalidateTrans);
    }

    else {
        cerr << "CoherenceProtocol: unrecognized protocol " << protocol
             <<  endl;
        fatal("");
    }
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
