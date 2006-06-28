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
 */

/**
 * @file
 * Declaration of CoherenceProcotol a basic coherence policy.
 */
#ifndef __COHERENCE_PROTOCOL_HH__
#define __COHERENCE_PROTOCOL_HH__

#include <string>

#include "sim/sim_object.hh"
#include "mem/packet.hh"
#include "mem/mem_cmd.hh"
#include "mem/cache/cache_blk.hh"
#include "base/statistics.hh"

class BaseCache;
class MSHR;

/**
 * A simple coherence policy for the memory hierarchy. Currently implements
 * MSI, MESI, and MOESI protocols.
 */
class CoherenceProtocol : public SimObject
{
  public:
    /**
     * Contruct and initialize this policy.
     * @param name The name of this policy.
     * @param protocol The string representation of the protocol to use.
     * @param doUpgrades True if bus upgrades should be used.
     */
    CoherenceProtocol(const std::string &name, const std::string &protocol,
                      const bool doUpgrades);

    /**
     * Destructor.
     */
    virtual ~CoherenceProtocol() {};

    /**
     * Register statistics
     */
    virtual void regStats();

    /**
     * Get the proper bus command for the given command and status.
     * @param cmd The request's command.
     * @param status The current state of the cache block.
     * @param mshr The MSHR matching the request.
     * @return The proper bus command, as determined by the protocol.
     */
    Packet::Command getBusCmd(Packet::Command cmd, CacheBlk::State status,
                     MSHR *mshr = NULL);

    /**
     * Return the proper state given the current state and the bus response.
     * @param req The bus response.
     * @param oldState The current block state.
     * @return The new state.
     */
    CacheBlk::State getNewState(const Packet * &pkt,
                                CacheBlk::State oldState);

    /**
     * Handle snooped bus requests.
     * @param cache The cache that snooped the request.
     * @param req The snooped bus request.
     * @param blk The cache block corresponding to the request, if any.
     * @param mshr The MSHR corresponding to the request, if any.
     * @param new_state The new coherence state of the block.
     * @return True if the request should be satisfied locally.
     */
    bool handleBusRequest(BaseCache *cache, Packet * &pkt, CacheBlk *blk,
                          MSHR *mshr, CacheBlk::State &new_state);

  protected:
    /** Snoop function type. */
    typedef bool (*SnoopFuncType)(BaseCache *, Packet *&, CacheBlk *,
                                  MSHR *, CacheBlk::State&);

    //
    // Standard snoop transition functions
    //

    /**
     * Do nothing transition.
     */
    static bool nullTransition(BaseCache *, Packet *&, CacheBlk *,
                               MSHR *, CacheBlk::State&);

    /**
     * Invalid transition, basically panic.
     */
    static bool invalidTransition(BaseCache *, Packet *&, CacheBlk *,
                                  MSHR *, CacheBlk::State&);

    /**
     * Invalidate block, move to Invalid state.
     */
    static bool invalidateTrans(BaseCache *, Packet *&, CacheBlk *,
                                MSHR *, CacheBlk::State&);

    /**
     * Supply data, no state transition.
     */
    static bool supplyTrans(BaseCache *, Packet *&, CacheBlk *,
                            MSHR *, CacheBlk::State&);

    /**
     * Supply data and go to Shared state.
     */
    static bool supplyAndGotoSharedTrans(BaseCache *, Packet *&, CacheBlk *,
                                         MSHR *, CacheBlk::State&);

    /**
     * Supply data and go to Owned state.
     */
    static bool supplyAndGotoOwnedTrans(BaseCache *, Packet *&, CacheBlk *,
                                        MSHR *, CacheBlk::State&);

    /**
     * Invalidate block, supply data, and go to Invalid state.
     */
    static bool supplyAndInvalidateTrans(BaseCache *, Packet *&, CacheBlk *,
                                         MSHR *, CacheBlk::State&);

    /**
     * Assert the shared line for a block that is shared/exclusive.
     */
    static bool assertShared(BaseCache *, Packet *&, CacheBlk *,
                                         MSHR *, CacheBlk::State&);

    /**
     * Definition of protocol state transitions.
     */
    class StateTransition
    {
        friend class CoherenceProtocol;

        /** The bus command of this transition. */
        Packet::Command busCmd;
        /** The state to transition to. */
        int newState;
        /** The snoop function for this transition. */
        SnoopFuncType snoopFunc;

        /**
         * Constructor, defaults to invalid transition.
         */
        StateTransition();

        /**
         * Initialize bus command.
         * @param cmd The bus command to use.
         */
        void onRequest(Packet::Command cmd)
        {
            busCmd = cmd;
        }

        /**
         * Set the transition state.
         * @param s The new state.
         */
        void onResponse(CacheBlk::State s)
        {
            newState = s;
        }

        /**
         * Initialize the snoop function.
         * @param f The new snoop function.
         */
        void onSnoop(SnoopFuncType f)
        {
            snoopFunc = f;
        }
    };

    friend class CoherenceProtocol::StateTransition;

    /** Mask to select status bits relevant to coherence protocol. */
    const static CacheBlk::State
        stateMask = BlkValid | BlkWritable | BlkDirty;

    /** The Modified (M) state. */
    const static CacheBlk::State
        Modified = BlkValid | BlkWritable | BlkDirty;
    /** The Owned (O) state. */
    const static CacheBlk::State
        Owned = BlkValid | BlkDirty;
    /** The Exclusive (E) state. */
    const static CacheBlk::State
        Exclusive = BlkValid | BlkWritable;
    /** The Shared (S) state. */
    const static CacheBlk::State
        Shared = BlkValid;
    /** The Invalid (I) state. */
    const static CacheBlk::State
        Invalid = 0;

    /**
     * Maximum state encoding value (used to size transition lookup
     * table).  Could be more than number of states, depends on
     * encoding of status bits.
     */
    const static int stateMax = stateMask;

    /**
     * The table of all possible transitions, organized by starting state and
     * request command.
     */
    StateTransition transitionTable[stateMax+1][NUM_MEM_CMDS];

    /**
     * @addtogroup CoherenceStatistics
     * @{
     */
    /**
     * State accesses from parent cache.
     */
    Stats::Scalar<> requestCount[stateMax+1][NUM_MEM_CMDS];
    /**
     * State accesses from snooped requests.
     */
    Stats::Scalar<> snoopCount[stateMax+1][NUM_MEM_CMDS];
    /**
     * @}
     */
};

#endif // __COHERENCE_PROTOCOL_HH__
