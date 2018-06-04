/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_PAGE_TABLE_WALKER_HH__
#define __ARCH_X86_PAGE_TABLE_WALKER_HH__

#include <vector>

#include "arch/x86/pagetable.hh"
#include "arch/x86/tlb.hh"
#include "base/types.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "params/X86PagetableWalker.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

class ThreadContext;

namespace X86ISA
{
    class Walker : public MemObject
    {
      protected:
        // Port for accessing memory
        class WalkerPort : public MasterPort
        {
          public:
            WalkerPort(const std::string &_name, Walker * _walker) :
                  MasterPort(_name, _walker), walker(_walker)
            {}

          protected:
            Walker *walker;

            bool recvTimingResp(PacketPtr pkt);
            void recvReqRetry();
        };

        friend class WalkerPort;
        WalkerPort port;

        // State to track each walk of the page table
        class WalkerState
        {
          friend class Walker;
          private:
            enum State {
                Ready,
                Waiting,
                // Long mode
                LongPML4, LongPDP, LongPD, LongPTE,
                // PAE legacy mode
                PAEPDP, PAEPD, PAEPTE,
                // Non PAE legacy mode with and without PSE
                PSEPD, PD, PTE
            };

          protected:
            Walker *walker;
            ThreadContext *tc;
            RequestPtr req;
            State state;
            State nextState;
            int dataSize;
            bool enableNX;
            unsigned inflight;
            TlbEntry entry;
            PacketPtr read;
            std::vector<PacketPtr> writes;
            Fault timingFault;
            TLB::Translation * translation;
            BaseTLB::Mode mode;
            bool functional;
            bool timing;
            bool retrying;
            bool started;
          public:
            WalkerState(Walker * _walker, BaseTLB::Translation *_translation,
                        const RequestPtr &_req, bool _isFunctional = false) :
                walker(_walker), req(_req), state(Ready),
                nextState(Ready), inflight(0),
                translation(_translation),
                functional(_isFunctional), timing(false),
                retrying(false), started(false)
            {
            }
            void initState(ThreadContext * _tc, BaseTLB::Mode _mode,
                           bool _isTiming = false);
            Fault startWalk();
            Fault startFunctional(Addr &addr, unsigned &logBytes);
            bool recvPacket(PacketPtr pkt);
            bool isRetrying();
            bool wasStarted();
            bool isTiming();
            void retry();
            std::string name() const {return walker->name();}

          private:
            void setupWalk(Addr vaddr);
            Fault stepWalk(PacketPtr &write);
            void sendPackets();
            void endWalk();
            Fault pageFault(bool present);
        };

        friend class WalkerState;
        // State for timing and atomic accesses (need multiple per walker in
        // the case of multiple outstanding requests in timing mode)
        std::list<WalkerState *> currStates;
        // State for functional accesses (only need one of these per walker)
        WalkerState funcState;

        struct WalkerSenderState : public Packet::SenderState
        {
            WalkerState * senderWalk;
            WalkerSenderState(WalkerState * _senderWalk) :
                senderWalk(_senderWalk) {}
        };

      public:
        // Kick off the state machine.
        Fault start(ThreadContext * _tc, BaseTLB::Translation *translation,
                const RequestPtr &req, BaseTLB::Mode mode);
        Fault startFunctional(ThreadContext * _tc, Addr &addr,
                unsigned &logBytes, BaseTLB::Mode mode);
        BaseMasterPort &getMasterPort(const std::string &if_name,
                                      PortID idx = InvalidPortID);

      protected:
        // The TLB we're supposed to load.
        TLB * tlb;
        System * sys;
        MasterID masterId;

        // The number of outstanding walks that can be squashed per cycle.
        unsigned numSquashable;

        // Wrapper for checking for squashes before starting a translation.
        void startWalkWrapper();

        /**
         * Event used to call startWalkWrapper.
         **/
        EventFunctionWrapper startWalkWrapperEvent;

        // Functions for dealing with packets.
        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
        bool sendTiming(WalkerState * sendingState, PacketPtr pkt);

      public:

        void setTLB(TLB * _tlb)
        {
            tlb = _tlb;
        }

        typedef X86PagetableWalkerParams Params;

        const Params *
        params() const
        {
            return static_cast<const Params *>(_params);
        }

        Walker(const Params *params) :
            MemObject(params), port(name() + ".port", this),
            funcState(this, NULL, NULL, true), tlb(NULL), sys(params->system),
            masterId(sys->getMasterId(this)),
            numSquashable(params->num_squash_per_cycle),
            startWalkWrapperEvent([this]{ startWalkWrapper(); }, name())
        {
        }
    };
}
#endif // __ARCH_X86_PAGE_TABLE_WALKER_HH__
