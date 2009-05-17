/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

class ThreadContext;

namespace X86ISA
{
    class Walker : public MemObject
    {
      public:
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

        // Act on the current state and determine what to do next. The global
        // read should be the packet that just came back from a read and write
        // should be NULL. When the function returns, read is either NULL
        // if the machine is finished, or points to a packet to initiate
        // the next read. If any write is required to update an "accessed"
        // bit, write will point to a packet to do the write. Otherwise it
        // will be NULL. The return value is whatever fault was incurred
        // during this stage of the lookup.
        Fault doNext(PacketPtr &write);

        // Kick off the state machine.
        Fault start(ThreadContext * _tc, BaseTLB::Translation *translation,
                RequestPtr req, BaseTLB::Mode mode);
        // Clean up after the state machine.
        void
        stop()
        {
            nextState = Ready;
            delete read->req;
            delete read;
            read = NULL;
        }

      protected:

        /*
         * State having to do with sending packets.
         */
        PacketPtr read;
        std::vector<PacketPtr> writes;

        // How many memory operations are in flight.
        unsigned inflight;

        bool retrying;

        /*
         * The fault, if any, that's waiting to be delivered in timing mode.
         */
        Fault timingFault;

        /*
         * Functions for dealing with packets.
         */
        bool recvTiming(PacketPtr pkt);
        void recvRetry();

        void sendPackets();

        /*
         * Port for accessing memory
         */
        class WalkerPort : public Port
        {
          public:
            WalkerPort(const std::string &_name, Walker * _walker) :
                  Port(_name, _walker), walker(_walker),
                  snoopRangeSent(false)
            {}

          protected:
            Walker * walker;

            bool snoopRangeSent;

            bool recvTiming(PacketPtr pkt);
            Tick recvAtomic(PacketPtr pkt);
            void recvFunctional(PacketPtr pkt);
            void recvStatusChange(Status status);
            void recvRetry();
            void getDeviceAddressRanges(AddrRangeList &resp,
                    bool &snoop)
            {
                resp.clear();
                snoop = true;
            }
        };

        Port *getPort(const std::string &if_name, int idx = -1);

        friend class WalkerPort;

        WalkerPort port;

        // The TLB we're supposed to load.
        TLB * tlb;
        System * sys;
        BaseTLB::Translation * translation;

        /*
         * State machine state.
         */
        ThreadContext * tc;
        RequestPtr req;
        State state;
        State nextState;
        int size;
        bool enableNX;
        BaseTLB::Mode mode;
        bool user;
        TlbEntry entry;
        
        Fault pageFault(bool present);

      public:

        void setTLB(TLB * _tlb)
        {
            tlb = _tlb;
        }

        typedef X86PagetableWalkerParams Params;

        Walker(const Params *params) :
            MemObject(params),
            read(NULL), inflight(0), retrying(false),
            port(name() + ".port", this),
            tlb(NULL), sys(params->system),
            tc(NULL), state(Ready), nextState(Ready)
        {
        }
    };
}
#endif // __ARCH_X86_PAGE_TABLE_WALKER_HH__
