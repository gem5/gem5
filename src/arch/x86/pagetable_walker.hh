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
