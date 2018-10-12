/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_INTMESSAGE_HH__
#define __ARCH_X86_INTMESSAGE_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace X86ISA
{
    BitUnion32(TriggerIntMessage)
        Bitfield<7, 0> destination;
        Bitfield<15, 8> vector;
        Bitfield<18, 16> deliveryMode;
        Bitfield<19> destMode;
        Bitfield<20> level;
        Bitfield<21> trigger;
    EndBitUnion(TriggerIntMessage)

    namespace DeliveryMode
    {
        enum IntDeliveryMode {
            Fixed = 0,
            LowestPriority = 1,
            SMI = 2,
            NMI = 4,
            INIT = 5,
            SIPI = 6,
            ExtInt = 7,
            NumModes
        };

        static const char * const names[NumModes] = {
            "Fixed", "LowestPriority", "SMI", "Reserved",
            "NMI", "INIT", "Startup", "ExtInt"
        };

        static inline bool
        isReserved(int mode)
        {
            return mode == 3;
        }
    }

    static const Addr TriggerIntOffset = 0;

    static inline PacketPtr
    prepIntRequest(const uint8_t id, Addr offset, Addr size)
    {
        RequestPtr req = std::make_shared<Request>(
            x86InterruptAddress(id, offset),
            size, Request::UNCACHEABLE,
            Request::intMasterId);

        PacketPtr pkt = new Packet(req, MemCmd::MessageReq);
        pkt->allocate();
        return pkt;
    }

    template<class T>
    PacketPtr
    buildIntRequest(const uint8_t id, T payload, Addr offset, Addr size)
    {
        PacketPtr pkt = prepIntRequest(id, offset, size);
        pkt->setRaw<T>(payload);
        return pkt;
    }

    static inline PacketPtr
    buildIntRequest(const uint8_t id, TriggerIntMessage payload)
    {
        return buildIntRequest(id, payload, TriggerIntOffset,
                sizeof(TriggerIntMessage));
    }

    static inline PacketPtr
    buildIntResponse()
    {
        panic("buildIntResponse not implemented.\n");
    }
}

#endif
