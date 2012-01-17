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

#include "dev/x86/intdev.hh"

void
X86ISA::IntDev::IntPort::sendMessage(ApicList apics,
        TriggerIntMessage message, bool timing)
{
    ApicList::iterator apicIt;
    for (apicIt = apics.begin(); apicIt != apics.end(); apicIt++) {
        PacketPtr pkt = buildIntRequest(*apicIt, message);
        if (timing) {
            sendMessageTiming(pkt, latency);
            // The target handles cleaning up the packet in timing mode.
        } else {
            sendMessageAtomic(pkt);
            delete pkt->req;
            delete pkt;
        }
    }
}

void
X86ISA::IntDev::init()
{
    if (!intPort) {
        panic("Int port not connected to anything!");
    }
    intPort->sendRangeChange();
}

X86ISA::IntSourcePin *
X86IntSourcePinParams::create()
{
    return new X86ISA::IntSourcePin(this);
}

X86ISA::IntSinkPin *
X86IntSinkPinParams::create()
{
    return new X86ISA::IntSinkPin(this);
}

X86ISA::IntLine *
X86IntLineParams::create()
{
    return new X86ISA::IntLine(this);
}
