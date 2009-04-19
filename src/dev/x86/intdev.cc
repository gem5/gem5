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
X86ISA::IntDev::IntPort::sendMessage(TriggerIntMessage message, bool timing)
{
    if (DeliveryMode::isReserved(message.deliveryMode)) {
        fatal("Tried to use reserved delivery mode %d\n",
                message.deliveryMode);
    } else if (DTRACE(IntDev)) {
        DPRINTF(IntDev, "Delivery mode is: %s.\n",
                DeliveryMode::names[message.deliveryMode]);
        DPRINTF(IntDev, "Vector is %#x.\n", message.vector);
    }
    if (message.destMode == 0) {
        DPRINTF(IntDev,
                "Sending interrupt to APIC ID %d.\n", message.destination);
        PacketPtr pkt = buildIntRequest(message.destination, message);
        if (timing)
            sendMessageTiming(pkt, latency);
        else
            sendMessageAtomic(pkt);
    } else {
        DPRINTF(IntDev, "Sending interrupts to APIC IDs:"
                "%s%s%s%s%s%s%s%s\n",
                bits((int)message.destination, 0) ? " 0": "",
                bits((int)message.destination, 1) ? " 1": "",
                bits((int)message.destination, 2) ? " 2": "",
                bits((int)message.destination, 3) ? " 3": "",
                bits((int)message.destination, 4) ? " 4": "",
                bits((int)message.destination, 5) ? " 5": "",
                bits((int)message.destination, 6) ? " 6": "",
                bits((int)message.destination, 7) ? " 7": ""
                );
        uint8_t dests = message.destination;
        uint8_t id = 0;
        while(dests) {
            if (dests & 0x1) {
                PacketPtr pkt = buildIntRequest(id, message);
                if (timing)
                    sendMessageTiming(pkt, latency);
                else
                    sendMessageAtomic(pkt);
            }
            dests >>= 1;
            id++;
        }
    }
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
