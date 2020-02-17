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
 */

#ifndef __ARCH_X86_APICREGS_HH__
#define __ARCH_X86_APICREGS_HH__

#include "base/bitunion.hh"

namespace X86ISA
{
    enum ApicRegIndex
    {
        APIC_ID,
        APIC_VERSION,
        APIC_TASK_PRIORITY,
        APIC_ARBITRATION_PRIORITY,
        APIC_PROCESSOR_PRIORITY,
        APIC_EOI,
        APIC_LOGICAL_DESTINATION,
        APIC_DESTINATION_FORMAT,
        APIC_SPURIOUS_INTERRUPT_VECTOR,

        APIC_IN_SERVICE_BASE,

        APIC_TRIGGER_MODE_BASE = APIC_IN_SERVICE_BASE + 16,

        APIC_INTERRUPT_REQUEST_BASE = APIC_TRIGGER_MODE_BASE + 16,

        APIC_ERROR_STATUS = APIC_INTERRUPT_REQUEST_BASE + 16,
        APIC_INTERRUPT_COMMAND_LOW,
        APIC_INTERRUPT_COMMAND_HIGH,
        APIC_LVT_TIMER,
        APIC_LVT_THERMAL_SENSOR,
        APIC_LVT_PERFORMANCE_MONITORING_COUNTERS,
        APIC_LVT_LINT0,
        APIC_LVT_LINT1,
        APIC_LVT_ERROR,
        APIC_INITIAL_COUNT,
        APIC_CURRENT_COUNT,
        APIC_DIVIDE_CONFIGURATION,

        APIC_INTERNAL_STATE,

        NUM_APIC_REGS
    };

    static inline ApicRegIndex
    APIC_IN_SERVICE(int index)
    {
        return (ApicRegIndex)(APIC_IN_SERVICE_BASE + index);
    }

    static inline ApicRegIndex
    APIC_TRIGGER_MODE(int index)
    {
        return (ApicRegIndex)(APIC_TRIGGER_MODE_BASE + index);
    }

    static inline ApicRegIndex
    APIC_INTERRUPT_REQUEST(int index)
    {
        return (ApicRegIndex)(APIC_INTERRUPT_REQUEST_BASE + index);
    }

    BitUnion32(InterruptCommandRegLow)
        Bitfield<7, 0> vector;
        Bitfield<10, 8> deliveryMode;
        Bitfield<11> destMode;
        Bitfield<12> deliveryStatus;
        Bitfield<14> level;
        Bitfield<15> trigger;
        Bitfield<19, 18> destShorthand;
    EndBitUnion(InterruptCommandRegLow)

    BitUnion32(InterruptCommandRegHigh)
        Bitfield<31, 24> destination;
    EndBitUnion(InterruptCommandRegHigh)
}

#endif
