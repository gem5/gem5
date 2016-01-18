/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __ARCH_ALPHA_TYPES_HH__
#define __ARCH_ALPHA_TYPES_HH__

#include "arch/generic/types.hh"
#include "base/types.hh"

namespace AlphaISA {

typedef uint32_t MachInst;
typedef uint64_t ExtMachInst;

typedef GenericISA::SimplePCState<MachInst> PCState;

enum annotes
{
    ANNOTE_NONE = 0,
    // An impossible number for instruction annotations
    ITOUCH_ANNOTE = 0xffffffff
};

/**
 * Alpha-specific memory request flags
 *
 * These flags map to the architecture-specific lower 8 bits of the
 * flags field in Request.
 */
struct AlphaRequestFlags
{
    typedef uint8_t ArchFlagsType;

    /** The request is an ALPHA VPTE pal access (hw_ld). */
    static const ArchFlagsType VPTE    = 0x01;
    /** Use the alternate mode bits in ALPHA. */
    static const ArchFlagsType ALTMODE = 0x02;
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_TYPES_HH__
