/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Nathan Binkert
 */

#include "arch/isa_traits.hh"
#include "base/bigint.hh"
#include "config/the_isa.hh"
#include "mem/packet.hh"
#include "sim/byteswap.hh"

#ifndef __MEM_PACKET_ACCESS_HH__
#define __MEM_PACKET_ACCESS_HH__
// The memory system needs to have an endianness. This is the easiest
// way to deal with it for now. At some point, we will have to remove
// these functions and make the users do their own byte swapping since
// the memory system does not in fact have an endianness.

/** return the value of what is pointed to in the packet. */
template <typename T>
inline T
Packet::get()
{
    assert(flags.isSet(STATIC_DATA|DYNAMIC_DATA));
    assert(sizeof(T) <= size);
    return TheISA::gtoh(*(T*)data);
}

/** set the value in the data pointer to v. */
template <typename T>
inline void
Packet::set(T v)
{
    assert(flags.isSet(STATIC_DATA|DYNAMIC_DATA));
    assert(sizeof(T) <= size);
    *(T*)data = TheISA::htog(v);
}

#endif //__MEM_PACKET_ACCESS_HH__
