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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Ali Saidi
 */

#include <string>

#include "arch/sparc/vtophys.hh"
#include "base/chunk_generator.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"

using namespace std;

namespace SparcISA
{
    PageTableEntry kernel_pte_lookup(FunctionalPort *mem,
            Addr ptbr, VAddr vaddr)
    {
        PageTableEntry pte(4);
        return pte;
    }

    Addr vtophys(Addr vaddr)
    {
        return vaddr;
    }

    Addr vtophys(ThreadContext *tc, Addr addr)
    {
        return addr;
    }


    void CopyOut(ThreadContext *tc, void *dest, Addr src, size_t cplen)
    {
    }

    void CopyIn(ThreadContext *tc, Addr dest, void *source, size_t cplen)
    {
    }

    void CopyStringOut(ThreadContext *tc, char *dst, Addr vaddr, size_t maxlen)
    {
    }

    void CopyStringIn(ThreadContext *tc, char *src, Addr vaddr)
    {
    }
}
