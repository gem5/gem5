/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <sys/types.h>
#include <algorithm>

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "kern/tru64/mbuf.hh"
#include "sim/host.hh"
#include "targetarch/arguments.hh"
#include "targetarch/isa_traits.hh"
#include "targetarch/vtophys.hh"

namespace tru64 {

void
DumpMbuf(AlphaArguments args)
{
    ExecContext *xc = args.getExecContext();
    Addr addr = (Addr)args;
    struct mbuf m;

    CopyOut(xc, &m, addr, sizeof(m));

    int count = m.m_pkthdr.len;

    ccprintf(DebugOut(), "m=%#lx, m->m_pkthdr.len=%#d\n", addr,
             m.m_pkthdr.len);

    while (count > 0) {
        ccprintf(DebugOut(), "m=%#lx, m->m_data=%#lx, m->m_len=%d\n",
                 addr, m.m_data, m.m_len);
        char *buffer = new char[m.m_len];
        CopyOut(xc, buffer, m.m_data, m.m_len);
        Trace::rawDump((uint8_t *)buffer, m.m_len);
        delete [] buffer;

        count -= m.m_len;
        if (!m.m_next)
            break;

        CopyOut(xc, &m, m.m_next, sizeof(m));
    }
}

} // namespace Tru64
