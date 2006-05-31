/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 */

class ExecContext;

//We need the "Tick" data type from here
#include "sim/host.hh"
//We need the "Addr" data type from here
#include "arch/isa_traits.hh"

namespace AlphaPseudo
{
    /**
     * @todo these externs are only here for a hack in fullCPU::takeOver...
     */
    extern bool doStatisticsInsts;
    extern bool doCheckpointInsts;
    extern bool doQuiesce;

    void arm(ExecContext *xc);
    void quiesce(ExecContext *xc);
    void quiesceNs(ExecContext *xc, uint64_t ns);
    void quiesceCycles(ExecContext *xc, uint64_t cycles);
    uint64_t quiesceTime(ExecContext *xc);
    void ivlb(ExecContext *xc);
    void ivle(ExecContext *xc);
    void m5exit(ExecContext *xc, Tick delay);
    void m5exit_old(ExecContext *xc);
    void resetstats(ExecContext *xc, Tick delay, Tick period);
    void dumpstats(ExecContext *xc, Tick delay, Tick period);
    void dumpresetstats(ExecContext *xc, Tick delay, Tick period);
    void m5checkpoint(ExecContext *xc, Tick delay, Tick period);
    uint64_t readfile(ExecContext *xc, Addr vaddr, uint64_t len, uint64_t offset);
    void debugbreak(ExecContext *xc);
    void switchcpu(ExecContext *xc);
    void addsymbol(ExecContext *xc, Addr addr, Addr symbolAddr);
}
