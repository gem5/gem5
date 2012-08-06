/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 */

#include <cstdio>
#include <string>

#include "cpu/thread_context.hh"
#include "debug/SyscallVerbose.hh"
#include "kern/linux/linux.hh"
#include "sim/process.hh"
#include "sim/system.hh"

int
Linux::openSpecialFile(std::string path, LiveProcess *process,
                       ThreadContext *tc)
{
    DPRINTF(SyscallVerbose, "Opening special file: %s\n", path.c_str());
    if (path.compare(0, 13, "/proc/meminfo") == 0) {
        std::string data = Linux::procMeminfo(process, tc);
        FILE *f = tmpfile();
        int fd = fileno(f);
        size_t ret M5_VAR_USED = fwrite(data.c_str(), 1, data.size(), f);
        assert(ret == data.size());
        rewind(f);
        return fd;
    }

    warn("Attempting to open special file: %s. Ignoring. Simulation may"
            " take un-expected code path or be non-deterministic until proper"
            "  handling is implemented.\n", path.c_str());
    return -1;
}

std::string
Linux::procMeminfo(LiveProcess *process, ThreadContext *tc)
{
    return csprintf("MemTotal:%12d kB\nMemFree: %12d kB\n",
            process->system->memSize() >> 10,
            process->system->freeMemSize() >> 10);
}

