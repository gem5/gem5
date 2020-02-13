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
 */

#include "kern/linux/linux.hh"

#include <cstdio>
#include <string>

#include "cpu/base.hh"
#include "debug/SyscallVerbose.hh"
#include "sim/mem_state.hh"
#include "sim/process.hh"
#include "sim/system.hh"
#include "sim/vma.hh"

// The OS methods are called statically. Instantiate the random number
// generator for access to /dev/urandom here.
Random Linux::random;

int
Linux::openSpecialFile(std::string path, Process *process,
                       ThreadContext *tc)
{
    DPRINTFR(SyscallVerbose,
             "%d: %s: generic-open: opening special file: %s\n",
             curTick(), tc->getCpuPtr()->name(), path.c_str());

    bool matched = false;
    std::string data;

    if (path.compare(0, 13, "/proc/meminfo") == 0) {
        data = Linux::procMeminfo(process, tc);
        matched = true;
    } else if (path.compare(0, 11, "/etc/passwd") == 0) {
        data = Linux::etcPasswd(process, tc);
        matched = true;
    } else if (path.compare(0, 15, "/proc/self/maps") == 0) {
        data = Linux::procSelfMaps(process, tc);
        matched = true;
    } else if (path.compare(0, 30, "/sys/devices/system/cpu/online") == 0) {
        data = Linux::cpuOnline(process, tc);
        matched = true;
    } else if (path.compare(0, 12 ,"/dev/urandom") == 0) {
        data = Linux::devRandom(process, tc);
        matched = true;
    }

    if (matched) {
        FILE *f = tmpfile();
        int fd = fileno(f);
        size_t ret M5_VAR_USED = fwrite(data.c_str(), 1, data.size(), f);
        assert(ret == data.size());
        rewind(f);
        return fd;
    } else {
        warn("Attempting to open special file: %s. Ignoring. Simulation may "
             "take un-expected code path or be non-deterministic until proper "
             "handling is implemented.\n", path.c_str());
        errno = EACCES;
        return -1;
    }
}

std::string
Linux::procMeminfo(Process *process, ThreadContext *tc)
{
    return csprintf("MemTotal:%12d kB\nMemFree: %12d kB\n",
            process->system->memSize() >> 10,
            process->system->freeMemSize() >> 10);
}

std::string
Linux::etcPasswd(Process *process, ThreadContext *tc)
{
    return csprintf("gem5-user:x:1000:1000:gem5-user,,,:%s:/bin/bash\n",
                    process->tgtCwd);
}

std::string
Linux::procSelfMaps(Process *process, ThreadContext *tc)
{
    return process->memState->printVmaList();
}

std::string
Linux::cpuOnline(Process *process, ThreadContext *tc)
{
    return csprintf("0-%d\n",
                    tc->getSystemPtr()->numContexts() - 1);
}

std::string
Linux::devRandom(Process *process, ThreadContext *tc)
{
    DPRINTFR(SyscallVerbose,
             "%d: %s: open: generating urandom\n",
             curTick(), tc->getCpuPtr()->name());

    std::stringstream line;
    int max = 1E5;
    for (int i = 0; i < max; i++) {
        uint8_t rand_uint = random.random<uint8_t>(0, 255);

        line << rand_uint;
    }
    return line.str();
}
