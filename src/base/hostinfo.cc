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
 */

#include <unistd.h>

#ifdef __APPLE__
#include <mach/mach_init.h>
#include <mach/shared_region.h>
#include <mach/task.h>
#endif

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/str.hh"
#include "base/types.hh"

using namespace std;

string
__get_hostname()
{
    char host[256];
    if (gethostname(host, sizeof host) == -1)
        warn("could not get host name!");
    return host;
}

string &
hostname()
{
    static string hostname = __get_hostname();
    return hostname;
}

uint64_t
procInfo(const char *filename, const char *target)
{
    int  done = 0;
    char line[80];
    char format[80];
    long usage;

    FILE *fp = fopen(filename, "r");

    while (fp && !feof(fp) && !done) {
        if (fgets(line, 80, fp)) {
            if (startswith(line, target)) {
                snprintf(format, sizeof(format), "%s %%ld", target);
                sscanf(line, format, &usage);

                fclose(fp);
                return usage ;
            }
        }
    }

    if (fp)
        fclose(fp);

    return 0;
}

uint64_t
memUsage()
{
// For the Mach-based Darwin kernel, use the task_info of the self task
#ifdef __APPLE__
    struct task_basic_info t_info;
    mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;

    if (KERN_SUCCESS != task_info(mach_task_self(),
                                  TASK_BASIC_INFO, (task_info_t)&t_info,
                                  &t_info_count)) {
        return 0;
    }

    // Mimic Darwin's implementation of top and subtract
    // SHARED_REGION_SIZE from the tasks virtual size to account for the
    // shared memory submap that is incorporated into every process.
    return (t_info.virtual_size - SHARED_REGION_SIZE) / 1024;
#else
    // Linux implementation
    return procInfo("/proc/self/status", "VmSize:");
#endif
}
