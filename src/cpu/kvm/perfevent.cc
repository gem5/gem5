/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Sandberg
 */

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <fcntl.h>
#include <syscall.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstring>

#include "base/misc.hh"
#include "perfevent.hh"

PerfKvmCounterConfig::PerfKvmCounterConfig(uint32_t type, uint64_t config)
{
    memset(&attr, 0, sizeof(attr));

    attr.size = PERF_ATTR_SIZE_VER0;
    attr.type = type;
    attr.config = config;
}

PerfKvmCounterConfig::~PerfKvmCounterConfig()
{
}


PerfKvmCounter::PerfKvmCounter(PerfKvmCounterConfig &config, pid_t tid)
    : fd(-1), ringBuffer(NULL), pageSize(-1)
{
    attach(config, tid, -1);
}

PerfKvmCounter::PerfKvmCounter(PerfKvmCounterConfig &config,
                         pid_t tid, const PerfKvmCounter &parent)
    : fd(-1), ringBuffer(NULL), pageSize(-1)
{
    attach(config, tid, parent);
}

PerfKvmCounter::PerfKvmCounter()
    : fd(-1), ringBuffer(NULL), pageSize(-1)
{
}

PerfKvmCounter::~PerfKvmCounter()
{
    if (attached())
        detach();
}

void
PerfKvmCounter::detach()
{
    assert(attached());

    if (munmap(ringBuffer, ringNumPages * pageSize) == -1)
        warn("PerfKvmCounter: Failed to unmap ring buffer (%i)\n",
             errno);
    close(fd);

    fd = -1;
    ringBuffer = NULL;
}

void
PerfKvmCounter::start()
{
    if (ioctl(PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP) == -1)
        panic("KVM: Failed to enable performance counters (%i)\n", errno);
}

void
PerfKvmCounter::stop()
{
    if (ioctl(PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP) == -1)
        panic("KVM: Failed to disable performance counters (%i)\n", errno);
}

void
PerfKvmCounter::period(uint64_t period)
{
    if (ioctl(PERF_EVENT_IOC_PERIOD, &period) == -1)
        panic("KVM: Failed to set period of performance counter (%i)\n", errno);
}

void
PerfKvmCounter::refresh(int refresh)
{
    if (ioctl(PERF_EVENT_IOC_REFRESH, refresh) == -1)
        panic("KVM: Failed to refresh performance counter (%i)\n", errno);
}

uint64_t
PerfKvmCounter::read() const
{
    uint64_t value;

    read(&value, sizeof(uint64_t));
    return value;
}

void
PerfKvmCounter::enableSignals(pid_t tid, int signal)
{
    struct f_owner_ex sigowner;

    sigowner.type = F_OWNER_TID;
    sigowner.pid = tid;

    if (fcntl(F_SETOWN_EX, &sigowner) == -1 ||
        fcntl(F_SETSIG, signal) == -1 ||
        fcntl(F_SETFL, O_ASYNC) == -1)
        panic("PerfKvmCounter: Failed to enable signals for counter (%i)\n",
              errno);
}

void
PerfKvmCounter::attach(PerfKvmCounterConfig &config,
                    pid_t tid, int group_fd)
{
    assert(!attached());

    fd = syscall(__NR_perf_event_open,
                 &config.attr, tid,
                 -1, // CPU (-1 => Any CPU that the task happens to run on)
                 group_fd,
                 0); // Flags
    if (fd == -1)
        panic("PerfKvmCounter::open failed (%i)\n", errno);

    mmapPerf(1);
}

pid_t
PerfKvmCounter::gettid()
{
    return syscall(__NR_gettid);
}

void
PerfKvmCounter::mmapPerf(int pages)
{
    assert(attached());
    assert(ringBuffer == NULL);

    if (pageSize == -1) {
        pageSize = sysconf(_SC_PAGE_SIZE);
        if (pageSize == -1)
            panic("PerfKvmCounter: Failed to determine page size (%i)\n",
                  errno);
    }

    ringNumPages = pages + 1;
    ringBuffer = (struct perf_event_mmap_page *)mmap(
        NULL, ringNumPages * 4096,
        PROT_READ | PROT_WRITE, MAP_SHARED,
        fd, 0);
    if (ringBuffer == MAP_FAILED)
        panic("PerfKvmCounter: MMAP failed (%i)\n",
              errno);
}

int
PerfKvmCounter::fcntl(int cmd, long p1)
{
    assert(attached());
    return ::fcntl(fd, cmd, p1);
}

int
PerfKvmCounter::ioctl(int request, long p1)
{
    assert(attached());
    return ::ioctl(fd, request, p1);
}

void
PerfKvmCounter::read(void *buf, size_t size) const
{
    char *_buf = (char *)buf;
    size_t _size = size;

    assert(attached());

    do {
        ssize_t ret;
        ret = ::read(fd, _buf, _size);
        switch (ret) {
          case -1:
            if (errno != EAGAIN)
                panic("PerfKvmCounter::read failed (%i)\n", errno);
            break;

          case 0:
            panic("PerfKvmCounter::read unexpected EOF.\n");

          default:
            _size -= ret;
            _buf += ret;
            break;
        }
    } while(_size);
}
