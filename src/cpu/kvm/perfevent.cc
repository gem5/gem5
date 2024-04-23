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
 */

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <syscall.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstring>

#include "base/logging.hh"
#include "perfevent.hh"

namespace gem5
{

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

PerfKvmCounter::PerfKvmCounter()
{
}

PerfKvmCounter *
PerfKvmCounter::create()
{
    // Check if we're running on a hybrid host architecture. Linux exposes
    // this via sysfs. If the directory /sys/devices/cpu exists, then we are
    // running on a regular architecture. Otherwise, /sys/devices/cpu_core and
    // /sys/devices/cpu_atom should exist. For simplicity, we use the
    // existence of /sys/devices/cpu_atom to indicate a hybrid host
    // architecture.
    const char *atom_path = "/sys/devices/cpu_atom";
    if (DIR *atom_dir = opendir(atom_path)) {
        closedir(atom_dir);

        // Since we're running on a hybrid architecture, use a hybrid
        // performance counter. This uses two 'physical' performance counters
        // to implement a 'logical' one which is the sum of the two.
        return new HybridPerfKvmCounter();
    } else {
        if (errno != ENOENT)
            warn("Unexpected error code from opendir(%s): %s\n",
                 atom_path, std::strerror(errno));

        // We're running on a regular architecture, so use a regular
        // performance counter.
        return new SimplePerfKvmCounter();
    }
}

SimplePerfKvmCounter::SimplePerfKvmCounter()
    : fd(-1), ringBuffer(NULL), pageSize(-1)
{
}

SimplePerfKvmCounter::~SimplePerfKvmCounter()
{
    if (attached())
        detach();
}

void
SimplePerfKvmCounter::detach()
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
SimplePerfKvmCounter::start()
{
    if (ioctl(PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP) == -1)
        panic("KVM: Failed to enable performance counters (%i)\n", errno);
}

void
SimplePerfKvmCounter::stop()
{
    if (ioctl(PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP) == -1)
        panic("KVM: Failed to disable performance counters (%i)\n", errno);
}

void
SimplePerfKvmCounter::period(uint64_t period)
{
    if (ioctl(PERF_EVENT_IOC_PERIOD, &period) == -1)
        panic("KVM: Failed to set period of performance counter (%i)\n", errno);
}

void
SimplePerfKvmCounter::refresh(int refresh)
{
    if (ioctl(PERF_EVENT_IOC_REFRESH, refresh) == -1)
        panic("KVM: Failed to refresh performance counter (%i)\n", errno);
}

uint64_t
SimplePerfKvmCounter::read() const
{
    uint64_t value;

    read(&value, sizeof(uint64_t));
    return value;
}

void
SimplePerfKvmCounter::enableSignals(pid_t tid, int signal)
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
SimplePerfKvmCounter::attach(PerfKvmCounterConfig &config,
                             pid_t tid, const PerfKvmCounter *parent)
{
    assert(!attached());

    int group_fd = -1;
    if (parent)
        group_fd = dynamic_cast<const SimplePerfKvmCounter &>(*parent).fd;

    fd = syscall(__NR_perf_event_open,
                 &config.attr, tid,
                 -1, // CPU (-1 => Any CPU that the task happens to run on)
                 group_fd,
                 0); // Flags
    if (fd == -1)
    {
        if (errno == EACCES)
        {
            panic("PerfKvmCounter::attach received error EACCESS.\n"
            "  This error may be caused by a too restrictive setting\n"
            "  in the file '/proc/sys/kernel/perf_event_paranoid'.\n"
            "  The default value was changed to 2 in kernel 4.6.\n"
            "  A value greater than 1 prevents gem5 from making\n"
            "  the syscall to perf_event_open.\n"
            "    Alternatively, you can set the usePerf flag of the KVM\n"
            "  CPU to False. Setting this flag to False will limit some\n"
            "  functionalities of KVM CPU, such as counting the number of\n"
            "  cycles and the number of instructions, as well as the\n"
            "  ability of exiting to gem5 after a certain amount of cycles\n"
            "  or instructions when using KVM CPU. An example can be found\n"
            "  here, configs/example/gem5_library/"
            "x86-ubuntu-run-with-kvm-no-perf.py.");
        }
        panic("PerfKvmCounter::attach failed (%i)\n", errno);
    }

    mmapPerf(1);
}

pid_t
PerfKvmCounter::sysGettid()
{
    return syscall(__NR_gettid);
}

void
SimplePerfKvmCounter::mmapPerf(int pages)
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
SimplePerfKvmCounter::fcntl(int cmd, long p1)
{
    assert(attached());
    return ::fcntl(fd, cmd, p1);
}

int
SimplePerfKvmCounter::ioctl(int request, long p1)
{
    assert(attached());
    return ::ioctl(fd, request, p1);
}

void
SimplePerfKvmCounter::read(void *buf, size_t size) const
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
    } while (_size);
}

void
HybridPerfKvmCounter::attach(PerfKvmCounterConfig &config, pid_t tid,
                             const PerfKvmCounter *parent)
{
    // We should only be using hybrid performance counters for hardware
    // events.
    assert(config.attr.type == PERF_TYPE_HARDWARE);

    const SimplePerfKvmCounter *parent_core_counter = nullptr;
    const SimplePerfKvmCounter *parent_atom_counter = nullptr;
    if (parent) {
        const HybridPerfKvmCounter &hybrid_parent =
            dynamic_cast<const HybridPerfKvmCounter &>(*parent);
        parent_core_counter = &hybrid_parent.coreCounter;
        parent_atom_counter = &hybrid_parent.atomCounter;
    }

    PerfKvmCounterConfig config_core = config;
    config_core.attr.config |= ConfigCore;
    coreCounter.attach(config_core, tid, parent_core_counter);

    PerfKvmCounterConfig config_atom = config;
    config_atom.attr.config |= ConfigAtom;
    atomCounter.attach(config_atom, tid, parent_atom_counter);
}

void
HybridPerfKvmCounter::detach()
{
    coreCounter.detach();
    atomCounter.detach();
}

bool
HybridPerfKvmCounter::attached() const
{
    assert(coreCounter.attached() == atomCounter.attached());
    return coreCounter.attached();
}

void
HybridPerfKvmCounter::start()
{
    coreCounter.start();
    atomCounter.start();
}

void
HybridPerfKvmCounter::stop()
{
    coreCounter.stop();
    atomCounter.stop();
}

void
HybridPerfKvmCounter::period(uint64_t period)
{
    coreCounter.period(period);
    atomCounter.period(period);
}

void
HybridPerfKvmCounter::refresh(int refresh)
{
    coreCounter.refresh(refresh);
    atomCounter.refresh(refresh);
}

uint64_t
HybridPerfKvmCounter::read() const
{
    // To get the logical counter value, we simply sum the individual physical
    // counter values.
    return coreCounter.read() + atomCounter.read();
}

void
HybridPerfKvmCounter::enableSignals(pid_t tid, int signal)
{
    coreCounter.enableSignals(tid, signal);
    atomCounter.enableSignals(tid, signal);
}

} // namespace gem5
