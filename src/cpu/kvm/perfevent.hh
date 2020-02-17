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

#ifndef __CPU_KVM_PERFEVENT_HH__
#define __CPU_KVM_PERFEVENT_HH__

#include <linux/perf_event.h>
#include <sys/types.h>

#include <inttypes.h>

#include "config/have_perf_attr_exclude_host.hh"

/**
 * PerfEvent counter configuration.
 */
class PerfKvmCounterConfig
{
  public:
    /**
     * Initialize PerfEvent counter configuration structure
     *
     * PerfEvent has the concept of counter types, which is a way to
     * abstract hardware performance counters or access software
     * events. The type field in the configuration specifies what type
     * of counter this is. For hardware performance counters, it's
     * typically PERF_TYPE_HARDWARE, PERF_TYPE_HW_CACHE, or
     * PERF_TYPE_RAW.
     *
     * The 'config' field has different meanings depending on the type
     * of counter. Possible values are listed in perf_event.h in the
     * kernel headers. When using raw counters, the value is the raw
     * value written to the performance counter configuration register
     * (some bits dealing with sampling and similar features are
     * usually masked).
     *
     * @param type Counter type.
     * @param config Counter configuration
     */
    PerfKvmCounterConfig(uint32_t type, uint64_t config);
    ~PerfKvmCounterConfig();

    /**
     * Set the initial sample period (overflow count) of an event. If
     * this is set to 0, the event acts as a normal counting event and
     * does not trigger overflows.
     *
     * @param period Number of counter events before the counter
     * overflows
     */
    PerfKvmCounterConfig &samplePeriod(uint64_t period) {
        attr.freq = 0;
        attr.sample_period = period;
        return *this;
    }

    /**
     * Set the number of samples that need to be triggered before
     * reporting data as being available on the perf event
     * FD. Defaults to 0, which disables overflow reporting.
     *
     * @param events Number of overflows before signaling a wake up
     */
    PerfKvmCounterConfig &wakeupEvents(uint32_t events) {
        attr.watermark = 0;
        attr.wakeup_events = events;
        return *this;
    }

    /**
     * Don't start the performance counter automatically when
     * attaching it.
     *
     * @param val true to disable, false to enable the counter
     */
    PerfKvmCounterConfig &disabled(bool val) {
        attr.disabled = val;
        return *this;
    }

    /**
     * Force the group to be on the active all the time (i.e.,
     * disallow multiplexing).
     *
     * Only applies to group leaders.
     *
     * @param val true to pin the counter
     */
    PerfKvmCounterConfig &pinned(bool val) {
        attr.pinned = val;
        return *this;
    }

    /**
     * Exclude the events from the host (i.e., only include events
     * from the guest system).
     *
     * Intel CPUs seem to support this attribute from Linux 3.2 and
     * onwards. Non-x86 architectures currently ignore this attribute
     * (Linux 3.12-rc5).
     *
     * @warn This attribute is ignored if it isn't present in the
     * kernel headers or if the kernel doesn't support it.
     *
     * @param val true to exclude host events
     */
    PerfKvmCounterConfig &exclude_host(bool val) {
#if HAVE_PERF_ATTR_EXCLUDE_HOST == 1
        attr.exclude_host = val;
#endif
        return *this;
    }

    /**
     * Exclude the hyper visor (i.e., only include events from the
     * guest system).
     *
     * @warn This is attribute only seems to be ignored on Intel.
     *
     * @param val true to exclude host events
     */
    PerfKvmCounterConfig &exclude_hv(bool val) {
        attr.exclude_hv = val;
        return *this;
    }

    /** Underlying perf_event_attr structure describing the counter */
    struct perf_event_attr attr;
};

/**
 * An instance of a performance counter.
 */
class PerfKvmCounter
{
public:
    /**
     * Create and attach a new counter group.
     *
     * @param config Counter configuration
     * @param tid Thread to sample (0 indicates current thread)
     */
    PerfKvmCounter(PerfKvmCounterConfig &config, pid_t tid);
    /**
     * Create and attach a new counter and make it a member of an
     * exist counter group.
     *
     * @param config Counter configuration
     * @param tid Thread to sample (0 indicates current thread)
     * @param parent Group leader
     */
    PerfKvmCounter(PerfKvmCounterConfig &config,
                pid_t tid, const PerfKvmCounter &parent);
    /**
     * Create a new counter, but don't attach it.
     */
    PerfKvmCounter();
    ~PerfKvmCounter();


    /**
     * Attach a counter.
     *
     * @note This operation is only supported if the counter isn't
     * already attached.
     *
     * @param config Counter configuration
     * @param tid Thread to sample (0 indicates current thread)
     */
    void attach(PerfKvmCounterConfig &config, pid_t tid) {
        attach(config, tid, -1);
    }

    /**
     * Attach a counter and make it a member of an existing counter
     * group.
     *
     * @note This operation is only supported if the counter isn't
     * already attached.
     *
     * @param config Counter configuration
     * @param tid Thread to sample (0 indicates current thread)
     * @param parent Group leader
     */
    void attach(PerfKvmCounterConfig &config,
                pid_t tid, const PerfKvmCounter &parent) {
        attach(config, tid, parent.fd);
    }

    /** Detach a counter from PerfEvent. */
    void detach();

    /** Check if a counter is attached. */
    bool attached() const { return fd != -1; }

    /**
     * Start counting.
     *
     * @note If this counter is a group leader, it will start the
     * entire group.
     */
    void start();

    /**
     * Stop counting.
     *
     * @note If this counter is a group leader, it will stop the
     * entire group.
     */
    void stop();

    /**
     * Update the period of an overflow counter.
     *
     * @warning This ioctl has some pretty bizarre semantics. It seems
     * like the new period isn't effective until after the next
     * counter overflow. If you use this method to change the sample
     * period, you will see one sample with the old period and then
     * start sampling with the new period. This problem was fixed for
     * ARM in version 3.7 of the kernel.
     *
     * @warning This method doesn't work at all on some 2.6.3x kernels
     * since it has inverted check for the return value when copying
     * parameters from userspace.
     *
     * @param period Overflow period in events
     */
    void period(uint64_t period);

    /**
     * Enable a counter for a fixed number of events.
     *
     * When this method is called, perf event enables the counter if
     * it was disabled. It then leaves the counter enabled until it
     * has overflowed a refresh times.
     *
     * @note This does not update the period of the counter.
     *
     * @param refresh Number of overflows before disabling the
     * counter.
     */
    void refresh(int refresh);

    /**
     * Read the current value of a counter.
     */
    uint64_t read() const;

    /**
     * Enable signal delivery to a thread on counter overflow.
     *
     * @param tid Thread to deliver signal to
     * @param signal Signal to send upon overflow
     */
    void enableSignals(pid_t tid, int signal);

    /**
     * Enable signal delivery on counter overflow. Identical to
     * enableSignals(pid_t) when called with the current TID as its
     * parameter.
     *
     * @param signal Signal to send upon overflow
     */
    void enableSignals(int signal) { enableSignals(sysGettid(), signal); }

private:
    // Disallow copying
    PerfKvmCounter(const PerfKvmCounter &that);
    // Disallow assignment
    PerfKvmCounter &operator=(const PerfKvmCounter &that);

    void attach(PerfKvmCounterConfig &config, pid_t tid, int group_fd);

    /**
     * Get the TID of the current thread.
     *
     * @return Current thread's TID
     */
    pid_t sysGettid();

    /**
     * MMAP the PerfEvent file descriptor.
     *
     * @note We currently don't use the ring buffer, but PerfEvent
     * requires this to be mapped for overflow handling to work.
     *
     * @note Overflow handling requires at least one buf_page to be
     * mapped.
     *
     * @param pages number of pages in circular sample buffer. Must be
     * an even power of 2.
     */
    void mmapPerf(int pages);

    /** @{ */
    /**
     * PerfEvent fnctl interface.
     *
     * @param cmd fcntl command
     * @param p1 Request parameter
     *
     * @return -1 on error (error number in errno), ioctl dependent
     * value otherwise.
     */
    int fcntl(int cmd, long p1);
    int fcntl(int cmd, void *p1) { return fcntl(cmd, (long)p1); }
    /** @} */

    /** @{ */
    /**
     * PerfEvent ioctl interface.
     *
     * @param request PerfEvent request
     * @param p1 Optional request parameter
     *
     * @return -1 on error (error number in errno), ioctl dependent
     * value otherwise.
     */
    int ioctl(int request, long p1);
    int ioctl(int request, void *p1) { return ioctl(request, (long)p1); }
    int ioctl(int request) { return ioctl(request, 0L); }
    /** @} */

    /**
     * Perform a read from the counter file descriptor.
     *
     * @param buf Destination buffer
     * @param size Amount of data to read
     */
    void read(void *buf, size_t size) const;

    /**
     * PerfEvent file descriptor associated with counter. -1 if not
     * attached to PerfEvent.
     */
    int fd;

    /** Memory mapped PerfEvent sample ring buffer */
    struct perf_event_mmap_page *ringBuffer;
    /** Total number of pages in ring buffer */
    int ringNumPages;

    /** Cached host page size */
    long pageSize;
};

#endif
