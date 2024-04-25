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

namespace gem5
{

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
  protected:
    /** Don't create directly; use PerfKvmCounter::create(). */
    PerfKvmCounter();

  public:
    virtual ~PerfKvmCounter() = default;

    /**
     * Create a new performance counter.
     * This automatically selects the appropriate implementation of
     * PerfKvmCounter, depending on whether the host has a hybrid architecture
     * (rare case) or not (common case).
     */
    static PerfKvmCounter *create(bool allow_hybrid);

    /**
     * Attach a counter and optionally make it a member of an existing counter
     * group.
     *
     * @note This operation is only supported if the counter isn't
     * already attached.
     *
     * @param config Counter configuration
     * @param tid Thread to sample (0 indicates current thread)
     * @param parent Group leader (nullptr indicates no group leader)
     */
    virtual void attach(const PerfKvmCounterConfig &config,
                        pid_t tid, const PerfKvmCounter *parent = nullptr) = 0;

    /** Detach a counter from PerfEvent. */
    virtual void detach() = 0;

    /** Check if a counter is attached. */
    virtual bool attached() const = 0;

    /**
     * Start counting.
     *
     * @note If this counter is a group leader, it will start the
     * entire group.
     */
    virtual void start() = 0;

    /**
     * Stop counting.
     *
     * @note If this counter is a group leader, it will stop the
     * entire group.
     */
    virtual void stop() = 0;

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
     * @note When using a hybrid perf counter, this actually sets
     * the period to 1/2 of the value provided. This ensures that an
     * overflow will always trigger before more than \p period events
     * occur, even in the pathological case when the host execution is
     * evenly split between a P-core and E-core.
     *
     * @param period Overflow period in events
     */
    virtual void period(uint64_t period) = 0;

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
    virtual void refresh(int refresh) = 0;

    /**
     * Read the current value of a counter.
     */
    virtual uint64_t read() const = 0;

    /**
     * Enable signal delivery to a thread on counter overflow.
     *
     * @param tid Thread to deliver signal to
     * @param signal Signal to send upon overflow
     */
    virtual void enableSignals(pid_t tid, int signal) = 0;

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

    /**
     * Get the TID of the current thread.
     *
     * @return Current thread's TID
     */
    pid_t sysGettid();
};

/**
 * An instance of a single physical performance counter.
 * In almost all cases, this is the PerfKvmCounter implementation to use. The
 * only situation in which you should _not_ use this counter is for creating
 a hardware event on a hybrid host architecture. In that case, use
 * a HybridPerfKvmCounter.
 */
class SimplePerfKvmCounter final : public PerfKvmCounter
{
  private:
    /** Don't create directly; use PerfKvmCounter::create(). */
    SimplePerfKvmCounter();

  public:
    ~SimplePerfKvmCounter();

    void attach(const PerfKvmCounterConfig &config,
                pid_t tid, const PerfKvmCounter *parent) override;

    void detach() override;
    bool attached() const override { return fd != -1; }
    void start() override;
    void stop() override;
    void period(uint64_t period) override;
    void refresh(int refresh) override;
    uint64_t read() const override;
    void enableSignals(pid_t tid, int signal) override;

  private:
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

    friend class PerfKvmCounter;
    friend class HybridPerfKvmCounter;
};

/**
 * Implements a single logical performance counter using two
 * physical performance counters (i.e., SimplePerfKvmCounter).
 * Use this for hardware counters (i.e., of type PERF_TYPE_HARDWARE)
 * when running on a hybrid host architecture. Such architectures have
 * multiple types of cores, each of which require their own individual
 * performance counter.
 */
class HybridPerfKvmCounter : public PerfKvmCounter
{
  private:
    /** Don't create directly; use PerfKvmCounter::create(). */
    HybridPerfKvmCounter() = default;

  public:
    void attach(const PerfKvmCounterConfig &config, pid_t tid,
                const PerfKvmCounter *parent) override;
    void detach() override;
    bool attached() const override;
    void start() override;
    void stop() override;
    void period(uint64_t period) override;
    void refresh(int refresh) override;
    uint64_t read() const override;
    void enableSignals(pid_t pid, int signal) override;

  private:
    SimplePerfKvmCounter coreCounter;
    SimplePerfKvmCounter atomCounter;

    using ConfigSubtype = decltype(perf_event_attr::config);
    using SamplePeriod = decltype(perf_event_attr::sample_type);

    /** @{ */
    /**
     * These constants for specifying core vs. atom events are taken from
     * Linux perf's documentation (tools/perf/Documentation/intel-hybrid.txt
     * in the linux source tree).
     */
    static inline constexpr ConfigSubtype ConfigCore = 0x4UL << 32;
    static inline constexpr ConfigSubtype ConfigAtom = 0x8UL << 32;
    /** @} */

    static PerfKvmCounterConfig fixupConfig(const PerfKvmCounterConfig &in,
                                            ConfigSubtype config_subtype);

    friend class PerfKvmCounter;
};

} // namespace gem5

#endif
