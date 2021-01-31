/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Anthony Gutierrez
 *          Eric van Tassell
 */

/**
 * @file
 * An HSADriver is an emulated driver that controls an HSA agent,
 * or more simply put, an HSA device. An HSA device is a device
 * that has an associated HSA packet processor.
 *
 * In the base HSADriver class the open() method is implemented, as
 * well as the mmap() call, which maps the HSA packet processor's
 * doorbells. Drivers for other HSA devices should derive from this
 * class and implement the necessary methods; typically this is an
 * ioctl() method that satisfies the ioctl requests needed to manage
 * and control the device.
 */

#ifndef __DEV_HSA_HSA_DRIVER_HH__
#define __DEV_HSA_HSA_DRIVER_HH__

#include <unordered_map>

#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "sim/emul_driver.hh"

struct HSADriverParams;
class HSADevice;
class PortProxy;

class HSADriver : public EmulatedDriver
{
  public:
    HSADriver(const HSADriverParams &p);

    int open(ThreadContext *tc, int mode, int flags);
    Addr mmap(ThreadContext *tc, Addr start, uint64_t length,
              int prot, int tgt_flags, int tgt_fd, off_t offset);
    virtual void signalWakeupEvent(uint32_t event_id);
    class DriverWakeupEvent : public Event
    {
      public:
        DriverWakeupEvent(HSADriver *hsa_driver, ThreadContext *thrd_cntxt)
            : driver(hsa_driver), tc(thrd_cntxt)  {}
        void process() override;
        const char *description() const override;
        void scheduleWakeup(Tick wakeup_delay);
      private:
        HSADriver *driver;
        ThreadContext *tc;
    };
    class EventTableEntry {
      public:
        EventTableEntry() :
            mailBoxPtr(0), tc(nullptr), threadWaiting(false), setEvent(false)
        {}
        // Mail box pointer for this address. Current implementation does not
        // use this mailBoxPtr to notify events but directly calls
        // signalWakeupEvent from dispatcher (GPU) to notify event. So,
        // currently this mailBoxPtr is not used. But a future implementation
        // may communicate to the driver using mailBoxPtr.
        Addr mailBoxPtr;
        // Thread context waiting on this event. We do not support multiple
        // threads waiting on an event currently.
        ThreadContext *tc;
        // threadWaiting = true, if some thread context is waiting on this
        // event.  A thread context waiting on this event is put to sleep.
        bool threadWaiting;
        // setEvent = true, if this event is triggered but when this event
        // triggered, no thread context was waiting on it. In the future, some
        // thread context will try to wait on this event but since event has
        // already happened, we will not allow that thread context to go to
        // sleep. The above mentioned scenario can happen when the waiting
        // thread and wakeup thread race on this event and the wakeup thread
        // beat the waiting thread at the driver.
        bool setEvent;
    };
    typedef class EventTableEntry ETEntry;

  protected:
    Addr eventPage;
    uint32_t eventSlotIndex;
    // Event table that keeps track of events. It is indexed with event ID.
    std::unordered_map<uint32_t, ETEntry> ETable;

    // TCEvents map keeps track of the events that can wakeup this thread. When
    // multiple events can wake up this thread, this data structure helps to
    // reset all events when one of those events wake up this thread. The
    // signal events that can wake up this thread are stored in signalEvents
    // whereas the timer wakeup event is stored in timerEvent.
    class EventList {
      public:
        EventList() : driver(nullptr), timerEvent(nullptr, nullptr) {}
        EventList(HSADriver *hsa_driver, ThreadContext *thrd_cntxt)
            : driver(hsa_driver), timerEvent(hsa_driver, thrd_cntxt)
        { }
        void clearEvents() {
            assert(driver);
            for (auto event : signalEvents) {
               assert(event < driver->eventSlotIndex);
               panic_if(driver->ETable[event].tc->status() == \
                            ThreadContext::Suspended,
                        "Thread should not be suspended\n");
               driver->ETable[event].tc = nullptr;
               driver->ETable[event].threadWaiting = false;
            }
            signalEvents.clear();
            if (timerEvent.scheduled()) {
                driver->deschedule(timerEvent);
            }
        }
        HSADriver *driver;
        DriverWakeupEvent timerEvent;
        // The set of events that can wake up the same thread.
        std::set<uint32_t> signalEvents;
    };
    std::unordered_map<ThreadContext *, EventList> TCEvents;

    /**
     * HSA agent (device) that is controled by this driver.
     */
    HSADevice *device;
    uint32_t queueId;

    void allocateQueue(ThreadContext *tc, Addr ioc_buf);
};

#endif // __DEV_HSA_HSA_DRIVER_HH__
