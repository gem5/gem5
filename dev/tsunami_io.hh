/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * Tsunami I/O Space mapping including RTC/timer interrupts
 */

#ifndef __DEV_TSUNAMI_IO_HH__
#define __DEV_TSUNAMI_IO_HH__

#include "dev/io_device.hh"
#include "base/range.hh"
#include "dev/tsunami.hh"
#include "sim/eventq.hh"

/**
 * Tsunami I/O device is a catch all for all the south bridge stuff we care
 * to implement.
 */
class TsunamiIO : public PioDevice
{
  private:
    /** The base address of this device */
    Addr addr;

    /** The size of mappad from the above address */
    static const Addr size = 0xff;

    struct tm tm;

    /**
     * In Tsunami RTC only has two i/o ports one for data and one for
     * address, so you write the address and then read/write the
     * data. This store the address you are going to be reading from
     * on a read.
     */
    uint8_t RTCAddress;

  protected:

    /**
     * The ClockEvent is handles the PIT interrupts
     */
    class ClockEvent : public Event
    {
      protected:
        /** how often the PIT fires */
        Tick interval;
        /** The mode of the PIT */
        uint8_t mode;
        /** The status of the PIT */
        uint8_t status;
        /** The current count of the PIT */
        uint16_t current_count;
        /** The latched count of the PIT */
        uint16_t latched_count;
        /** The state of the output latch of the PIT */
        bool latch_on;
        /** The next count half (byte) to read */
        enum {READ_LSB, READ_MSB} read_byte;

      public:
        /**
         * Just set the mode to 0
         */
        ClockEvent();

        /**
         * processs the timer event
         */
        virtual void process();

        /**
         * Returns a description of this event
         * @return the description
         */
        virtual const char *description();

        /**
         * Schedule a timer interrupt to occur sometime in the future.
         */
        void Program(int count);

        /**
         * Write the mode bits of the PIT.
         * @param mode the new mode
         */
        void ChangeMode(uint8_t mode);

        /**
         * The current PIT status.
         * @return the status of the PIT
         */
        uint8_t Status();

        /**
         * Latch the count of the PIT.
         */
        void LatchCount();

        /**
         * The current PIT count.
         * @return the count of the PIT
         */
        uint8_t Read();

       /**
         * Serialize this object to the given output stream.
         * @param os The stream to serialize to.
         */
        virtual void serialize(std::ostream &os);


        /**
         * Reconstruct the state of this object from a checkpoint.
         * @param cp The checkpoint use.
         * @param section The section name of this object
         */
        virtual void unserialize(Checkpoint *cp, const std::string &section);
     };

    /**
     * Process RTC timer events and generate interrupts appropriately.
     */
    class RTCEvent : public Event
    {
      protected:
        /** A pointer back to tsunami to create interrupt the processor. */
        Tsunami* tsunami;
        Tick interval;

      public:
        /**
         * RTC Event initializes the RTC event by scheduling an event
         * RTC_RATE times pre second.
         */
        RTCEvent(Tsunami* t, Tick i);

        /**
         * Interrupt the processor and reschedule the event.
         */
        virtual void process();

        /**
         * Return a description of this event.
         * @return a description
         */
        virtual const char *description();

        /**
         * Serialize this object to the given output stream.
         * @param os The stream to serialize to.
         */
        virtual void serialize(std::ostream &os);

        /**
         * Reconstruct the state of this object from a checkpoint.
         * @param cp The checkpoint use.
         * @param section The section name of this object
         */
        virtual void unserialize(Checkpoint *cp, const std::string &section);

        void scheduleIntr();
    };

    /** uip UpdateInProgess says that the rtc is updating, but we just fake it
     * by alternating it on every read of the bit since we are going to
     * override the loop_per_jiffy time that it is trying to use the UIP to
     * calculate.
     */
    uint8_t uip;

    /** Mask of the PIC1 */
    uint8_t mask1;

    /** Mask of the PIC2 */
    uint8_t mask2;

    /** Mode of PIC1. Not used for anything */
    uint8_t mode1;

    /** Mode of PIC2. Not used for anything */
    uint8_t mode2;

    /** Raw PIC interrupt register before masking */
    uint8_t picr; //Raw PIC interrput register

    /** Is the pic interrupting right now or not. */
    bool picInterrupting;

    Tick clockInterval;

    /** A pointer to the Tsunami device which be belong to */
    Tsunami *tsunami;

    /**
     * This timer is initilized, but after I wrote the code
     * it doesn't seem to be used again, and best I can tell
     * it too is not connected to any interrupt port
     */
    ClockEvent timer0;

    /**
     * This timer is used to control the speaker, which
     * we normally could care less about, however it is
     * also used to calculated the clockspeed and hense
     * bogomips which is kinda important to the scheduler
     * so we need to implemnt it although after boot I can't
     * imagine we would be playing with the PC speaker much
     */
    ClockEvent timer2;

    /** This is the event used to interrupt the cpu like an RTC.  */
    RTCEvent rtc;

    /** The interval is set via two writes to the PIT.
     * This variable contains a flag as to how many writes have happened, and
     * the time so far.
     */
    uint16_t timerData;

  public:
    /**
     * Return the freqency of the RTC
     * @return interrupt rate of the RTC
     */
    Tick frequency() const;

    /**
     * Initialize all the data for devices supported by Tsunami I/O.
     * @param name name of this device.
     * @param t pointer back to the Tsunami object that we belong to.
     * @param init_time Time (as in seconds since 1970) to set RTC to.
     * @param a address we are mapped at.
     * @param mmu pointer to the memory controller that sends us events.
     */
    TsunamiIO(const std::string &name, Tsunami *t, time_t init_time,
              Addr a, MemoryController *mmu, HierParams *hier, Bus *bus,
              Tick pio_latency, Tick ci);

    /**
     * Create the tm struct from seconds since 1970
     */
    void set_time(time_t t);

    /**
      * Process a read to one of the devices we are emulating.
      * @param req Contains the address to read from.
      * @param data A pointer to write the read data to.
      * @return The fault condition of the access.
      */
    virtual Fault read(MemReqPtr &req, uint8_t *data);

    /**
      * Process a write to one of the devices we emulate.
      * @param req Contains the address to write to.
      * @param data The data to write.
      * @return The fault condition of the access.
      */
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    /**
     * Post an PIC interrupt to the CPU via the CChip
     * @param bitvector interrupt to post.
     */
    void postPIC(uint8_t bitvector);

    /**
     * Clear a posted interrupt
     * @param bitvector interrupt to clear
     */
    void clearPIC(uint8_t bitvector);

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    virtual void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    Tick cacheAccess(MemReqPtr &req);
};

#endif // __DEV_TSUNAMI_IO_HH__
