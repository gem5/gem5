/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/* @file
 * Tsunami Fake I/O Space mapping including RTC/timer interrupts
 */

#ifndef __TSUNAMI_DMA_HH__
#define __TSUNAMI_DMA_HH__

#define RTC_RATE 1024

#include "mem/functional_mem/functional_memory.hh"
#include "dev/tsunami.hh"

/*
 * Tsunami I/O device
 */
class TsunamiIO : public FunctionalMemory
{
  private:
    Addr addr;
    static const Addr size = 0xff;

    struct tm tm;

    // In Tsunami RTC only has two i/o ports
    // one  for data and one for address, so you
    // write the address and then read/write the data
    uint8_t RTCAddress;

  protected:

    class ClockEvent : public Event
    {
      protected:
        Tick interval;
        uint8_t mode;
        uint8_t status;

      public:
        ClockEvent();

        virtual void process();
        virtual const char *description();
        void Program(int count);
        void ChangeMode(uint8_t mode);
        uint8_t Status();

    };

    class RTCEvent : public Event
    {
      protected:
        Tsunami* tsunami;
      public:
        RTCEvent(Tsunami* t);

        virtual void process();
        virtual const char *description();
    };

    uint8_t uip;

    uint8_t mask1;
    uint8_t mask2;
    uint8_t mode1;
    uint8_t mode2;

    uint8_t picr; //Raw PIC interrput register, before masking
    bool picInterrupting;

    Tsunami *tsunami;

    /*
     * This timer is initilized, but after I wrote the code
     * it doesn't seem to be used again, and best I can tell
     * it too is not connected to any interrupt port
     */
    ClockEvent timer0;

    /*
     * This timer is used to control the speaker, which
     * we normally could care less about, however it is
     * also used to calculated the clockspeed and hense
     * bogomips which is kinda important to the scheduler
     * so we need to implemnt it although after boot I can't
     * imagine we would be playing with the PC speaker much
     */
    ClockEvent timer2;

    RTCEvent rtc;

    uint32_t timerData;


  public:
    uint32_t  frequency() const { return RTC_RATE; }

    TsunamiIO(const std::string &name, Tsunami *t, time_t init_time,
              Addr a, MemoryController *mmu);

    void set_time(time_t t);

    virtual Fault read(MemReqPtr &req, uint8_t *data);
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    void postPIC(uint8_t bitvector);
    void clearPIC(uint8_t bitvector);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __TSUNAMI_IO_HH__
