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
 * Malta I/O Space mapping including RTC/timer interrupts
 */

#ifndef __DEV_MALTA_IO_HH__
#define __DEV_MALTA_IO_HH__

#include "dev/mips/malta.hh"
#include "dev/mips/malta_cchip.hh"
#include "dev/intel_8254_timer.hh"
#include "dev/io_device.hh"
#include "dev/mc146818.hh"
#include "params/MaltaIO.hh"
#include "sim/eventq.hh"

/**
 * Malta I/O device is a catch all for all the south bridge stuff we care
 * to implement.
 */
class MaltaIO : public BasicPioDevice
{
  protected:

    class RTC : public MC146818
    {
      public:
        Malta *malta;
        RTC(const std::string &name, const MaltaIOParams *p);

      protected:
        void handleEvent()
        {
            //Actually interrupt the processor here
            malta->cchip->postRTC();
        }
    };

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

    /** A pointer to the Malta device which be belong to */
    Malta *malta;

    /** Intel 8253 Periodic Interval Timer */
    Intel8254Timer pitimer;

    RTC rtc;

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

    typedef MaltaIOParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /**
     * Initialize all the data for devices supported by Malta I/O.
     * @param p pointer to Params struct
     */
    MaltaIO(const Params *p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;


    /** Post an Interrupt to the CPU */
    void postIntr(uint8_t interrupt);

    /** Clear an Interrupt to the CPU */
    void clearIntr(uint8_t interrupt);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Start running.
     */
    void startup() override;

};

#endif // __DEV_MALTA_IO_HH__
