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
 *
 * Authors: Ali Saidi
 *          Andrew Schultz
 *          Miguel Serrano
 */

/** @file
 * Tsunami I/O Space mapping including RTC/timer interrupts
 */

#ifndef __DEV_TSUNAMI_IO_HH__
#define __DEV_TSUNAMI_IO_HH__

#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunami_cchip.hh"
#include "dev/intel_8254_timer.hh"
#include "dev/io_device.hh"
#include "dev/mc146818.hh"
#include "params/TsunamiIO.hh"
#include "sim/eventq.hh"

/**
 * Tsunami I/O device is a catch all for all the south bridge stuff we care
 * to implement.
 */
class TsunamiIO : public BasicPioDevice
{

  protected:

    class RTC : public MC146818
    {
      public:
        Tsunami *tsunami;
        RTC(const std::string &n, const TsunamiIOParams *p);

      protected:
        void handleEvent()
        {
            //Actually interrupt the processor here
            tsunami->cchip->postRTC();
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

    /** A pointer to the Tsunami device which be belong to */
    Tsunami *tsunami;

    /** Intel 8253 Periodic Interval Timer */
    Intel8254Timer pitimer;

    RTC rtc;

    uint8_t rtcAddr;

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

  public:
    typedef TsunamiIOParams Params;
    /**
     * Initialize all the data for devices supported by Tsunami I/O.
     * @param p pointer to Params struct
     */
    TsunamiIO(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

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

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Start running.
     */
    void startup() override;

};

#endif // __DEV_TSUNAMI_IO_HH__
