/*
 * Copyright (c) 2010 ARM Limited
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

#ifndef __DEV_ARM_SP804_HH__
#define __DEV_ARM_SP804_HH__

#include "dev/arm/amba_device.hh"
#include "params/Sp804.hh"

/** @file
 * This implements the dual Sp804 timer block
 */

class BaseGic;

class Sp804 : public AmbaPioDevice
{
  protected:
    class Timer : public Serializable
    {

      public:
        enum {
            LoadReg    = 0x00,
            CurrentReg = 0x04,
            ControlReg = 0x08,
            IntClear   = 0x0C,
            RawISR     = 0x10,
            MaskedISR  = 0x14,
            BGLoad     = 0x18,
            Size       = 0x20
        };

        BitUnion32(CTRL)
            Bitfield<0>   oneShot;
            Bitfield<1>   timerSize;
            Bitfield<3,2> timerPrescale;
            Bitfield<5>   intEnable;
            Bitfield<6>   timerMode;
            Bitfield<7>   timerEnable;
        EndBitUnion(CTRL)

      protected:
        std::string _name;

        /** Pointer to parent class */
        Sp804 *parent;

        /** Pointer to the interrupt pin */
        ArmInterruptPin * const interrupt;

        /** Number of ticks in a clock input */
        const Tick clock;

        /** Control register as specified above */
        CTRL control;

        /** If timer has caused an interrupt. This is irrespective of
         * interrupt enable */
        bool rawInt;

        /** If an interrupt is currently pending. Logical and of CTRL.intEnable
         * and rawInt */
        bool pendingInt;

        /** Value to load into counter when periodic mode reaches 0 */
        uint32_t loadValue;

        /** Called when the counter reaches 0 */
        void counterAtZero();
        EventFunctionWrapper zeroEvent;

      public:
        /** Restart the counter ticking at val
         * @param val the value to start at (pre-16 bit masking if en) */
        void restartCounter(uint32_t val);

        Timer(std::string __name, Sp804 *parent, ArmInterruptPin *_interrupt,
              Tick clock);

        std::string name() const { return _name; }

        /** Handle read for a single timer */
        void read(PacketPtr pkt, Addr daddr);

        /** Handle write for a single timer */
        void write(PacketPtr pkt, Addr daddr);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    /** Timers that do the actual work */
    Timer timer0;
    Timer timer1;

  public:
    typedef Sp804Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    /**
      * The constructor for RealView just registers itself with the MMU.
      * @param p params structure
      */
    Sp804(Params *p);

    /**
     * Handle a read to the device
     * @param pkt The memory request.
     * @param data Where to put the data.
     */
    Tick read(PacketPtr pkt) override;

    /**
     * All writes are simply ignored.
     * @param pkt The memory request.
     * @param data the data
     */
    Tick write(PacketPtr pkt) override;


    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};


#endif // __DEV_ARM_SP804_HH__

