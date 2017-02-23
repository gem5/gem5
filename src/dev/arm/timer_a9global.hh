/*
 * Copyright (c) 2017 Gedare Bloom
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
 *
 * Authors: Ali Saidi
 *          Gedare Bloom
 */

#ifndef __DEV_ARM_GLOBAL_TIMER_HH__
#define __DEV_ARM_GLOBAL_TIMER_HH__

#include "dev/io_device.hh"
#include "params/A9GlobalTimer.hh"

/** @file
 * This implements the Cortex A9-MPCore global timer from TRM rev r4p1.
 * The global timer is an incrementing timer.
 */

class BaseGic;

class A9GlobalTimer : public BasicPioDevice
{
  protected:
    class Timer : public Serializable
    {

      public:
        /* TODO: IntStatusReg, CmpValRegLow32, CmpValRegHigh32,
         * and AutoIncrementReg are banked per-cpu. Some bits of
         * ControlReg are also banked per-cpu, see below. */
        enum {
            CounterRegLow32  = 0x00,
            CounterRegHigh32 = 0x04,
            ControlReg       = 0x08,
            IntStatusReg     = 0x0C,
            CmpValRegLow32   = 0x10,
            CmpValRegHigh32  = 0x14,
            AutoIncrementReg = 0x18,
            Size             = 0x1C
        };

        /* TODO: bits 1--3 are banked per-cpu */
        BitUnion32(CTRL)
            Bitfield<0>    enable;
            Bitfield<1>    cmpEnable;
            Bitfield<2>    intEnable;
            Bitfield<3>    autoIncrement;
            Bitfield<7,4>  reserved;
            Bitfield<15,8> prescalar;
        EndBitUnion(CTRL)

      protected:
        std::string _name;

        /** Pointer to parent class */
        A9GlobalTimer *parent;

        /** Number of interrupt to cause/clear */
        const uint32_t intNum;

        /** Control register as specified above */
        /* TODO: one per-cpu? */
        CTRL control;

        /** If timer has caused an interrupt. This is irrespective of
         * interrupt enable */
        /* TODO: one per-cpu */
        bool rawInt;

        /** If an interrupt is currently pending. Logical and of CTRL.intEnable
         * and rawInt */
        bool pendingInt;

        /** Value of the comparator */
        uint64_t cmpVal;

        /** Value to add to comparator when counter reaches comparator */
        /* TODO: one per-cpu */
        uint32_t autoIncValue;

        /** Called when the counter reaches the comparator */
        void counterAtCmpVal();
        EventWrapper<Timer, &Timer::counterAtCmpVal> cmpValEvent;

      public:
        /** Restart the counter ticking */
        void restartCounter();
        /**
          * Convert a number of ticks into the time counter format
          * @param ticks number of ticks
          */
        uint64_t getTimeCounterFromTicks(Tick ticks);
        Timer(std::string __name, A9GlobalTimer *parent, int int_num);

        std::string name() const { return _name; }

        /** Handle read for a single timer */
        void read(PacketPtr pkt, Addr daddr);

        /** Handle write for a single timer */
        void write(PacketPtr pkt, Addr daddr);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    /** Pointer to the GIC for causing an interrupt */
    BaseGic *gic;

    /** Timer that does the actual work */
    Timer global_timer;

  public:
    typedef A9GlobalTimerParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    /**
      * The constructor for RealView just registers itself with the MMU.
      * @param p params structure
      */
    A9GlobalTimer(Params *p);

    /**
     * Handle a read to the device
     * @param pkt The memory request.
     * @return Returns latency of device read
     */
    Tick read(PacketPtr pkt) override;

    /**
     * Handle a write to the device.
     * @param pkt The memory request.
     * @return Returns latency of device write
     */
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __DEV_ARM_GLOBAL_TIMER_HH__
