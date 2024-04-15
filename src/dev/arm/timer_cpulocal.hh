/*
 * Copyright (c) 2010-2011,2018 ARM Limited
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

#ifndef __DEV_ARM_LOCALTIMER_HH__
#define __DEV_ARM_LOCALTIMER_HH__

#include <cstdint>
#include <memory>
#include <vector>

#include "base/bitunion.hh"
#include "base/types.hh"
#include "dev/io_device.hh"
#include "params/CpuLocalTimer.hh"
#include "sim/serialize.hh"

/** @file
 * This implements the cpu local timer from the Cortex-A9 MPCore
 * Technical Reference Manual rev r2p2 (ARM DDI 0407F)
 */

namespace gem5
{

class BaseGic;
class ArmInterruptPin;

class CpuLocalTimer : public BasicPioDevice
{
  protected:
    class Timer : public Serializable
    {
      public:
        enum
        {
            TimerLoadReg = 0x00,
            TimerCounterReg = 0x04,
            TimerControlReg = 0x08,
            TimerIntStatusReg = 0x0C,
            WatchdogLoadReg = 0x20,
            WatchdogCounterReg = 0x24,
            WatchdogControlReg = 0x28,
            WatchdogIntStatusReg = 0x2C,
            WatchdogResetStatusReg = 0x30,
            WatchdogDisableReg = 0x34,
            Size = 0x38
        };

        BitUnion32(TimerCtrl)
            Bitfield<0> enable;
            Bitfield<1> autoReload;
            Bitfield<2> intEnable;
            Bitfield<7, 3> reserved;
            Bitfield<15, 8> prescalar;
        EndBitUnion(TimerCtrl)

        BitUnion32(WatchdogCtrl)
            Bitfield<0> enable;
            Bitfield<1> autoReload;
            Bitfield<2> intEnable;
            Bitfield<3> watchdogMode;
            Bitfield<7, 4> reserved;
            Bitfield<15, 8> prescalar;
        EndBitUnion(WatchdogCtrl)

      protected:
        std::string _name;

        /** Pointer to parent class */
        CpuLocalTimer *parent;

        /** Interrupt to cause/clear */
        ArmInterruptPin *intTimer;
        ArmInterruptPin *intWatchdog;

        /** Control register as specified above */
        TimerCtrl timerControl;
        WatchdogCtrl watchdogControl;

        /** If timer has caused an interrupt. This is irrespective of
         * interrupt enable */
        bool rawIntTimer;
        bool rawIntWatchdog;
        bool rawResetWatchdog;
        uint32_t watchdogDisableReg;

        /** If an interrupt is currently pending. Logical and of Timer or
         * Watchdog Ctrl.enable and rawIntTimer or rawIntWatchdog */
        bool pendingIntTimer;
        bool pendingIntWatchdog;

        /** Value to load into counters when periodic mode reaches 0 */
        uint32_t timerLoadValue;
        uint32_t watchdogLoadValue;

        /** Called when the counter reaches 0 */
        void timerAtZero();
        EventFunctionWrapper timerZeroEvent;

        void watchdogAtZero();
        EventFunctionWrapper watchdogZeroEvent;

      public:
        /** Restart the counter ticking at val
         * @param val the value to start at */
        void restartTimerCounter(uint32_t val);
        void restartWatchdogCounter(uint32_t val);

        Timer(const std::string &name, CpuLocalTimer *_parent,
              ArmInterruptPin *int_timer, ArmInterruptPin *int_watchdog);

        std::string
        name() const
        {
            return _name;
        }

        /** Handle read for a single timer */
        void read(PacketPtr pkt, Addr daddr);

        /** Handle write for a single timer */
        void write(PacketPtr pkt, Addr daddr);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        friend class CpuLocalTimer;
    };

    /** Pointer to the GIC for causing an interrupt */
    BaseGic *gic;

    /** Timers that do the actual work */
    std::vector<std::unique_ptr<Timer>> localTimer;

  public:
    PARAMS(CpuLocalTimer);

    /**
     * The constructor for RealView just registers itself with the MMU.
     * @param p params structure
     */
    CpuLocalTimer(const Params &p);

    /** Inits the local timers */
    void init() override;

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

} // namespace gem5

#endif // __DEV_ARM_SP804_HH__
