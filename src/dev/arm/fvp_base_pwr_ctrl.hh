/*
 * Copyright (c) 2020 ARM Limited
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

#ifndef __DEV_ARM_FVP_BASE_PWR_CTRL_HH__
#define __DEV_ARM_FVP_BASE_PWR_CTRL_HH__

#include <unordered_map>

#include "base/bitunion.hh"
#include "dev/io_device.hh"

namespace gem5
{

class ArmSystem;
struct FVPBasePwrCtrlParams;
class ThreadContext;

/**
 * @file
 * This class implements the base power controller for FVP-based
 * platforms. Based on Fast Models version 11.8.
 */
class FVPBasePwrCtrl : public BasicPioDevice
{
  public:
    FVPBasePwrCtrl(const FVPBasePwrCtrlParams &params);

    /**
     * Triggered by the ISA when a WFI instruction is executed and (1) there
     * are no pending interrupts and (2) it is not trapped. Core is set
     * to quiescent state only if there is a pending power off
     * @param tc Thread context representing the core
     */
    void setStandByWfi(ThreadContext *const tc);

    /**
     * Triggered when an interrupt is posted to the core. Core is brought up
     * from quiescent state if it is suspended. The latter is done by
     * BaseCPU::wakeup.
     * @param tc Thread context representing the core
     */
    void clearStandByWfi(ThreadContext *const tc);

    /**
     * Triggered by the GIC when GICR_WAKER.ProcessorSleep is 1 and there are
     * pending interrupts for the core
     * @param tc Thread context representing the core
     * @return true if the core is powered ON when PwrStatus.WEN is enabled,
     * false otherwise
     */
    bool setWakeRequest(ThreadContext *const tc);

    /**
     * Triggered by the GIC when GICR_WAKER.ProcessorSleep becomes 0
     * @param tc Thread context representing the core
     */
    void clearWakeRequest(ThreadContext *const tc);

    void startup() override;

  protected:
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  private:
    BitUnion32(PwrStatus)
        Bitfield<30> l1;
        Bitfield<29> l0;
        Bitfield<28> wen;
        Bitfield<27> pc;
        Bitfield<26> pp;
        Bitfield<25,24> wk;
        Bitfield<1> pwfi;
        Bitfield<0> pwk;
    EndBitUnion(PwrStatus)

    enum Offset : Addr
    {
        PPOFFR = 0x00,
        PPONR  = 0x04,
        PCOFFR = 0x08,
        PWKUPR = 0x0c,
        PSYSR  = 0x10
    };

    struct Registers
    {
        uint32_t ppoffr;
        uint32_t pponr;
        uint32_t pcoffr;
        uint32_t pwkupr;
        uint32_t psysr;
    } regs;

    /** Mask for extracting the MPID from a 32-bit value */
    static constexpr uint32_t MPID_MSK = 0x00ffffff;
    /** Wake-up reasons */
    enum { WK_COLD, WK_RESET, WK_PPONR, WK_GICWR };

    /**
     * Per-core power status. This is power related information for each core
     * that is bound to this power controller functionality
     */
    std::vector<PwrStatus> corePwrStatus;

    /** Number of powered cores per cluster. Helps keep track of PSYSR.L1 */
    std::unordered_map<uint32_t, size_t> poweredCoresPerCluster;

    /**
     * Retrieves the power status of a certain core and resizes the entries
     * if needed. This is a workaround for a limitation of the System object
     * only exposing existing thread contexts after "init()"
     * @param Thread (HW) context in the core
     * @return Core power status
     */
    PwrStatus *getCorePwrStatus(ThreadContext *const tc);

    /**
     * Retrieves the thread context reference for a CPU core by MPID
     * @param mpid ID provided by software
     * @return valid thread context reference if valid MPID, nullptr otherwise
     */
    ThreadContext *getThreadContextByMPID(uint32_t mpid) const;

    /**
     * Powers on a core. A Reset fault is invoked in the core followed by
     * an activation
     * @param Thread (HW) context in the core
     * @param Core power status
     */
    void powerCoreOn(ThreadContext *const tc, PwrStatus *const pwrs);

    /**
     * Powers off a core. The core enters into quiescent state until
     * an explicit PPONR write or a WakeRequest from the GIC wakes it up
     * @param Thread (HW) context in the core
     * @param Core power status
     */
    void powerCoreOff(ThreadContext *const tc, PwrStatus *const pwrs);

    /**
     * Starts a core up. This invokes the reset vector to setup the wake-up
     * entrypoint and activates the thread context. This covers cases when
     * PSYSR.WEN is enabled or PPONR is written
     * @param Thread (HW) context in the core
     */
    void startCoreUp(ThreadContext *const tc);

    /** Reference to the arm system */
    ArmSystem &system;
};

} // namespace gem5

#endif // __DEV_ARM_FVP_BASE_PWR_CTRL_HH__
