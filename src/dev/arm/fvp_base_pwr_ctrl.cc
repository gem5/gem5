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

#include "dev/arm/fvp_base_pwr_ctrl.hh"

#include "arch/arm/faults.hh"
#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/FVPBasePwrCtrl.hh"
#include "mem/packet_access.hh"
#include "params/FVPBasePwrCtrl.hh"
#include "sim/system.hh"

namespace gem5
{

FVPBasePwrCtrl::FVPBasePwrCtrl(const FVPBasePwrCtrlParams &params)
    : BasicPioDevice(params, 0x1000),
      regs(),
      system(*static_cast<ArmSystem *>(sys))
{
    warn_if(sys->multiThread,
            "Base Power Controller does not support multi-threaded systems\n");
    system.setPowerController(this);
}

void
FVPBasePwrCtrl::startup()
{
    // All cores are ON by default (PwrStatus.{l0,l1} = 0b1)
    corePwrStatus.resize(sys->threads.size(), 0x60000000);
    for (const auto &tc : sys->threads)
        poweredCoresPerCluster[tc->socketId()] += 1;
    BasicPioDevice::startup();
}

void
FVPBasePwrCtrl::setStandByWfi(ThreadContext *const tc)
{
    PwrStatus *pwrs = getCorePwrStatus(tc);

    if (!pwrs->pwfi)
        DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::setStandByWfi: STANDBYWFI "
                "asserted for core %d\n", tc->contextId());
    pwrs->pwfi = 1;
    if (pwrs->l0 && (pwrs->pp || pwrs->pc))
        powerCoreOff(tc, pwrs);
}

void
FVPBasePwrCtrl::clearStandByWfi(ThreadContext *const tc)
{
    PwrStatus *pwrs = getCorePwrStatus(tc);

    if (pwrs->pwfi)
        DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::clearStandByWfi: STANDBYWFI "
                "deasserted for core %d\n", tc->contextId());
    pwrs->pwfi = 0;
}

bool
FVPBasePwrCtrl::setWakeRequest(ThreadContext *const tc)
{
    PwrStatus *pwrs = getCorePwrStatus(tc);

    if (!pwrs->pwk)
        DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::setWakeRequest: WakeRequest "
                "asserted for core %d\n", tc->contextId());
    pwrs->pwk = 1;
    if (!pwrs->l0 && pwrs->wen) {
        pwrs->wk = WK_GICWR;
        powerCoreOn(tc, pwrs);
        return true;
    }
    return false;
}

void
FVPBasePwrCtrl::clearWakeRequest(ThreadContext *const tc)
{
    PwrStatus *pwrs = getCorePwrStatus(tc);

    if (pwrs->pwk)
        DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::clearWakeRequest: "
                "WakeRequest deasserted for core %d\n", tc->contextId());
    pwrs->pwk = 0;
}

Tick
FVPBasePwrCtrl::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    const size_t size = pkt->getSize();
    panic_if(size != 4, "FVPBasePwrCtrl::read: Invalid size %i\n", size);

    uint64_t resp = 0;
    switch (addr) {
      case PPOFFR:
        resp = regs.ppoffr;
        break;
      case PPONR:
        resp = regs.pponr;
        break;
      case PCOFFR:
        resp = regs.pcoffr;
        break;
      case PWKUPR:
        resp = regs.pwkupr;
        break;
      case PSYSR:
        resp = regs.psysr;
        break;
      default:
        warn("FVPBasePwrCtrl::read: Unexpected address (0x%x:%i), "
             "assuming RAZ\n", addr, size);
    }

    DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::read: 0x%x<-0x%x(%i)\n", resp,
            addr, size);

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeResponse();
    return pioDelay;
}

Tick
FVPBasePwrCtrl::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    const size_t size = pkt->getSize();
    panic_if(size != 4, "FVPBasePwrCtrl::write: Invalid size %i\n", size);

    uint64_t data = pkt->getUintX(ByteOrder::little);

    // Software may use the power controller to check for core presence
    // If core is not present, return an invalid MPID as notification
    ThreadContext *tc = getThreadContextByMPID(data & MPID_MSK);
    PwrStatus *pwrs = tc ? getCorePwrStatus(tc) : nullptr;
    switch (addr) {
      case PPOFFR:
        if (!tc) {
            regs.ppoffr = ~0;
        } else if (pwrs->l0) {
            // Set pending power off
            pwrs->pp = 1;
            regs.ppoffr = data & MPID_MSK;
        } else {
            regs.ppoffr = ~0 & MPID_MSK;
        }
        break;
      case PPONR:
        if (!tc) {
            regs.pponr = ~0;
        } else {
            if (!pwrs->l0) {
                pwrs->wk = WK_PPONR;
                powerCoreOn(tc, pwrs);
                startCoreUp(tc);
                regs.pponr = data & MPID_MSK;
            } else {
                regs.pponr = ~0 & MPID_MSK;
            }
        }
        break;
      case PCOFFR:
        if (!tc) {
            regs.pcoffr = ~0;
        } else if (pwrs->l0) {
            // Power off all cores in the cluster
            for (const auto &tco : sys->threads) {
                if (tc->socketId() == tco->socketId()) {
                    PwrStatus *npwrs = getCorePwrStatus(tco);
                    // Set pending cluster power off
                    npwrs->pc = 1;
                    if (npwrs->l0 && npwrs->pwfi)
                        powerCoreOff(tco, npwrs);
                }
            }
        } else {
            regs.pcoffr = ~0 & MPID_MSK;
        }
        break;
      case PWKUPR:
        if (!tc) {
            regs.pwkupr = ~0;
        } else {
            // Update WEN value
            pwrs->wen = bits(data, 31);
            // Power-on if there is any pending Wakeup Requests
            if (!pwrs->l0 && pwrs->wen && pwrs->pwk) {
                pwrs->wk = WK_GICWR;
                powerCoreOn(tc, pwrs);
                startCoreUp(tc);
            }
            regs.pwkupr = data & (MPID_MSK | (1 << 31));
        }
        break;
      case PSYSR:
        if (!tc)
            regs.psysr = ~0;
        else
            regs.psysr = (data & MPID_MSK) | (((uint32_t) *pwrs) & ~MPID_MSK);
        break;
      default:
        warn("FVPBasePwrCtrl::write: Unexpected address (0x%x:%i), "
             "assuming WI\n", addr, size);
    }

    DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::write: 0x%x->0x%x(%i)\n", data,
            addr, size);

    pkt->makeResponse();
    return pioDelay;
}

FVPBasePwrCtrl::PwrStatus *
FVPBasePwrCtrl::getCorePwrStatus(ThreadContext *const tc)
{
    PwrStatus *pwrs = &corePwrStatus[tc->contextId()];
    pwrs->l1 = poweredCoresPerCluster[tc->socketId()] > 0;
    return pwrs;
}

ThreadContext *
FVPBasePwrCtrl::getThreadContextByMPID(uint32_t mpid) const
{
    for (auto &tc : sys->threads) {
        if (mpid == ArmISA::getAffinity(&system, tc))
            return tc;
    }
    return nullptr;
}

void
FVPBasePwrCtrl::powerCoreOn(ThreadContext *const tc, PwrStatus *const pwrs)
{
    DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::powerCoreOn: Powering ON "
            "core %d\n", tc->contextId());
    pwrs->l0 = 1;
    poweredCoresPerCluster[tc->socketId()]++;
    // Clear pending power-offs to the core
    pwrs->pp = 0;
    // Clear pending power-offs to the core's cluster
    for (const auto &tco : sys->threads) {
        if (tc->socketId() == tco->socketId()) {
            PwrStatus *npwrs = getCorePwrStatus(tco);
            npwrs->pc = 0;
        }
    }
    tc->getCpuPtr()->powerState->set(enums::PwrState::ON);
}

void
FVPBasePwrCtrl::powerCoreOff(ThreadContext *const tc, PwrStatus *const pwrs)
{
    DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::powerCoreOff: Powering OFF "
            "core %d\n", tc->contextId());
    pwrs->l0 = 0;
    poweredCoresPerCluster[tc->socketId()]--;
    // Clear pending power-offs to the core
    pwrs->pp = 0;
    pwrs->pc = 0;
    // Clear power-on reason
    pwrs->wk = 0;
    tc->getCpuPtr()->powerState->set(enums::PwrState::OFF);
}

void
FVPBasePwrCtrl::startCoreUp(ThreadContext *const tc)
{
    DPRINTF(FVPBasePwrCtrl, "FVPBasePwrCtrl::startCoreUp: Starting core %d "
            "from the power controller\n", tc->contextId());
    clearStandByWfi(tc);
    clearWakeRequest(tc);

    // InitCPU
    ArmISA::Reset().invoke(tc);
    tc->activate();
}

} // namespace gem5
