/*
 * Copyright (c) 2010, 2012-2013, 2015,2017-2020 ARM Limited
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
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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

#include "arch/arm/system.hh"

#include <iostream>

#include "arch/arm/fs_workload.hh"
#include "arch/arm/semihosting.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "dev/arm/fvp_base_pwr_ctrl.hh"
#include "dev/arm/gic_v2.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"

using namespace std;
using namespace Linux;

ArmSystem::ArmSystem(Params *p)
    : System(p),
      _haveSecurity(p->have_security),
      _haveLPAE(p->have_lpae),
      _haveVirtualization(p->have_virtualization),
      _haveCrypto(p->have_crypto),
      _genericTimer(nullptr),
      _gic(nullptr),
      _pwrCtrl(nullptr),
      _highestELIs64(p->highest_el_is_64),
      _physAddrRange64(p->phys_addr_range_64),
      _haveLargeAsid64(p->have_large_asid_64),
      _haveSVE(p->have_sve),
      _sveVL(p->sve_vl),
      _haveLSE(p->have_lse),
      _havePAN(p->have_pan),
      semihosting(p->semihosting),
      multiProc(p->multi_proc)
{
    if (p->auto_reset_addr) {
        _resetAddr = (workload->entry & workload->loadAddrMask) +
            workload->loadAddrOffset;
    } else {
        _resetAddr = p->reset_addr;
    }

    auto *arm_workload = dynamic_cast<ArmISA::FsWorkload *>(p->workload);
    panic_if(!arm_workload,
            "Workload was not the expected type (ArmISA::FsWorkload).");

    warn_if(workload->entry != _resetAddr,
            "Workload entry point %#x overriding reset address %#x",
            workload->entry, _resetAddr);
    _resetAddr = workload->entry;

    if (arm_workload->highestELIs64() != _highestELIs64) {
        warn("Highest ARM exception-level set to AArch%d but the workload "
              "is for AArch%d. Assuming you wanted these to match.",
              _highestELIs64 ? 64 : 32,
              arm_workload->highestELIs64() ? 64 : 32);
        _highestELIs64 = arm_workload->highestELIs64();
    }

    if (_highestELIs64 && (
            _physAddrRange64 < 32 ||
            _physAddrRange64 > 48 ||
            (_physAddrRange64 % 4 != 0 && _physAddrRange64 != 42))) {
        fatal("Invalid physical address range (%d)\n", _physAddrRange64);
    }
}

bool
ArmSystem::haveSecurity(ThreadContext *tc)
{
    return FullSystem? getArmSystem(tc)->haveSecurity() : false;
}

bool
ArmSystem::haveLPAE(ThreadContext *tc)
{
    return FullSystem? getArmSystem(tc)->haveLPAE() : false;
}

bool
ArmSystem::haveVirtualization(ThreadContext *tc)
{
    return FullSystem? getArmSystem(tc)->haveVirtualization() : false;
}

bool
ArmSystem::highestELIs64(ThreadContext *tc)
{
    return FullSystem? getArmSystem(tc)->highestELIs64() : true;
}

ExceptionLevel
ArmSystem::highestEL(ThreadContext *tc)
{
    return FullSystem? getArmSystem(tc)->highestEL() : EL1;
}

bool
ArmSystem::haveEL(ThreadContext *tc, ExceptionLevel el)
{
    switch (el) {
      case EL0:
      case EL1:
        return true;
      case EL2:
        return haveVirtualization(tc);
      case EL3:
        return haveSecurity(tc);
      default:
        warn("Unimplemented Exception Level\n");
        return false;
    }
}

Addr
ArmSystem::resetAddr(ThreadContext *tc)
{
    return getArmSystem(tc)->resetAddr();
}

uint8_t
ArmSystem::physAddrRange(ThreadContext *tc)
{
    return getArmSystem(tc)->physAddrRange();
}

Addr
ArmSystem::physAddrMask(ThreadContext *tc)
{
    return getArmSystem(tc)->physAddrMask();
}

bool
ArmSystem::haveLargeAsid64(ThreadContext *tc)
{
    return getArmSystem(tc)->haveLargeAsid64();
}

bool
ArmSystem::haveSemihosting(ThreadContext *tc)
{
    return FullSystem && getArmSystem(tc)->haveSemihosting();
}

uint64_t
ArmSystem::callSemihosting64(ThreadContext *tc,
                             uint32_t op, uint64_t param)
{
    ArmSystem *sys = getArmSystem(tc);
    return sys->semihosting->call64(tc, op, param);
}

uint32_t
ArmSystem::callSemihosting32(ThreadContext *tc,
                             uint32_t op, uint32_t param)
{
    ArmSystem *sys = getArmSystem(tc);
    return sys->semihosting->call32(tc, op, param);
}

void
ArmSystem::callSetStandByWfi(ThreadContext *tc)
{
    if (FVPBasePwrCtrl *pwr_ctrl = getArmSystem(tc)->getPowerController())
        pwr_ctrl->setStandByWfi(tc);
}

void
ArmSystem::callClearStandByWfi(ThreadContext *tc)
{
    if (FVPBasePwrCtrl *pwr_ctrl = getArmSystem(tc)->getPowerController())
        pwr_ctrl->clearStandByWfi(tc);
}

ArmSystem *
ArmSystemParams::create()
{
    return new ArmSystem(this);
}
