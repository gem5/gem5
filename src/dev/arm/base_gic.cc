/*
 * Copyright (c) 2012, 2017-2018 ARM Limited
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
 * Authors: Andreas Sandberg
 */

#include "dev/arm/base_gic.hh"

#include "cpu/thread_context.hh"
#include "dev/arm/realview.hh"
#include "params/ArmInterruptPin.hh"
#include "params/ArmPPI.hh"
#include "params/ArmSPI.hh"
#include "params/BaseGic.hh"

BaseGic::BaseGic(const Params *p)
        : PioDevice(p),
          platform(p->platform)
{
    RealView *const rv(dynamic_cast<RealView*>(p->platform));
    // The platform keeps track of the GIC that is hooked up to the
    // system. Due to quirks in gem5's configuration system, the
    // platform can't take a GIC as parameter. Instead, we need to
    // register with the platform when a new GIC is created. If we
    // can't find a platform, something is seriously wrong.
    fatal_if(!rv, "GIC model can't register with platform code");
    rv->setGic(this);
}

BaseGic::~BaseGic()
{
}

void
BaseGic::init()
{
    PioDevice::init();
    getSystem()->setGIC(this);
}

const BaseGic::Params *
BaseGic::params() const
{
    return dynamic_cast<const Params *>(_params);
}

ArmInterruptPinGen::ArmInterruptPinGen(const ArmInterruptPinParams *p)
  : SimObject(p)
{
}

ArmSPIGen::ArmSPIGen(const ArmSPIParams *p)
    : ArmInterruptPinGen(p), pin(new ArmSPI(p->platform, p->num))
{
}

ArmInterruptPin*
ArmSPIGen::get(ThreadContext* tc)
{
    return pin;
}

ArmPPIGen::ArmPPIGen(const ArmPPIParams *p)
    : ArmInterruptPinGen(p)
{
}

ArmInterruptPin*
ArmPPIGen::get(ThreadContext* tc)
{
    panic_if(!tc, "Invalid Thread Context\n");
    ContextID cid = tc->contextId();

    auto pin_it = pins.find(cid);

    if (pin_it != pins.end()) {
        // PPI Pin Already generated
        return pin_it->second;
    } else {
        // Generate PPI Pin
        auto p = static_cast<const ArmPPIParams *>(_params);
        ArmPPI *pin = new ArmPPI(p->platform, tc, p->num);

        pins.insert({cid, pin});

        return pin;
    }
}

ArmInterruptPin::ArmInterruptPin(
    Platform  *_platform, ThreadContext *tc, uint32_t int_num)
      : threadContext(tc), platform(dynamic_cast<RealView*>(_platform)),
        intNum(int_num)
{
    fatal_if(!platform, "Interrupt not connected to a RealView platform");
}

void
ArmInterruptPin::setThreadContext(ThreadContext *tc)
{
    panic_if(threadContext,
             "InterruptLine::setThreadContext called twice\n");

    threadContext = tc;
}

ContextID
ArmInterruptPin::targetContext() const
{
    panic_if(!threadContext, "Per-context interrupt triggered without a " \
             "call to InterruptLine::setThreadContext.\n");
    return threadContext->contextId();
}

ArmSPI::ArmSPI(
    Platform  *_platform, uint32_t int_num)
      : ArmInterruptPin(_platform, nullptr, int_num)
{
}

void
ArmSPI::raise()
{
    platform->gic->sendInt(intNum);
}

void
ArmSPI::clear()
{
    platform->gic->clearInt(intNum);
}

ArmPPI::ArmPPI(
    Platform  *_platform, ThreadContext *tc, uint32_t int_num)
      : ArmInterruptPin(_platform, tc, int_num)
{
}

void
ArmPPI::raise()
{
    platform->gic->sendPPInt(intNum, targetContext());
}

void
ArmPPI::clear()
{
    platform->gic->clearPPInt(intNum, targetContext());
}

ArmSPIGen *
ArmSPIParams::create()
{
    return new ArmSPIGen(this);
}

ArmPPIGen *
ArmPPIParams::create()
{
    return new ArmPPIGen(this);
}
