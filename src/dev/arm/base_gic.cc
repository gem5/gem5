/*
 * Copyright (c) 2012, 2017-2018, 2021 Arm Limited
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

#include "dev/arm/base_gic.hh"

#include "cpu/thread_context.hh"
#include "dev/arm/realview.hh"
#include "debug/GIC.hh"
#include "params/ArmInterruptPin.hh"
#include "params/ArmPPI.hh"
#include "params/ArmSigInterruptPin.hh"
#include "params/ArmSPI.hh"
#include "params/BaseGic.hh"

namespace gem5
{

BaseGic::BaseGic(const Params &p)
        : PioDevice(p),
          platform(p.platform)
{
    RealView *const rv = dynamic_cast<RealView*>(p.platform);
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

const BaseGic::Params &
BaseGic::params() const
{
    return dynamic_cast<const Params &>(_params);
}

ArmInterruptPinGen::ArmInterruptPinGen(const ArmInterruptPinParams &p)
  : SimObject(p)
{
}

ArmSPIGen::ArmSPIGen(const ArmSPIParams &p)
    : ArmInterruptPinGen(p), pin(new ArmSPI(p))
{
}

ArmInterruptPin*
ArmSPIGen::get(ThreadContext* tc)
{
    return pin;
}

ArmPPIGen::ArmPPIGen(const ArmPPIParams &p)
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
        ArmPPI *pin = new ArmPPI(ArmPPIGen::params(), tc);

        pins.insert({cid, pin});

        return pin;
    }
}

ArmSigInterruptPinGen::ArmSigInterruptPinGen(const ArmSigInterruptPinParams &p)
    : ArmInterruptPinGen(p), pin(new ArmSigInterruptPin(p))
{}

ArmInterruptPin*
ArmSigInterruptPinGen::get(ThreadContext* tc)
{
    return pin;
}

Port &
ArmSigInterruptPinGen::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "irq") {
        assert(idx != InvalidPortID);
        if (idx >= pin->sigPin.size())
            pin->sigPin.resize(idx + 1);
        if (!pin->sigPin.at(idx))
            pin->sigPin.at(idx).reset(
                new IntSourcePin<ArmSigInterruptPinGen>(
                    csprintf("%s.irq[%d]", name(), idx), idx, this));
        return *pin->sigPin.at(idx);
    }

    return ArmInterruptPinGen::getPort(if_name, idx);
}

ArmInterruptPin::ArmInterruptPin(
    const ArmInterruptPinParams &p, ThreadContext *tc)
      : threadContext(tc), platform(dynamic_cast<RealView*>(p.platform)),
        intNum(p.num), triggerType(p.int_type), _active(false)
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

void
ArmInterruptPin::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_active);
}

void
ArmInterruptPin::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_active);
}

ArmSPI::ArmSPI(
    const ArmSPIParams &p)
      : ArmInterruptPin(p, nullptr)
{
}

void
ArmSPI::raise()
{
    _active = true;
    platform->gic->sendInt(intNum);
}

void
ArmSPI::clear()
{
    _active = false;
    platform->gic->clearInt(intNum);
}

ArmPPI::ArmPPI(
    const ArmPPIParams &p, ThreadContext *tc)
      : ArmInterruptPin(p, tc)
{
}

void
ArmPPI::raise()
{
    _active = true;
    platform->gic->sendPPInt(intNum, targetContext());
}

void
ArmPPI::clear()
{
    _active = false;
    platform->gic->clearPPInt(intNum, targetContext());
}

ArmSigInterruptPin::ArmSigInterruptPin(const ArmSigInterruptPinParams &p)
      : ArmInterruptPin(p, nullptr)
{}

void
ArmSigInterruptPin::raise()
{
    _active = true;
    for (auto &pin : sigPin)
        if (pin)
            pin->raise();
}

void
ArmSigInterruptPin::clear()
{
    _active = false;
    for (auto &pin : sigPin)
        if (pin)
            pin->lower();
}

} // namespace gem5
