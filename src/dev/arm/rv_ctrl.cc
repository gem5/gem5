/*
 * Copyright (c) 2010,2013,2015 ARM Limited
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
 */

#include "dev/arm/rv_ctrl.hh"

#include "base/trace.hh"
#include "debug/RVCTRL.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/power/thermal_model.hh"
#include "sim/system.hh"
#include "sim/voltage_domain.hh"

RealViewCtrl::RealViewCtrl(Params *p)
    : BasicPioDevice(p, 0xD4), flags(0), scData(0)
{
}

Tick
RealViewCtrl::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;

    switch(daddr) {
      case ProcId0:
        pkt->set(params()->proc_id0);
        break;
      case ProcId1:
        pkt->set(params()->proc_id1);
        break;
      case Clock24:
        Tick clk;
        clk = SimClock::Float::MHz * curTick() * 24;
        pkt->set((uint32_t)(clk));
        break;
      case Clock100:
        Tick clk100;
        clk100 = SimClock::Float::MHz * curTick() * 100;
        pkt->set((uint32_t)(clk100));
        break;
      case Flash:
        pkt->set<uint32_t>(0);
        break;
      case Clcd:
        pkt->set<uint32_t>(0x00001F00);
        break;
      case Osc0:
        pkt->set<uint32_t>(0x00012C5C);
        break;
      case Osc1:
        pkt->set<uint32_t>(0x00002CC0);
        break;
      case Osc2:
        pkt->set<uint32_t>(0x00002C75);
        break;
      case Osc3:
        pkt->set<uint32_t>(0x00020211);
        break;
      case Osc4:
        pkt->set<uint32_t>(0x00002C75);
        break;
      case Lock:
        pkt->set<uint32_t>(sysLock);
        break;
      case Flags:
        pkt->set<uint32_t>(flags);
        break;
      case IdReg:
        pkt->set<uint32_t>(params()->idreg);
        break;
      case CfgStat:
        pkt->set<uint32_t>(1);
        break;
      case CfgData:
        pkt->set<uint32_t>(scData);
        DPRINTF(RVCTRL, "Read %#x from SCReg\n", scData);
        break;
      case CfgCtrl:
        pkt->set<uint32_t>(0); // not busy
        DPRINTF(RVCTRL, "Read 0 from CfgCtrl\n");
        break;
      default:
        warn("Tried to read RealView I/O at offset %#x that doesn't exist\n",
             daddr);
        pkt->set<uint32_t>(0);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;

}

Tick
RealViewCtrl::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;
    switch (daddr) {
      case Flash:
      case Clcd:
      case Osc0:
      case Osc1:
      case Osc2:
      case Osc3:
      case Osc4:
        break;
      case Lock:
        sysLock.lockVal = pkt->get<uint16_t>();
        break;
      case Flags:
        flags = pkt->get<uint32_t>();
        break;
      case FlagsClr:
        flags = 0;
        break;
      case CfgData:
        scData = pkt->get<uint32_t>();
        break;
      case CfgCtrl: {
          // A request is being submitted to read/write the system control
          // registers.  See
          // http://infocenter.arm.com/help/topic/com.arm.doc.dui0447h/CACDEFGH.html
          CfgCtrlReg req = pkt->get<uint32_t>();
          if (!req.start) {
              DPRINTF(RVCTRL, "SCReg: write %#x to ctrl but not starting\n",
                      req);
              break;
          }

          auto it_dev(devices.find(req & CFG_CTRL_ADDR_MASK));
          if (it_dev == devices.end()) {
              warn_once("SCReg: Access to unknown device "
                        "dcc%d:site%d:pos%d:fn%d:dev%d\n",
                        req.dcc, req.site, req.pos, req.func, req.dev);
              break;
          }

          // Service the request as a read or write depending on the
          // wr bit in the control register.
          Device &dev(*it_dev->second);
          if (req.wr) {
              DPRINTF(RVCTRL, "SCReg: Writing %#x (ctrlWr %#x)\n",
                      scData, req);
              dev.write(scData);

          } else {
              scData = dev.read();
              DPRINTF(RVCTRL, "SCReg: Reading %#x (ctrlRd %#x)\n",
                      scData, req);
          }
      } break;
      case CfgStat:     // Weird to write this
      default:
        warn("Tried to write RVIO at offset %#x (data %#x) that doesn't exist\n",
             daddr, pkt->get<uint32_t>());
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
RealViewCtrl::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(flags);
}

void
RealViewCtrl::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(flags);
}

void
RealViewCtrl::registerDevice(DeviceFunc func, uint8_t site, uint8_t pos,
                             uint8_t dcc, uint16_t dev,
                             Device *handler)
{
    CfgCtrlReg addr = 0;
    addr.func = func;
    addr.site = site;
    addr.pos = pos;
    addr.dcc = dcc;
    addr.dev = dev;

    if (devices.find(addr) != devices.end()) {
        fatal("Platform device dcc%d:site%d:pos%d:fn%d:dev%d "
              "already registered.",
              addr.dcc, addr.site, addr.pos, addr.func, addr.dev);
    }

    devices[addr] = handler;
}


RealViewOsc::RealViewOsc(RealViewOscParams *p)
    : ClockDomain(p, p->voltage_domain),
      RealViewCtrl::Device(*p->parent, RealViewCtrl::FUNC_OSC,
                           p->site, p->position, p->dcc, p->device)
{
    if (SimClock::Float::s  / p->freq > UINT32_MAX) {
        fatal("Oscillator frequency out of range: %f\n",
            SimClock::Float::s  / p->freq / 1E6);
    }

    _clockPeriod = p->freq;
}

void
RealViewOsc::startup()
{
    // Tell dependent object to set their clock frequency
    for (auto m : members)
        m->updateClockPeriod();
}

void
RealViewOsc::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_clockPeriod);
}

void
RealViewOsc::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_clockPeriod);
}

void
RealViewOsc::clockPeriod(Tick clock_period)
{
    panic_if(clock_period == 0, "%s has a clock period of zero\n", name());

    // Align all members to the current tick
    for (auto m : members)
        m->updateClockPeriod();

    _clockPeriod = clock_period;

    // inform any derived clocks they need to updated their period
    for (auto m : children)
        m->updateClockPeriod();
}

uint32_t
RealViewOsc::read() const
{
    const uint32_t freq(SimClock::Float::s / _clockPeriod);
    DPRINTF(RVCTRL, "Reading OSC frequency: %f MHz\n", freq / 1E6);
    return freq;
}

void
RealViewOsc::write(uint32_t freq)
{
    DPRINTF(RVCTRL, "Setting new OSC frequency: %f MHz\n", freq / 1E6);
    clockPeriod(SimClock::Float::s / freq);
}

uint32_t
RealViewTemperatureSensor::read() const
{
    // Temperature reported in uC
    ThermalModel * tm = system->getThermalModel();
    if (tm) {
        double t = tm->getTemp();
        if (t < 0)
            warn("Temperature below zero!\n");
        return fmax(0, t) * 1000000;
    }

    // Report a dummy 25 degrees temperature
    return 25000000;
}

RealViewCtrl *
RealViewCtrlParams::create()
{
    return new RealViewCtrl(this);
}

RealViewOsc *
RealViewOscParams::create()
{
    return new RealViewOsc(this);
}

RealViewTemperatureSensor *
RealViewTemperatureSensorParams::create()
{
    return new RealViewTemperatureSensor(this);
}
