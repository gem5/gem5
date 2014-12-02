/*
 * Copyright (c) 2010,2013 ARM Limited
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

#include "base/trace.hh"
#include "debug/RVCTRL.hh"
#include "dev/arm/rv_ctrl.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

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
          // For now, model as much of the OSC regs (can't find docs) as Linux
          // seems to require (can't find docs); some clocks are deemed to be 0,
          // giving all kinds of /0 problems booting Linux 3.9.  Return a
          // vaguely plausible number within the range the device trees state:
          uint32_t data = pkt->get<uint32_t>();
          uint16_t dev = bits(data, 11, 0);
          uint8_t pos = bits(data, 15, 12);
          uint8_t site = bits(data, 17, 16);
          uint8_t func = bits(data, 25, 20);
          uint8_t dcc = bits(data, 29, 26);
          bool wr = bits(data, 30);
          bool start = bits(data, 31);

          if (start) {
              if (wr) {
                  warn_once("SCReg: Writing %#x to dcc%d:site%d:pos%d:fn%d:dev%d\n",
                          scData, dcc, site, pos, func, dev);
                  // Only really support reading, for now!
              } else {
                  // Only deal with function 1 (oscillators) so far!
                  if (dcc != 0 || pos != 0 || func != 1) {
                      warn("SCReg: read from unknown area "
                           "(dcc %d:site%d:pos%d:fn%d:dev%d)\n",
                           dcc, site, pos, func, dev);
                  } else {
                      switch (site) {
                        case 0: { // Motherboard regs
                            switch(dev) {
                              case 0: // MCC clk
                                scData = 25000000;
                                break;
                              case 1: // CLCD clk
                                scData = 25000000;
                                break;
                              case 2: // PeriphClk 24MHz
                                scData = 24000000;
                                break;
                              default:
                                scData = 0;
                                warn("SCReg: read from unknown dev %d "
                                     "(site%d:pos%d:fn%d)\n",
                                     dev, site, pos, func);
                            }
                        } break;
                        case 1: { // Coretile 1 regs
                            switch(dev) {
                              case 0: // CPU PLL ref
                                scData = 50000000;
                                break;
                              case 4: // Muxed AXI master clock
                                scData = 40000000;
                                break;
                              case 5: // HDLCD clk
                                scData = 50000000;
                                break;
                              case 6: // SMB clock
                                scData = 35000000;
                                break;
                              case 7: // SYS PLL (also used for pl011 UART!)
                                scData = 40000000;
                                break;
                              case 8: // DDR PLL 40MHz fixed
                                scData = 40000000;
                                break;
                              default:
                                scData = 0;
                                warn("SCReg: read from unknown dev %d "
                                     "(site%d:pos%d:fn%d)\n",
                                     dev, site, pos, func);
                            }
                        } break;
                        default:
                          warn("SCReg: Read from unknown site %d (pos%d:fn%d:dev%d)\n",
                               site, pos, func, dev);
                      }
                      DPRINTF(RVCTRL, "SCReg: Will read %#x (ctrlWr %#x)\n", scData, data);
                  }
              }
          } else {
              DPRINTF(RVCTRL, "SCReg: write %#x to ctrl but not starting\n", data);
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
RealViewCtrl::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(flags);
}

void
RealViewCtrl::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(flags);
}

RealViewCtrl *
RealViewCtrlParams::create()
{
    return new RealViewCtrl(this);
}
