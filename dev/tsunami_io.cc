/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * Tsunami I/O including PIC, PIT, RTC, DMA
 */

#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/tsunami_io.hh"
#include "dev/tsunami.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "sim/builder.hh"
#include "dev/tsunami_cchip.hh"
#include "dev/tsunamireg.h"
#include "dev/rtcreg.h"
#include "mem/functional/memory_control.hh"

using namespace std;

#define UNIX_YEAR_OFFSET 52

struct tm TsunamiIO::tm = { 0 };

// Timer Event for Periodic interrupt of RTC
TsunamiIO::RTCEvent::RTCEvent(Tsunami* t, Tick i)
    : Event(&mainEventQueue), tsunami(t), interval(i)
{
    DPRINTF(MC146818, "RTC Event Initilizing\n");
    intr_count = 0;
    schedule(curTick + interval);
}

void
TsunamiIO::RTCEvent::process()
{
    DPRINTF(MC146818, "RTC Timer Interrupt\n");
    schedule(curTick + interval);
    //Actually interrupt the processor here
    tsunami->cchip->postRTC();

    if (intr_count == 1023)
        tm.tm_sec = (tm.tm_sec + 1) % 60;

    intr_count = (intr_count + 1) % 1024;

}

const char *
TsunamiIO::RTCEvent::description()
{
    return "tsunami RTC interrupt";
}

void
TsunamiIO::RTCEvent::serialize(std::ostream &os)
{
    Tick time = when();
    SERIALIZE_SCALAR(time);
}

void
TsunamiIO::RTCEvent::unserialize(Checkpoint *cp, const std::string &section)
{
    Tick time;
    UNSERIALIZE_SCALAR(time);
    reschedule(time);
}


// Timer Event for PIT Timers
TsunamiIO::ClockEvent::ClockEvent()
    : Event(&mainEventQueue)
{
    /* This is the PIT Tick Rate. A constant for the 8254 timer. The
     * Tsunami platform has one of these cycle counters on the Cypress
     * South Bridge and it is used by linux for estimating the cycle
     * frequency of the machine it is running on. --Ali
     */
    interval = (Tick)(Clock::Float::s / 1193180.0);

    DPRINTF(Tsunami, "Clock Event Initilizing\n");
    mode = 0;

    current_count = 0;
    latched_count = 0;
    latch_on = false;
    read_byte = READ_LSB;
}

void
TsunamiIO::ClockEvent::process()
{
    DPRINTF(Tsunami, "Timer Interrupt\n");
    if (mode == 0)
        status = 0x20; // set bit that linux is looking for
    else
        schedule(curTick + interval);

    current_count--; //decrement count
}

void
TsunamiIO::ClockEvent::Program(int count)
{
    DPRINTF(Tsunami, "Timer set to curTick + %d\n", count * interval);
    schedule(curTick + count * interval);
    status = 0;

    current_count = (uint16_t)count;
}

const char *
TsunamiIO::ClockEvent::description()
{
    return "tsunami 8254 Interval timer";
}

void
TsunamiIO::ClockEvent::ChangeMode(uint8_t md)
{
    mode = md;
}

uint8_t
TsunamiIO::ClockEvent::Status()
{
    return status;
}

void
TsunamiIO::ClockEvent::LatchCount()
{
    // behave like a real latch
    if(!latch_on) {
        latch_on = true;
        read_byte = READ_LSB;
        latched_count = current_count;
    }
}

uint8_t
TsunamiIO::ClockEvent::Read()
{
   uint8_t result = 0;

    if(latch_on) {
        switch (read_byte) {
          case READ_LSB:
            read_byte = READ_MSB;
            result = (uint8_t)latched_count;
            break;
          case READ_MSB:
            read_byte = READ_LSB;
            latch_on = false;
            result = latched_count >> 8;
            break;
        }
    } else {
        switch (read_byte) {
          case READ_LSB:
            read_byte = READ_MSB;
            result = (uint8_t)current_count;
            break;
          case READ_MSB:
            read_byte = READ_LSB;
            result = current_count >> 8;
            break;
        }
    }

    return result;
}


void
TsunamiIO::ClockEvent::serialize(std::ostream &os)
{
    Tick time = scheduled() ? when() : 0;
    SERIALIZE_SCALAR(time);
    SERIALIZE_SCALAR(status);
    SERIALIZE_SCALAR(mode);
    SERIALIZE_SCALAR(interval);
}

void
TsunamiIO::ClockEvent::unserialize(Checkpoint *cp, const std::string &section)
{
    Tick time;
    UNSERIALIZE_SCALAR(time);
    UNSERIALIZE_SCALAR(status);
    UNSERIALIZE_SCALAR(mode);
    UNSERIALIZE_SCALAR(interval);
    if (time)
        schedule(time);
}

TsunamiIO::TsunamiIO(const string &name, Tsunami *t, time_t init_time,
                     Addr a, MemoryController *mmu, HierParams *hier, Bus *bus,
                     Tick pio_latency, Tick ci)
    : PioDevice(name, t), addr(a), clockInterval(ci), tsunami(t), rtc(t, ci)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                       &TsunamiIO::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * bus->clockRate;
    }

    // set the back pointer from tsunami to myself
    tsunami->io = this;

    timerData = 0;
    set_time(init_time == 0 ? time(NULL) : init_time);
    uip = 1;
    picr = 0;
    picInterrupting = false;
}

Tick
TsunamiIO::frequency() const
{
    return Clock::Frequency / clockInterval;
}

void
TsunamiIO::set_time(time_t t)
{
    gmtime_r(&t, &tm);
    DPRINTFN("Real-time clock set to %s", asctime(&tm));
}

Fault
TsunamiIO::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(Tsunami, "io read  va=%#x size=%d IOPorrt=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask)) + 0x20;


    switch(req->size) {
      case sizeof(uint8_t):
        switch(daddr) {
          // PIC1 mask read
          case TSDEV_PIC1_MASK:
            *(uint8_t*)data = ~mask1;
            return No_Fault;
          case TSDEV_PIC2_MASK:
            *(uint8_t*)data = ~mask2;
            return No_Fault;
          case TSDEV_PIC1_ISR:
              // !!! If this is modified 64bit case needs to be too
              // Pal code has to do a 64 bit physical read because there is
              // no load physical byte instruction
              *(uint8_t*)data = picr;
              return No_Fault;
          case TSDEV_PIC2_ISR:
              // PIC2 not implemnted... just return 0
              *(uint8_t*)data = 0x00;
              return No_Fault;
          case TSDEV_TMR_CTL:
            *(uint8_t*)data = timer2.Status();
            return No_Fault;
          case TSDEV_TMR0_DATA:
            *(uint8_t *)data = timer0.Read();
            return No_Fault;
          case TSDEV_RTC_DATA:
            switch(RTCAddress) {
              case RTC_CNTRL_REGA:
                *(uint8_t*)data = uip << 7 | 0x26;
                uip = !uip;
                return No_Fault;
              case RTC_CNTRL_REGB:
                // DM and 24/12 and UIE
                *(uint8_t*)data = 0x46;
                return No_Fault;
              case RTC_CNTRL_REGC:
                // If we want to support RTC user access in linux
                // This won't work, but for now it's fine
                *(uint8_t*)data = 0x00;
                return No_Fault;
              case RTC_CNTRL_REGD:
                panic("RTC Control Register D not implemented");
              case RTC_SEC:
                *(uint8_t *)data = tm.tm_sec;
                return No_Fault;
              case RTC_MIN:
                *(uint8_t *)data = tm.tm_min;
                return No_Fault;
              case RTC_HR:
                *(uint8_t *)data = tm.tm_hour;
                return No_Fault;
              case RTC_DOW:
                *(uint8_t *)data = tm.tm_wday;
                return No_Fault;
              case RTC_DOM:
                *(uint8_t *)data = tm.tm_mday;
                return No_Fault;
              case RTC_MON:
                *(uint8_t *)data = tm.tm_mon + 1;
                return No_Fault;
              case RTC_YEAR:
                *(uint8_t *)data = tm.tm_year - UNIX_YEAR_OFFSET;
                return No_Fault;
              default:
                panic("Unknown RTC Address\n");
            }

          /* Added for keyboard reads */
          case TSDEV_KBD:
            *(uint8_t *)data = 0x00;
            return No_Fault;
          /* Added for ATA PCI DMA */
          case ATA_PCI_DMA:
            *(uint8_t *)data = 0x00;
            return No_Fault;
          default:
            panic("I/O Read - va%#x size %d\n", req->vaddr, req->size);
        }
      case sizeof(uint16_t):
      case sizeof(uint32_t):
        panic("I/O Read - invalid size - va %#x size %d\n",
              req->vaddr, req->size);

      case sizeof(uint64_t):
       switch(daddr) {
          case TSDEV_PIC1_ISR:
              // !!! If this is modified 8bit case needs to be too
              // Pal code has to do a 64 bit physical read because there is
              // no load physical byte instruction
              *(uint64_t*)data = (uint64_t)picr;
              return No_Fault;
          default:
              panic("I/O Read - invalid size - va %#x size %d\n",
                    req->vaddr, req->size);
       }

      default:
        panic("I/O Read - invalid size - va %#x size %d\n",
              req->vaddr, req->size);
    }
    panic("I/O Read - va%#x size %d\n", req->vaddr, req->size);

    return No_Fault;
}

Fault
TsunamiIO::write(MemReqPtr &req, const uint8_t *data)
{

#if TRACING_ON
    uint8_t dt = *(uint8_t*)data;
    uint64_t dt64 = dt;
#endif

    DPRINTF(Tsunami, "io write - va=%#x size=%d IOPort=%#x Data=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff, dt64);

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask)) + 0x20;

    switch(req->size) {
      case sizeof(uint8_t):
        switch(daddr) {
          case TSDEV_PIC1_MASK:
            mask1 = ~(*(uint8_t*)data);
            if ((picr & mask1) && !picInterrupting) {
                picInterrupting = true;
                tsunami->cchip->postDRIR(55);
                DPRINTF(Tsunami, "posting pic interrupt to cchip\n");
            }
            if ((!(picr & mask1)) && picInterrupting) {
                picInterrupting = false;
                tsunami->cchip->clearDRIR(55);
                DPRINTF(Tsunami, "clearing pic interrupt\n");
            }
            return No_Fault;
          case TSDEV_PIC2_MASK:
            mask2 = *(uint8_t*)data;
            //PIC2 Not implemented to interrupt
            return No_Fault;
          case TSDEV_PIC1_ACK:
            // clear the interrupt on the PIC
            picr &= ~(1 << (*(uint8_t*)data & 0xF));
            if (!(picr & mask1))
                tsunami->cchip->clearDRIR(55);
            return No_Fault;
          case TSDEV_PIC2_ACK:
            return No_Fault;
          case TSDEV_DMA1_RESET:
            return No_Fault;
          case TSDEV_DMA2_RESET:
            return No_Fault;
          case TSDEV_DMA1_MODE:
            mode1 = *(uint8_t*)data;
            return No_Fault;
          case TSDEV_DMA2_MODE:
            mode2 = *(uint8_t*)data;
            return No_Fault;
          case TSDEV_DMA1_MASK:
          case TSDEV_DMA2_MASK:
            return No_Fault;
          case TSDEV_TMR_CTL:
            return No_Fault;
          case TSDEV_TMR2_CTL:
            switch((*(uint8_t*)data >> 4) & 0x3) {
              case 0x0:
                switch(*(uint8_t*)data >> 6) {
                  case 0:
                    timer0.LatchCount();
                    break;
                  case 2:
                    timer2.LatchCount();
                    break;
                  default:
                    panic("Read Back Command not implemented\n");
                }
                break;
              case 0x3:
                break;
              default:
                panic("Only L/M write and Counter-Latch read supported\n");
            }

            switch(*(uint8_t*)data >> 6) {
              case 0:
                timer0.ChangeMode((*(uint8_t*)data & 0xF) >> 1);
                break;
              case 2:
                timer2.ChangeMode((*(uint8_t*)data & 0xF) >> 1);
                break;
              default:
                panic("Read Back Command not implemented\n");
            }
            return No_Fault;
          case TSDEV_TMR2_DATA:
            /* two writes before we actually start the Timer
               so I set a flag in the timerData */
            if(timerData & 0x1000) {
                timerData &= 0x1000;
                timerData += *(uint8_t*)data << 8;
                timer2.Program(timerData);
            } else {
                timerData = *(uint8_t*)data;
                timerData |= 0x1000;
            }
            return No_Fault;
          case TSDEV_TMR0_DATA:
            /* two writes before we actually start the Timer
               so I set a flag in the timerData */
            if(timerData & 0x1000) {
                timerData &= 0x1000;
                timerData += *(uint8_t*)data << 8;
                timer0.Program(timerData);
            } else {
                timerData = *(uint8_t*)data;
                timerData |= 0x1000;
            }
            return No_Fault;
          case TSDEV_RTC_ADDR:
            RTCAddress = *(uint8_t*)data;
            return No_Fault;
          case TSDEV_KBD:
            return No_Fault;
          case TSDEV_RTC_DATA:
            switch(RTCAddress) {
              case RTC_CNTRL_REGA:
                return No_Fault;
              case RTC_CNTRL_REGB:
                return No_Fault;
              case RTC_CNTRL_REGC:
                return No_Fault;
              case RTC_CNTRL_REGD:
                return No_Fault;
              case RTC_SEC:
                tm.tm_sec = *(uint8_t *)data;
                return No_Fault;
              case RTC_MIN:
                tm.tm_min = *(uint8_t *)data;
                return No_Fault;
              case RTC_HR:
                tm.tm_hour = *(uint8_t *)data;
                return No_Fault;
              case RTC_DOW:
                tm.tm_wday = *(uint8_t *)data;
                return No_Fault;
              case RTC_DOM:
                tm.tm_mday = *(uint8_t *)data;
                return No_Fault;
              case RTC_MON:
                 tm.tm_mon = *(uint8_t *)data - 1;
                return No_Fault;
              case RTC_YEAR:
                tm.tm_year = *(uint8_t *)data + UNIX_YEAR_OFFSET;
                return No_Fault;
            //panic("RTC Write not implmented (rtc.o won't work)\n");
            }
          default:
            panic("I/O Write - va%#x size %d\n", req->vaddr, req->size);
        }
      case sizeof(uint16_t):
      case sizeof(uint32_t):
      case sizeof(uint64_t):
      default:
        panic("I/O Write - invalid size - va %#x size %d\n",
              req->vaddr, req->size);
    }


    return No_Fault;
}

void
TsunamiIO::postPIC(uint8_t bitvector)
{
    //PIC2 Is not implemented, because nothing of interest there
    picr |= bitvector;
    if (picr & mask1) {
        tsunami->cchip->postDRIR(55);
        DPRINTF(Tsunami, "posting pic interrupt to cchip\n");
    }
}

void
TsunamiIO::clearPIC(uint8_t bitvector)
{
    //PIC2 Is not implemented, because nothing of interest there
    picr &= ~bitvector;
    if (!(picr & mask1)) {
        tsunami->cchip->clearDRIR(55);
        DPRINTF(Tsunami, "clearing pic interrupt to cchip\n");
    }
}

Tick
TsunamiIO::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}

void
TsunamiIO::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(timerData);
    SERIALIZE_SCALAR(uip);
    SERIALIZE_SCALAR(mask1);
    SERIALIZE_SCALAR(mask2);
    SERIALIZE_SCALAR(mode1);
    SERIALIZE_SCALAR(mode2);
    SERIALIZE_SCALAR(picr);
    SERIALIZE_SCALAR(picInterrupting);
    SERIALIZE_SCALAR(RTCAddress);

    // Serialize the timers
    nameOut(os, csprintf("%s.timer0", name()));
    timer0.serialize(os);
    nameOut(os, csprintf("%s.timer2", name()));
    timer2.serialize(os);
    nameOut(os, csprintf("%s.rtc", name()));
    rtc.serialize(os);
}

void
TsunamiIO::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(timerData);
    UNSERIALIZE_SCALAR(uip);
    UNSERIALIZE_SCALAR(mask1);
    UNSERIALIZE_SCALAR(mask2);
    UNSERIALIZE_SCALAR(mode1);
    UNSERIALIZE_SCALAR(mode2);
    UNSERIALIZE_SCALAR(picr);
    UNSERIALIZE_SCALAR(picInterrupting);
    UNSERIALIZE_SCALAR(RTCAddress);

    // Unserialize the timers
    timer0.unserialize(cp, csprintf("%s.timer0", section));
    timer2.unserialize(cp, csprintf("%s.timer2", section));
    rtc.unserialize(cp, csprintf("%s.rtc", section));
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

    SimObjectParam<Tsunami *> tsunami;
    Param<time_t> time;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<Bus*> io_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;
    Param<Tick> frequency;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(time, "System time to use (0 for actual time"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams),
    INIT_PARAM(frequency, "clock interrupt frequency")

END_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

CREATE_SIM_OBJECT(TsunamiIO)
{
    return new TsunamiIO(getInstanceName(), tsunami, time,  addr, mmu, hier,
                         io_bus, pio_latency, frequency);
}

REGISTER_SIM_OBJECT("TsunamiIO", TsunamiIO)
