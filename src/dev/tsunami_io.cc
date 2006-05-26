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
#include "dev/pitreg.h"
#include "dev/rtcreg.h"
#include "dev/tsunami_cchip.hh"
#include "dev/tsunami.hh"
#include "dev/tsunami_io.hh"
#include "dev/tsunamireg.h"
#include "mem/port.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
//Should this be AlphaISA?
using namespace TheISA;

TsunamiIO::RTC::RTC(const string &name, Tsunami* t, Tick i)
    : _name(name), event(t, i), addr(0)
{
    memset(clock_data, 0, sizeof(clock_data));
    stat_regA = RTCA_32768HZ | RTCA_1024HZ;
    stat_regB = RTCB_PRDC_IE |RTCB_BIN | RTCB_24HR;
}

void
TsunamiIO::RTC::set_time(time_t t)
{
    struct tm tm;
    gmtime_r(&t, &tm);

    sec = tm.tm_sec;
    min = tm.tm_min;
    hour = tm.tm_hour;
    wday = tm.tm_wday + 1;
    mday = tm.tm_mday;
    mon = tm.tm_mon + 1;
    year = tm.tm_year;

    DPRINTFN("Real-time clock set to %s", asctime(&tm));
}

void
TsunamiIO::RTC::writeAddr(const uint8_t data)
{
    if (data <= RTC_STAT_REGD)
        addr = data;
    else
        panic("RTC addresses over 0xD are not implemented.\n");
}

void
TsunamiIO::RTC::writeData(const uint8_t data)
{
    if (addr < RTC_STAT_REGA)
        clock_data[addr] = data;
    else {
        switch (addr) {
          case RTC_STAT_REGA:
            if (data != (RTCA_32768HZ | RTCA_1024HZ))
                panic("Unimplemented RTC register A value write!\n");
            stat_regA = data;
            break;
          case RTC_STAT_REGB:
            if ((data & ~(RTCB_PRDC_IE | RTCB_SQWE)) != (RTCB_BIN | RTCB_24HR))
                panic("Write to RTC reg B bits that are not implemented!\n");

            if (data & RTCB_PRDC_IE) {
                if (!event.scheduled())
                    event.scheduleIntr();
            } else {
                if (event.scheduled())
                    event.deschedule();
            }
            stat_regB = data;
            break;
          case RTC_STAT_REGC:
          case RTC_STAT_REGD:
            panic("RTC status registers C and D are not implemented.\n");
            break;
        }
    }
}

void
TsunamiIO::RTC::readData(uint8_t *data)
{
    if (addr < RTC_STAT_REGA)
        *data = clock_data[addr];
    else {
        switch (addr) {
          case RTC_STAT_REGA:
            // toggle UIP bit for linux
            stat_regA ^= RTCA_UIP;
            *data = stat_regA;
            break;
          case RTC_STAT_REGB:
            *data = stat_regB;
            break;
          case RTC_STAT_REGC:
          case RTC_STAT_REGD:
            *data = 0x00;
            break;
        }
    }
}

void
TsunamiIO::RTC::serialize(const string &base, ostream &os)
{
    paramOut(os, base + ".addr", addr);
    arrayParamOut(os, base + ".clock_data", clock_data, sizeof(clock_data));
    paramOut(os, base + ".stat_regA", stat_regA);
    paramOut(os, base + ".stat_regB", stat_regB);
}

void
TsunamiIO::RTC::unserialize(const string &base, Checkpoint *cp,
                            const string &section)
{
    paramIn(cp, section, base + ".addr", addr);
    arrayParamIn(cp, section, base + ".clock_data", clock_data,
                 sizeof(clock_data));
    paramIn(cp, section, base + ".stat_regA", stat_regA);
    paramIn(cp, section, base + ".stat_regB", stat_regB);

    // We're not unserializing the event here, but we need to
    // rescehedule the event since curTick was moved forward by the
    // checkpoint
    event.reschedule(curTick + event.interval);
}

TsunamiIO::RTC::RTCEvent::RTCEvent(Tsunami*t, Tick i)
    : Event(&mainEventQueue), tsunami(t), interval(i)
{
    DPRINTF(MC146818, "RTC Event Initilizing\n");
    schedule(curTick + interval);
}

void
TsunamiIO::RTC::RTCEvent::scheduleIntr()
{
  schedule(curTick + interval);
}

void
TsunamiIO::RTC::RTCEvent::process()
{
    DPRINTF(MC146818, "RTC Timer Interrupt\n");
    schedule(curTick + interval);
    //Actually interrupt the processor here
    tsunami->cchip->postRTC();
}

const char *
TsunamiIO::RTC::RTCEvent::description()
{
    return "tsunami RTC interrupt";
}

TsunamiIO::PITimer::PITimer(const string &name)
    : _name(name), counter0(name + ".counter0"), counter1(name + ".counter1"),
      counter2(name + ".counter2")
{
    counter[0] = &counter0;
    counter[1] = &counter0;
    counter[2] = &counter0;
}

void
TsunamiIO::PITimer::writeControl(const uint8_t data)
{
    int rw;
    int sel;

    sel = GET_CTRL_SEL(data);

    if (sel == PIT_READ_BACK)
       panic("PITimer Read-Back Command is not implemented.\n");

    rw = GET_CTRL_RW(data);

    if (rw == PIT_RW_LATCH_COMMAND)
        counter[sel]->latchCount();
    else {
        counter[sel]->setRW(rw);
        counter[sel]->setMode(GET_CTRL_MODE(data));
        counter[sel]->setBCD(GET_CTRL_BCD(data));
    }
}

void
TsunamiIO::PITimer::serialize(const string &base, ostream &os)
{
    // serialize the counters
    counter0.serialize(base + ".counter0", os);
    counter1.serialize(base + ".counter1", os);
    counter2.serialize(base + ".counter2", os);
}

void
TsunamiIO::PITimer::unserialize(const string &base, Checkpoint *cp,
                                const string &section)
{
    // unserialze the counters
    counter0.unserialize(base + ".counter0", cp, section);
    counter1.unserialize(base + ".counter1", cp, section);
    counter2.unserialize(base + ".counter2", cp, section);
}

TsunamiIO::PITimer::Counter::Counter(const string &name)
    : _name(name), event(this), count(0), latched_count(0), period(0),
      mode(0), output_high(false), latch_on(false), read_byte(LSB),
      write_byte(LSB)
{

}

void
TsunamiIO::PITimer::Counter::latchCount()
{
    // behave like a real latch
    if(!latch_on) {
        latch_on = true;
        read_byte = LSB;
        latched_count = count;
    }
}

void
TsunamiIO::PITimer::Counter::read(uint8_t *data)
{
    if (latch_on) {
        switch (read_byte) {
          case LSB:
            read_byte = MSB;
            *data = (uint8_t)latched_count;
            break;
          case MSB:
            read_byte = LSB;
            latch_on = false;
            *data = latched_count >> 8;
            break;
        }
    } else {
        switch (read_byte) {
          case LSB:
            read_byte = MSB;
            *data = (uint8_t)count;
            break;
          case MSB:
            read_byte = LSB;
            *data = count >> 8;
            break;
        }
    }
}

void
TsunamiIO::PITimer::Counter::write(const uint8_t data)
{
    switch (write_byte) {
      case LSB:
        count = (count & 0xFF00) | data;

        if (event.scheduled())
          event.deschedule();
        output_high = false;
        write_byte = MSB;
        break;

      case MSB:
        count = (count & 0x00FF) | (data << 8);
        period = count;

        if (period > 0) {
            DPRINTF(Tsunami, "Timer set to curTick + %d\n",
                    count * event.interval);
            event.schedule(curTick + count * event.interval);
        }
        write_byte = LSB;
        break;
    }
}

void
TsunamiIO::PITimer::Counter::setRW(int rw_val)
{
    if (rw_val != PIT_RW_16BIT)
        panic("Only LSB/MSB read/write is implemented.\n");
}

void
TsunamiIO::PITimer::Counter::setMode(int mode_val)
{
    if(mode_val != PIT_MODE_INTTC && mode_val != PIT_MODE_RATEGEN &&
       mode_val != PIT_MODE_SQWAVE)
        panic("PIT mode %#x is not implemented: \n", mode_val);

    mode = mode_val;
}

void
TsunamiIO::PITimer::Counter::setBCD(int bcd_val)
{
    if (bcd_val != PIT_BCD_FALSE)
        panic("PITimer does not implement BCD counts.\n");
}

bool
TsunamiIO::PITimer::Counter::outputHigh()
{
    return output_high;
}

void
TsunamiIO::PITimer::Counter::serialize(const string &base, ostream &os)
{
    paramOut(os, base + ".count", count);
    paramOut(os, base + ".latched_count", latched_count);
    paramOut(os, base + ".period", period);
    paramOut(os, base + ".mode", mode);
    paramOut(os, base + ".output_high", output_high);
    paramOut(os, base + ".latch_on", latch_on);
    paramOut(os, base + ".read_byte", read_byte);
    paramOut(os, base + ".write_byte", write_byte);

    Tick event_tick = 0;
    if (event.scheduled())
        event_tick = event.when();
    paramOut(os, base + ".event_tick", event_tick);
}

void
TsunamiIO::PITimer::Counter::unserialize(const string &base, Checkpoint *cp,
                                         const string &section)
{
    paramIn(cp, section, base + ".count", count);
    paramIn(cp, section, base + ".latched_count", latched_count);
    paramIn(cp, section, base + ".period", period);
    paramIn(cp, section, base + ".mode", mode);
    paramIn(cp, section, base + ".output_high", output_high);
    paramIn(cp, section, base + ".latch_on", latch_on);
    paramIn(cp, section, base + ".read_byte", read_byte);
    paramIn(cp, section, base + ".write_byte", write_byte);

    Tick event_tick;
    paramIn(cp, section, base + ".event_tick", event_tick);
    if (event_tick)
        event.schedule(event_tick);
}

TsunamiIO::PITimer::Counter::CounterEvent::CounterEvent(Counter* c_ptr)
    : Event(&mainEventQueue)
{
    interval = (Tick)(Clock::Float::s / 1193180.0);
    counter = c_ptr;
}

void
TsunamiIO::PITimer::Counter::CounterEvent::process()
{
    DPRINTF(Tsunami, "Timer Interrupt\n");
    switch (counter->mode) {
      case PIT_MODE_INTTC:
        counter->output_high = true;
      case PIT_MODE_RATEGEN:
      case PIT_MODE_SQWAVE:
        break;
      default:
        panic("Unimplemented PITimer mode.\n");
    }
}

const char *
TsunamiIO::PITimer::Counter::CounterEvent::description()
{
    return "tsunami 8254 Interval timer";
}

TsunamiIO::TsunamiIO(Params *p)
    : BasicPioDevice(p), tsunami(p->tsunami), pitimer(p->name + "pitimer"),
      rtc(p->name + ".rtc", p->tsunami, p->frequency)
{
    pioSize = 0xff;

    // set the back pointer from tsunami to myself
    tsunami->io = this;

    timerData = 0;
    rtc.set_time(p->init_time == 0 ? time(NULL) : p->init_time);
    picr = 0;
    picInterrupting = false;
}

Tick
TsunamiIO::frequency() const
{
    return Clock::Frequency / params()->frequency;
}

Tick
TsunamiIO::read(Packet *pkt)
{
    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    pkt->time += pioDelay;
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Tsunami, "io read  va=%#x size=%d IOPorrt=%#x\n", pkt->getAddr(),
            pkt->getSize(), daddr);

    pkt->allocate();

    if (pkt->getSize() == sizeof(uint8_t)) {
        switch(daddr) {
          // PIC1 mask read
          case TSDEV_PIC1_MASK:
            pkt->set(~mask1);
            break;
          case TSDEV_PIC2_MASK:
            pkt->set(~mask2);
            break;
          case TSDEV_PIC1_ISR:
              // !!! If this is modified 64bit case needs to be too
              // Pal code has to do a 64 bit physical read because there is
              // no load physical byte instruction
              pkt->set(picr);
              break;
          case TSDEV_PIC2_ISR:
              // PIC2 not implemnted... just return 0
              pkt->set(0x00);
              break;
          case TSDEV_TMR0_DATA:
            pitimer.counter0.read(pkt->getPtr<uint8_t>());
            break;
          case TSDEV_TMR1_DATA:
            pitimer.counter1.read(pkt->getPtr<uint8_t>());
            break;
          case TSDEV_TMR2_DATA:
            pitimer.counter2.read(pkt->getPtr<uint8_t>());
            break;
          case TSDEV_RTC_DATA:
            rtc.readData(pkt->getPtr<uint8_t>());
            break;
          case TSDEV_CTRL_PORTB:
            if (pitimer.counter2.outputHigh())
                pkt->set(PORTB_SPKR_HIGH);
            else
                pkt->set(0x00);
            break;
          default:
            panic("I/O Read - va%#x size %d\n", pkt->getAddr(), pkt->getSize());
        }
    } else if (pkt->getSize() == sizeof(uint64_t)) {
        if (daddr == TSDEV_PIC1_ISR)
            pkt->set<uint64_t>(picr);
        else
           panic("I/O Read - invalid addr - va %#x size %d\n",
                   pkt->getAddr(), pkt->getSize());
    } else {
       panic("I/O Read - invalid size - va %#x size %d\n", pkt->getAddr(), pkt->getSize());
    }
    pkt->result = Packet::Success;
    return pioDelay;
}

Tick
TsunamiIO::write(Packet *pkt)
{
    pkt->time += pioDelay;

    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Tsunami, "io write - va=%#x size=%d IOPort=%#x Data=%#x\n",
            pkt->getAddr(), pkt->getSize(), pkt->getAddr() & 0xfff, (uint32_t)pkt->get<uint8_t>());

    assert(pkt->getSize() == sizeof(uint8_t));

    switch(daddr) {
      case TSDEV_PIC1_MASK:
        mask1 = ~(pkt->get<uint8_t>());
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
        break;
      case TSDEV_PIC2_MASK:
        mask2 = pkt->get<uint8_t>();
        //PIC2 Not implemented to interrupt
        break;
      case TSDEV_PIC1_ACK:
        // clear the interrupt on the PIC
        picr &= ~(1 << (pkt->get<uint8_t>() & 0xF));
        if (!(picr & mask1))
            tsunami->cchip->clearDRIR(55);
        break;
      case TSDEV_DMA1_MODE:
        mode1 = pkt->get<uint8_t>();
        break;
      case TSDEV_DMA2_MODE:
        mode2 = pkt->get<uint8_t>();
        break;
      case TSDEV_TMR0_DATA:
        pitimer.counter0.write(pkt->get<uint8_t>());
        break;
      case TSDEV_TMR1_DATA:
        pitimer.counter1.write(pkt->get<uint8_t>());
        break;
      case TSDEV_TMR2_DATA:
        pitimer.counter2.write(pkt->get<uint8_t>());
        break;
      case TSDEV_TMR_CTRL:
        pitimer.writeControl(pkt->get<uint8_t>());
        break;
      case TSDEV_RTC_ADDR:
        rtc.writeAddr(pkt->get<uint8_t>());
        break;
      case TSDEV_RTC_DATA:
        rtc.writeData(pkt->get<uint8_t>());
        break;
      case TSDEV_KBD:
      case TSDEV_DMA1_CMND:
      case TSDEV_DMA2_CMND:
      case TSDEV_DMA1_MMASK:
      case TSDEV_DMA2_MMASK:
      case TSDEV_PIC2_ACK:
      case TSDEV_DMA1_RESET:
      case TSDEV_DMA2_RESET:
      case TSDEV_DMA1_MASK:
      case TSDEV_DMA2_MASK:
      case TSDEV_CTRL_PORTB:
        break;
      default:
        panic("I/O Write - va%#x size %d data %#x\n", pkt->getAddr(), pkt->getSize(), pkt->get<uint8_t>());
    }

    pkt->result = Packet::Success;
    return pioDelay;
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

void
TsunamiIO::serialize(ostream &os)
{
    SERIALIZE_SCALAR(timerData);
    SERIALIZE_SCALAR(mask1);
    SERIALIZE_SCALAR(mask2);
    SERIALIZE_SCALAR(mode1);
    SERIALIZE_SCALAR(mode2);
    SERIALIZE_SCALAR(picr);
    SERIALIZE_SCALAR(picInterrupting);

    // Serialize the timers
    pitimer.serialize("pitimer", os);
    rtc.serialize("rtc", os);
}

void
TsunamiIO::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_SCALAR(timerData);
    UNSERIALIZE_SCALAR(mask1);
    UNSERIALIZE_SCALAR(mask2);
    UNSERIALIZE_SCALAR(mode1);
    UNSERIALIZE_SCALAR(mode2);
    UNSERIALIZE_SCALAR(picr);
    UNSERIALIZE_SCALAR(picInterrupting);

    // Unserialize the timers
    pitimer.unserialize("pitimer", cp, section);
    rtc.unserialize("rtc", cp, section);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    Param<Tick> frequency;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;
    Param<time_t> time;
    SimObjectParam<Tsunami *> tsunami;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(frequency, "clock interrupt frequency"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(time, "System time to use (0 for actual time"),
    INIT_PARAM(tsunami, "Tsunami")

END_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

CREATE_SIM_OBJECT(TsunamiIO)
{
    TsunamiIO::Params *p = new TsunamiIO::Params;
    p->frequency = frequency;
    p->name = getInstanceName();
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    p->init_time = time;
    p->tsunami = tsunami;
    return new TsunamiIO(p);
}

REGISTER_SIM_OBJECT("TsunamiIO", TsunamiIO)
