/*
 * Copyright (c) 2004, 2005
 * The Regents of The University of Michigan
 * All Rights Reserved
 *
 * This code is part of the M5 simulator.
 *
 * Permission is granted to use, copy, create derivative works and
 * redistribute this software and such derivative works for any
 * purpose, so long as the copyright notice above, this grant of
 * permission, and the disclaimer below appear in all copies made; and
 * so long as the name of The University of Michigan is not used in
 * any advertising or publicity pertaining to the use or distribution
 * of this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
 * UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND
 * WITHOUT WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE REGENTS OF THE UNIVERSITY OF MICHIGAN SHALL NOT BE
 * LIABLE FOR ANY DAMAGES, INCLUDING DIRECT, SPECIAL, INDIRECT,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, WITH RESPECT TO ANY CLAIM
 * ARISING OUT OF OR IN CONNECTION WITH THE USE OF THE SOFTWARE, EVEN
 * IF IT HAS BEEN OR IS HEREAFTER ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGES.
 */

#include "dev/intel_8254_timer.hh"

#include "base/logging.hh"
#include "debug/Intel8254Timer.hh"
#include "sim/core.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

Intel8254Timer::Intel8254Timer(EventManager *em, const std::string &name)
    : EventManager(em),
      _name(name),
      counters{ { { this, name + ".counter0", 0 },
                  { this, name + ".counter1", 1 },
                  { this, name + ".counter2", 2 } } }
{}

void
Intel8254Timer::writeControl(const CtrlReg data)
{
    int sel = data.sel;

    if (sel == ReadBackCommand) {
        ReadBackCommandVal rb_val = static_cast<uint8_t>(data);

        panic_if(!rb_val.status,
                 "Latching the PIT status byte is not implemented.");

        if (!rb_val.count) {
            for (auto &counter : counters) {
                if (bits((uint8_t)rb_val.select, counter.index()))
                    counter.latchCount();
            }
        }
        return;
    }

    if (data.rw == LatchCommand) {
        counters[sel].latchCount();
    } else {
        counters[sel].setRW(data.rw);
        counters[sel].setMode(data.mode);
        counters[sel].setBCD(data.bcd);
    }
}

void
Intel8254Timer::serialize(const std::string &base, CheckpointOut &cp) const
{
    // serialize the counters
    counters[0].serialize(base + ".counter0", cp);
    counters[1].serialize(base + ".counter1", cp);
    counters[2].serialize(base + ".counter2", cp);
}

void
Intel8254Timer::unserialize(const std::string &base, CheckpointIn &cp)
{
    // unserialze the counters
    counters[0].unserialize(base + ".counter0", cp);
    counters[1].unserialize(base + ".counter1", cp);
    counters[2].unserialize(base + ".counter2", cp);
}

void
Intel8254Timer::startup()
{
    counters[0].startup();
    counters[1].startup();
    counters[2].startup();
}

Intel8254Timer::Counter::Counter(Intel8254Timer *p, const std::string &name,
                                 unsigned int _num)
    : _name(name),
      num(_num),
      event(this),
      running(false),
      initial_count(0),
      latched_count(0),
      period(0),
      mode(0),
      output_high(false),
      latch_on(false),
      read_byte(LSB),
      write_byte(LSB),
      parent(p)
{
    offset = period * event.getInterval();
}

void
Intel8254Timer::Counter::latchCount()
{
    // behave like a real latch
    if (!latch_on) {
        latch_on = true;
        read_byte = LSB;
        latched_count = currentCount();
    }
}

int
Intel8254Timer::Counter::currentCount()
{
    int clocks = event.clocksLeft();
    if (clocks == -1) {
        warn_once("Reading current count from inactive timer.\n");
        return 0;
    }
    if (mode == RateGen || mode == SquareWave)
        return clocks + 1;
    else
        return clocks;
}

uint8_t
Intel8254Timer::Counter::read()
{
    if (latch_on) {
        switch (read_byte) {
        case LSB:
            read_byte = MSB;
            return (uint8_t)latched_count;
            break;
        case MSB:
            read_byte = LSB;
            latch_on = false;
            return latched_count >> 8;
            break;
        default:
            panic("Shouldn't be here");
        }
    } else {
        uint16_t count = currentCount();
        switch (read_byte) {
        case LSB:
            read_byte = MSB;
            return (uint8_t)count;
            break;
        case MSB:
            read_byte = LSB;
            return count >> 8;
            break;
        default:
            panic("Shouldn't be here");
        }
    }
}

void
Intel8254Timer::Counter::write(const uint8_t data)
{
    switch (write_byte) {
    case LSB:
        initial_count = (initial_count & 0xFF00) | data;

        if (event.scheduled())
            parent->deschedule(event);
        output_high = false;
        write_byte = MSB;
        break;

    case MSB:
        initial_count = (initial_count & 0x00FF) | (data << 8);
        // In the RateGen or SquareWave modes, the timer wraps around and
        // triggers on a value of 1, not 0.
        if (mode == RateGen || mode == SquareWave)
            period = initial_count - 1;
        else
            period = initial_count;

        offset = period * event.getInterval();

        if (running && (period > 0))
            event.setTo(period);

        write_byte = LSB;
        break;
    }
}

void
Intel8254Timer::Counter::setRW(int rw_val)
{
    if (rw_val != TwoPhase)
        panic("Only LSB/MSB read/write is implemented.\n");
}

void
Intel8254Timer::Counter::setMode(int mode_val)
{
    if (mode_val != InitTc && mode_val != RateGen && mode_val != SquareWave)
        panic("PIT mode %#x is not implemented: \n", mode_val);

    mode = mode_val;
}

void
Intel8254Timer::Counter::setBCD(int bcd_val)
{
    if (bcd_val)
        panic("PITimer does not implement BCD counts.\n");
}

bool
Intel8254Timer::Counter::outputHigh()
{
    return output_high;
}

void
Intel8254Timer::Counter::serialize(const std::string &base,
                                   CheckpointOut &cp) const
{
    paramOut(cp, base + ".initial_count", initial_count);
    paramOut(cp, base + ".latched_count", latched_count);
    paramOut(cp, base + ".period", period);
    paramOut(cp, base + ".mode", mode);
    paramOut(cp, base + ".output_high", output_high);
    paramOut(cp, base + ".latch_on", latch_on);
    paramOut(cp, base + ".read_byte", read_byte);
    paramOut(cp, base + ".write_byte", write_byte);

    Tick event_tick_offset = 0;
    if (event.scheduled())
        event_tick_offset = event.when() - curTick();
    paramOut(cp, base + ".event_tick_offset", event_tick_offset);
}

void
Intel8254Timer::Counter::unserialize(const std::string &base, CheckpointIn &cp)
{
    paramIn(cp, base + ".initial_count", initial_count);
    paramIn(cp, base + ".latched_count", latched_count);
    paramIn(cp, base + ".period", period);
    paramIn(cp, base + ".mode", mode);
    paramIn(cp, base + ".output_high", output_high);
    paramIn(cp, base + ".latch_on", latch_on);
    paramIn(cp, base + ".read_byte", read_byte);
    paramIn(cp, base + ".write_byte", write_byte);

    Tick event_tick_offset = 0;
    assert(!event.scheduled());
    paramIn(cp, base + ".event_tick_offset", event_tick_offset);
    offset = event_tick_offset;
}

void
Intel8254Timer::Counter::startup()
{
    running = true;
    if ((period > 0) && (offset > 0)) {
        parent->schedule(event, curTick() + offset);
    }
}

Intel8254Timer::Counter::CounterEvent::CounterEvent(Counter *c_ptr)
{
    interval = (Tick)(sim_clock::as_float::s / 1193180.0);
    counter = c_ptr;
}

void
Intel8254Timer::Counter::CounterEvent::process()
{
    switch (counter->mode) {
    case InitTc:
        counter->output_high = true;
        break;
    case RateGen:
    case SquareWave:
        setTo(counter->period);
        break;
    default:
        panic("Unimplemented PITimer mode.\n");
    }
    counter->parent->counterInterrupt(counter->num);
}

void
Intel8254Timer::Counter::CounterEvent::setTo(int clocks)
{
    if (clocks == 0)
        panic("Timer can't be set to go off instantly.\n");
    DPRINTF(Intel8254Timer, "Timer set to curTick() + %d\n",
            clocks * interval);
    counter->parent->schedule(this, curTick() + clocks * interval);
}

int
Intel8254Timer::Counter::CounterEvent::clocksLeft()
{
    if (!scheduled())
        return -1;
    return (when() - curTick() + interval - 1) / interval;
}

const char *
Intel8254Timer::Counter::CounterEvent::description() const
{
    return "Intel 8254 Interval timer";
}

Tick
Intel8254Timer::Counter::CounterEvent::getInterval()
{
    return interval;
}

} // namespace gem5
