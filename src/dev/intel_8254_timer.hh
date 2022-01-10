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

#ifndef __DEV_8254_HH__
#define __DEV_8254_HH__

#include <array>
#include <iostream>
#include <string>

#include "base/bitunion.hh"
#include "base/types.hh"
#include "base/trace.hh"
#include "debug/Intel8254Timer.hh"
#include "sim/eventq.hh"
#include "sim/serialize.hh"

namespace gem5
{

/** Programmable Interval Timer (Intel 8254) */
class Intel8254Timer : public EventManager
{
  protected:
    BitUnion8(CtrlReg)
        Bitfield<7, 6> sel;
        Bitfield<5, 4> rw;
        Bitfield<3, 1> mode;
        Bitfield<0> bcd;
    EndBitUnion(CtrlReg)

    BitUnion8(ReadBackCommandVal)
        Bitfield<4> status; // Active low.
        Bitfield<5> count; // Active low.
        SubBitUnion(select, 3, 1)
            Bitfield<3> cnt2;
            Bitfield<2> cnt1;
            Bitfield<1> cnt0;
        EndSubBitUnion(select)
    EndBitUnion(ReadBackCommandVal)

    enum SelectVal
    {
        SelectCounter0,
        SelectCounter1,
        SelectCounter2,
        ReadBackCommand
    };

    enum ReadWriteVal
    {
        LatchCommand,
        LsbOnly,
        MsbOnly,
        TwoPhase
    };

    enum ModeVal
    {
        InitTc,
        OneShot,
        RateGen,
        SquareWave,
        SoftwareStrobe,
        HardwareStrobe
    };

    /** Counter element for PIT */
    class Counter
    {
        /** Event for counter interrupt */
        class CounterEvent : public Event
        {
          private:
            /** Pointer back to Counter */
            Counter* counter;
            Tick interval;

          public:
            CounterEvent(Counter*);

            /** Event process */
            void process();

            /** Event description */
            virtual const char *description() const;

            friend class Counter;

            void setTo(int clocks);

            int clocksLeft();

            Tick getInterval();
        };

      private:
        std::string _name;
        const std::string &name() const { return _name; }

        unsigned int num;

        CounterEvent event;

        /** True after startup is called. */
        bool running;

        /** Initial count value */
        uint16_t initial_count;

        /** Latched count */
        uint16_t latched_count;

        /** Interrupt period */
        uint16_t period;

        /** When to start ticking */
        Tick offset;

        /** Current mode of operation */
        uint8_t mode;

        /** Output goes high when the counter reaches zero */
        bool output_high;

        /** State of the count latch */
        bool latch_on;

        /** Set of values for read_byte and write_byte */
        enum {LSB, MSB};

        /** Determine which byte of a 16-bit count value to read/write */
        uint8_t read_byte, write_byte;

        /** Pointer to container */
        Intel8254Timer *parent;

      public:
        Counter(Intel8254Timer *p, const std::string &name, unsigned int num);

        unsigned int index() const { return num; }

        /** Latch the current count (if one is not already latched) */
        void latchCount();

        /** Get the current count for this counter */
        int currentCount();

        /** Set the read/write mode */
        void setRW(int rw_val);

        /** Set operational mode */
        void setMode(int mode_val);

        /** Set count encoding */
        void setBCD(int bcd_val);

        /** Read a count byte */
        uint8_t read();

        /** Write a count byte */
        void write(const uint8_t data);

        /** Is the output high? */
        bool outputHigh();

        /**
         * Serialize this object to the given output stream.
         * @param base The base name of the counter object.
         * @param os   The stream to serialize to.
         */
        void serialize(const std::string &base, CheckpointOut &cp) const;

        /**
         * Reconstruct the state of this object from a checkpoint.
         * @param base The base name of the counter object.
         * @param cp The checkpoint use.
         * @param section The section name of this object
         */
        void unserialize(const std::string &base, CheckpointIn &cp);

        /** Start ticking */
        void startup();
    };

  protected:
    std::string _name;
    const std::string &name() const { return _name; }

    /** PIT has three seperate counters */
    std::array<Counter, 3> counters;

    virtual void
    counterInterrupt(unsigned int num)
    {
        DPRINTF(Intel8254Timer, "Timer interrupt from counter %d.\n", num);
    }

  public:

    virtual
    ~Intel8254Timer()
    {}

    Intel8254Timer(EventManager *em, const std::string &name);

    /** Write control word */
    void writeControl(const CtrlReg data);

    uint8_t
    readCounter(unsigned int num)
    {
        assert(num < 3);
        return counters[num].read();
    }

    void
    writeCounter(unsigned int num, const uint8_t data)
    {
        assert(num < 3);
        counters[num].write(data);
    }

    bool
    outputHigh(unsigned int num)
    {
        assert(num < 3);
        return counters[num].outputHigh();
    }

    /**
     * Serialize this object to the given output stream.
     * @param base The base name of the counter object.
     * @param os The stream to serialize to.
     */
    void serialize(const std::string &base, CheckpointOut &cp) const;

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param base The base name of the counter object.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(const std::string &base, CheckpointIn &cp);

    /** Start ticking */
    void startup();
};

} // namespace gem5

#endif // __DEV_8254_HH__
