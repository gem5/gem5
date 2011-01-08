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
 *
 * Authors: Ali Saidi
 *          Andrew Schultz
 *          Miguel Serrano
 */

#ifndef __DEV_MC146818_HH__
#define __DEV_MC146818_HH__

#include "base/range.hh"
#include "sim/eventq.hh"

/** Real-Time Clock (MC146818) */
class MC146818 : public EventManager
{
  protected:
    virtual void handleEvent()
    {
        warn("No RTC event handler defined.\n");
    }

  private:
    /** Event for RTC periodic interrupt */
    struct RTCEvent : public Event
    {
        MC146818 * parent;
        Tick interval;

        RTCEvent(MC146818 * _parent, Tick i);

        /** Schedule the RTC periodic interrupt */
        void scheduleIntr();

        /** Event process to occur at interrupt*/
        virtual void process();

        /** Event description */
        virtual const char *description() const;
    };

    /** Event for RTC periodic interrupt */
    struct RTCTickEvent : public Event
    {
        MC146818 * parent;

        RTCTickEvent(MC146818 * _parent) : parent(_parent)
        {
            parent->schedule(this, curTick() + SimClock::Int::s);
        }

        /** Event process to occur at interrupt*/
        void process();

        /** Event description */
        const char *description() const;
    };

  private:
    std::string _name;
    const std::string &name() const { return _name; }

    /** RTC periodic interrupt event */
    RTCEvent event;

    /** RTC tick event */
    RTCTickEvent tickEvent;

    /** Data for real-time clock function */
    union {
        uint8_t clock_data[10];

        struct {
            uint8_t sec;
            uint8_t sec_alrm;
            uint8_t min;
            uint8_t min_alrm;
            uint8_t hour;
            uint8_t hour_alrm;
            uint8_t wday;
            uint8_t mday;
            uint8_t mon;
            uint8_t year;
        };
    };

    struct tm curTime;

    void setTime(const struct tm time);

    /** RTC status register A */
    uint8_t stat_regA;

    /** RTC status register B */
    uint8_t stat_regB;

  public:
    MC146818(EventManager *em, const std::string &name, const struct tm time,
            bool bcd, Tick frequency);
    virtual ~MC146818();

    /** RTC write data */
    void writeData(const uint8_t addr, const uint8_t data);

    /** RTC read data */
    uint8_t readData(const uint8_t addr);

    void tickClock();

    /**
      * Serialize this object to the given output stream.
      * @param base The base name of the counter object.
      * @param os The stream to serialize to.
      */
    void serialize(const std::string &base, std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param base The base name of the counter object.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(const std::string &base, Checkpoint *cp,
                     const std::string &section);
};

#endif // __DEV_MC146818_HH__
