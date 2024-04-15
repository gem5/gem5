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

#ifndef __DEV_MC146818_HH__
#define __DEV_MC146818_HH__

#include "base/bitunion.hh"
#include "base/logging.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"

namespace gem5
{

/** Real-Time Clock (MC146818) */
class MC146818 : public EventManager
{
  protected:
    virtual void
    handleEvent()
    {
        warn("No RTC event handler defined.\n");
    }

  private:
    /** Event for RTC periodic interrupt */
    struct RTCEvent : public Event
    {
        MC146818 *parent;
        Tick interval;
        Tick offset;

        RTCEvent(MC146818 *_parent, Tick i);

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
        MC146818 *parent;
        Tick offset;

        RTCTickEvent(MC146818 *_parent)
            : parent(_parent), offset(sim_clock::as_int::s)
        {}

        /** Event process to occur at interrupt*/
        void process();

        /** Event description */
        const char *description() const;
    };

  private:
    std::string _name;

    const std::string &
    name() const
    {
        return _name;
    }

    /** RTC periodic interrupt event */
    RTCEvent event;

    /** RTC tick event */
    RTCTickEvent tickEvent;

    /** Data for real-time clock function */
    union
    {
        uint8_t clock_data[10];

        struct
        {
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

    BitUnion8(RtcRegA)
        Bitfield<7> uip;   /// 1 = date and time update in progress
        Bitfield<6, 4> dv; /// Divider configuration
        /** Rate selection
            0 = Disabled
            For 32768 Hz time bases:
              Freq = 32768Hz / 2**(n-1) for n >= 3
              Freq = 256Hz if n = 1
              Freq = 128Hz if n = 2
            Othwerise:
              Freq = 32768Hz / 2**(n-1)
        */
        Bitfield<3, 0> rs;
    EndBitUnion(RtcRegA)

    /// Is the DV field in regA set to disabled?
    static inline bool rega_dv_disabled(const RtcRegA &reg);

    BitUnion8(RtcRegB)
        Bitfield<7> set;       /// stop clock updates
        Bitfield<6> pie;       /// 1 = enable periodic clock interrupt
        Bitfield<5> aie;       /// 1 = enable alarm interrupt
        Bitfield<4> uie;       /// 1 = enable update-ended interrupt
        Bitfield<3> sqwe;      /// 1 = output sqare wave at SQW pin
        Bitfield<2> dm;        /// 0 = BCD, 1 = Binary coded time
        Bitfield<1> format24h; /// 0 = 12 hours, 1 = 24 hours
        Bitfield<0> dse;       /// USA Daylight Savings Time enable
    EndBitUnion(RtcRegB)

    /** RTC status register A */
    RtcRegA stat_regA;

    /** RTC status register B */
    RtcRegB stat_regB;

  public:
    MC146818(EventManager *em, const std::string &name, const struct tm time,
             bool bcd, Tick frequency);
    virtual ~MC146818();

    /** Start ticking */
    virtual void startup();

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
    void serialize(const std::string &base, CheckpointOut &cp) const;

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param base The base name of the counter object.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    void unserialize(const std::string &base, CheckpointIn &cp);
};

} // namespace gem5

#endif // __DEV_MC146818_HH__
