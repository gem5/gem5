/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_TIME_HH__
#define __BASE_TIME_HH__

#include <sys/time.h>
#include <inttypes.h>

#include <cmath>
#include <cstring>
#include <ctime>
#include <iosfwd>
#include <iostream>
#include <string>

#include "base/types.hh"
#include "sim/serialize.hh"

class Time
{
  protected:
    timespec _time;

    /**
     * Internal time set function
     */
    void _set(bool monotonic);

  public:
    static const long NSEC_PER_SEC  = 1000 * 1000 * 1000;
    static const long NSEC_PER_MSEC = 1000 * 1000;
    static const long NSEC_PER_USEC = 1000;

  public:
    explicit Time() { clear(); }
    explicit Time(double sec) { operator=(sec); }
    Time(const Time &val) : _time(val._time) { }
    Time(uint64_t sec, uint64_t nsec) { set(sec, nsec); }
    Time(const timeval &tv) { operator=(tv); }
    Time(const timespec &ts) { operator=(ts); }

    /**
     * Accessors for getting and setting the current clock
     */
    time_t sec() const { return _time.tv_sec; }
    long msec() const { return _time.tv_nsec / NSEC_PER_MSEC; }
    long usec() const { return _time.tv_nsec / NSEC_PER_USEC; }
    long nsec() const { return _time.tv_nsec; }

    void sec(time_t sec) { _time.tv_sec = sec; }
    void msec(long msec) { _time.tv_nsec = msec * NSEC_PER_MSEC; }
    void usec(long usec) { _time.tv_nsec = usec * NSEC_PER_USEC; }
    void nsec(long nsec) { _time.tv_nsec = nsec; }

    /**
     * Clear the time
     */
    void clear() { memset(&_time, 0, sizeof(_time)); }

    /**
     * Use this to set time for the purposes of time measurement (use
     * a monotonic clock if it is available
     */
    void setTimer() { _set(true); }

    /**
     * Use this to set the time to the actual current time
     */
    void setWallclock() { _set(false); }

    /**
     * Set the current time
     */
    void set(time_t _sec, long _nsec) { sec(_sec); nsec(_nsec); }

    /**
     * Set the current time from a value measured in Ticks
     * @param ticks Number of ticks to convert into a time.
     */
    void setTick(Tick ticks);

    /**
     * Get the current time from a value measured in Ticks
     * @return Time value measured in Ticks.
     */
    Tick getTick() const;

    const Time &
    operator=(const Time &other)
    {
        sec(other.sec());
        nsec(other.nsec());
        return *this;
    }

    const Time &
    operator=(double new_time)
    {
        double seconds = floor(new_time);
        sec((time_t)seconds);
        nsec((long)((seconds - new_time) * 1e9));
        return *this;
    }

    const Time &
    operator=(const timeval &tv)
    {
        sec(tv.tv_sec);
        nsec(tv.tv_usec * 1000);
        return *this;
    }

    const Time &
    operator=(const timespec &ts)
    {
        sec(ts.tv_sec);
        nsec(ts.tv_nsec);
        return *this;
    }

    /**
     * Get the time in floating point seconds
     */
    operator double() const
    {
        return (double)sec() + ((double)nsec()) * 1e-9;
    }

    /**
     * operators for time conversion
     */
    operator timespec() const { return _time; }
    operator timeval() const
    {
        timeval tv;
        tv.tv_sec = sec();
        tv.tv_usec = usec();
        return tv;
    }

    const Time &
    operator+=(const Time &other)
    {

        _time.tv_sec += other.sec();
        _time.tv_nsec += other.nsec();
        if (_time.tv_nsec > NSEC_PER_SEC) {
            _time.tv_sec++;
            _time.tv_nsec -= NSEC_PER_SEC;
        }

        return *this;
    }

    const Time &
    operator-=(const Time &other)
    {
        _time.tv_sec -= other.sec();
        _time.tv_nsec -= other.nsec();
        if (_time.tv_nsec < 0) {
            _time.tv_sec--;
            _time.tv_nsec += NSEC_PER_SEC;
        }

        return *this;
    }

    std::string date(const std::string &format = "") const;
    std::string time() const;

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);
};

void sleep(const Time &time);

inline bool
operator==(const Time &l, const Time &r)
{
    return l.sec() == r.sec() && l.nsec() == r.nsec();
}

inline bool
operator!=(const Time &l, const Time &r)
{
    return l.sec() != r.sec() || l.nsec() != r.nsec();
}

inline bool
operator<(const Time &l, const Time &r)
{
    return (l.sec() < r.sec()) ||
        (l.sec() == r.sec() && l.nsec() < r.nsec());
}

inline bool
operator<=(const Time &l, const Time &r)
{
    return (l.sec() < r.sec()) ||
        (l.sec() == r.sec() && l.nsec() <= r.nsec());
}

inline bool
operator>(const Time &l, const Time &r)
{
    return (l.sec() > r.sec()) ||
        (l.sec() == r.sec() && l.nsec() > r.nsec());
}

inline bool
operator>=(const Time &l, const Time &r)
{
    return (l.sec() > r.sec()) ||
        (l.sec() == r.sec() && l.nsec() >= r.nsec());
}

inline Time
operator+(const Time &l, const Time &r)
{
    Time time(l);
    time += r;
    return time;
}

inline Time
operator-(const Time &l, const Time &r)
{
    Time time(l);
    time -= r;
    return time;
}

inline std::ostream &
operator<<(std::ostream &out, const Time &time)
{
    out << time.date();
    return out;
}

time_t mkutctime(struct tm *time);

#endif // __BASE_TIME_HH__
