/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_EXT_CHANNEL_SC_CLOCK_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_CLOCK_HH__

#include "../core/sc_time.hh"
#include "sc_signal.hh"

namespace sc_gem5
{

class ClockTick;

} // namespace sc_gem5

namespace sc_core
{

template <class T>
class sc_in;

class sc_time;

class sc_clock : public sc_signal<bool>
{
  public:
    sc_clock();
    explicit sc_clock(const char *name);

    sc_clock(const char *name, const sc_time &period,
             double duty_cycle=0.5, const sc_time &start_time=SC_ZERO_TIME,
             bool posedge_first=true);

    sc_clock(const char *name, double period_v, sc_time_unit period_tu,
             double duty_cycle=0.5);

    sc_clock(const char *name, double period_v, sc_time_unit period_tu,
             double duty_cycle, double start_time_v,
             sc_time_unit start_time_tu, bool posedge_first=true);

    // Deprecated.
    sc_clock(const char *name, double period, double duty_cycle=0.5,
             double start_time=0.0, bool posedge_first=true);

    virtual ~sc_clock();

    virtual void write(const bool &);

    const sc_time &period() const;
    double duty_cycle() const;
    const sc_time &start_time() const;
    bool posedge_first() const;

    // Nonstandard
    static const sc_time &time_stamp();

    virtual const char *kind() const { return "sc_clock"; }

  protected:
    virtual void before_end_of_elaboration();

  private:
    friend class ::sc_gem5::ClockTick;

    // Disabled
    sc_clock(const sc_clock &) : sc_interface(), sc_signal<bool>() {}
    sc_clock &operator = (const sc_clock &) { return *this; }

    sc_time _period;
    double _dutyCycle;
    sc_time _startTime;
    bool _posedgeFirst;

    ::sc_gem5::ClockTick *_gem5UpEdge;
    ::sc_gem5::ClockTick *_gem5DownEdge;

    void
    tickUp()
    {
        m_new_val = true;
        request_update();
    }
    void
    tickDown()
    {
        m_new_val = false;
        request_update();
    }
};

typedef sc_in<bool> sc_in_clk;

// Deprecated
typedef sc_inout<bool> sc_inout_clk;
typedef sc_out<bool> sc_out_clk;

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_CLOCK_HH__
