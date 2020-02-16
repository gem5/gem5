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
 */

#ifndef __SYSTEMC_EXT_CORE_SC_PRIM_HH__
#define __SYSTEMC_EXT_CORE_SC_PRIM_HH__

#include "sc_object.hh"
#include "sc_time.hh"

namespace sc_gem5
{

class Channel;

uint64_t getChangeStamp();

} // namespace sc_gem5

namespace sc_core
{

class sc_event;
class sc_event_and_list;
class sc_event_or_list;

class sc_prim_channel : public sc_object
{
  public:
    virtual const char *kind() const { return "sc_prim_channel"; }

  protected:
    sc_prim_channel();
    explicit sc_prim_channel(const char *);
    virtual ~sc_prim_channel();

    void request_update();
    void async_request_update();
    virtual void update() {}

    void next_trigger();
    void next_trigger(const sc_event &);
    void next_trigger(const sc_event_or_list &);
    void next_trigger(const sc_event_and_list &);
    void next_trigger(const sc_time &);
    void next_trigger(double, sc_time_unit);
    void next_trigger(const sc_time &, const sc_event &);
    void next_trigger(double, sc_time_unit, const sc_event &);
    void next_trigger(const sc_time &, const sc_event_or_list &);
    void next_trigger(double, sc_time_unit, const sc_event_or_list &);
    void next_trigger(const sc_time &, const sc_event_and_list &);
    void next_trigger(double, sc_time_unit, const sc_event_and_list &);

    // Nonstandard.
    bool timed_out();

    void wait();
    void wait(int);
    void wait(const sc_event &);
    void wait(const sc_event_or_list &);
    void wait(const sc_event_and_list &);
    void wait(const sc_time &);
    void wait(double, sc_time_unit);
    void wait(const sc_time &, const sc_event &);
    void wait(double, sc_time_unit, const sc_event &);
    void wait(const sc_time &, const sc_event_or_list &);
    void wait(double, sc_time_unit, const sc_event_or_list &);
    void wait(const sc_time &, const sc_event_and_list &);
    void wait(double, sc_time_unit, const sc_event_and_list &);

    friend class sc_gem5::Kernel;

    virtual void before_end_of_elaboration() {}
    virtual void end_of_elaboration() {}
    virtual void start_of_simulation() {}
    virtual void end_of_simulation() {}

  private:
    // Disabled
    sc_prim_channel(const sc_prim_channel &);
    sc_prim_channel &operator = (const sc_prim_channel &);

    friend class sc_gem5::Channel;
    sc_gem5::Channel *_gem5_channel;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_PRIM_HH__
