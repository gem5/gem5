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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_EVENT_QUEUE_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_EVENT_QUEUE_HH__

#include "../core/sc_interface.hh"
#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "../core/sc_module_name.hh"
#include "../core/sc_time.hh"
#include "warn_unimpl.hh"

namespace sc_core
{

class sc_event;

class sc_event_queue_if : public virtual sc_interface
{
  public:
    virtual void notify(double, sc_time_unit) = 0;
    virtual void notify(const sc_time &) = 0;
    virtual void cancel_all() = 0;
};

class sc_event_queue : public sc_event_queue_if, public sc_module
{
  public:
    sc_event_queue(sc_module_name name=
                   sc_module_name(sc_gen_unique_name("event_queue")));
    ~sc_event_queue();

    virtual const char *kind() const;

    virtual void notify(double, sc_time_unit);
    virtual void notify(const sc_time &);
    virtual void cancel_all();

    virtual const sc_event &default_event() const;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_EVENT_QUEUE_HH__
