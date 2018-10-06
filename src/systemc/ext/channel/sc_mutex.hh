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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_MUTEX_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_MUTEX_HH__

#include "../core/sc_event.hh"
#include "../core/sc_object.hh"
#include "../core/sc_process_handle.hh"
#include "sc_mutex_if.hh"

namespace sc_core
{

class sc_mutex : public sc_mutex_if, public sc_object
{
  public:
    sc_mutex();
    explicit sc_mutex(const char *name);

    virtual int lock();
    virtual int trylock();
    virtual int unlock();

    virtual const char *kind() const { return "sc_mutex"; }

  private:
    // Disabled
    sc_mutex(const sc_mutex &) : sc_interface(), sc_mutex_if(), sc_object() {}
    sc_mutex &operator = (const sc_mutex &) { return *this; }

    sc_process_handle holder;
    sc_gem5::InternalScEvent unlockEvent;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_MUTEX_HH__
