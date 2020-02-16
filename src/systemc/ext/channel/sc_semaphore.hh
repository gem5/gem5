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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_SEMAPHORE_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_SEMAPHORE_HH__

#include "../core/sc_event.hh"
#include "../core/sc_object.hh"
#include "sc_semaphore_if.hh"

namespace sc_core
{

class sc_semaphore : public sc_semaphore_if, public sc_object
{
  public:
    explicit sc_semaphore(int);
    sc_semaphore(const char *name, int);

    virtual int wait();
    virtual int trywait();
    virtual int post();
    virtual int get_value() const;

    virtual const char *kind() const { return "sc_semaphore"; }

  private:
    // Disabled
    sc_semaphore(const sc_semaphore &) :
            sc_interface(), sc_semaphore_if(), sc_object()
    {}

    sc_semaphore &operator = (const sc_semaphore &) { return *this; }

    int _value;
    sc_gem5::InternalScEvent posted;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_SEMAPHORE_HH__
