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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_IN_RV_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_IN_RV_HH__

#include <sstream>

#include "messages.hh"
#include "sc_in.hh"
#include "sc_signal_rv.hh"

namespace sc_dt
{

template <int W>
class sc_lv;

} // namespace sc_dt

namespace sc_core
{

template <int W>
class sc_in_rv : public sc_in<sc_dt::sc_lv<W>>
{
  public:
    sc_in_rv() : sc_in<sc_dt::sc_lv<W>>() {}
    explicit sc_in_rv(const char *name) : sc_in<sc_dt::sc_lv<W>>(name) {}
    virtual ~sc_in_rv() {};

    virtual void
    end_of_elaboration()
    {
        sc_in<sc_dt::sc_lv<W> >::end_of_elaboration();
        if (!dynamic_cast<sc_signal_rv<W> *>(this->get_interface())) {
            std::ostringstream ss;
            ss << "port '" << this->name() << "' (" << this->kind() << ")";
            SC_REPORT_ERROR(SC_ID_RESOLVED_PORT_NOT_BOUND_, ss.str().c_str());
        }
    }

    virtual const char *kind() const { return "sc_in_rv"; }

  private:
    // Disabled
    sc_in_rv(const sc_in_rv<W> &) : sc_in<sc_dt::sc_lv<W>>() {}
    sc_in_rv<W> &operator = (const sc_in_rv<W> &) { *this; }
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_IN_RV_HH__
