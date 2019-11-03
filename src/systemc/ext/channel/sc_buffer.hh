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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_BUFFER_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_BUFFER_HH__

#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "sc_signal.hh"

namespace sc_core
{

// Having a default value for the WRITER_POLICY parameter is non-standard, but
// matches the Accellera implementation to enable the regression tests.
template <class T, sc_writer_policy WRITER_POLICY=SC_ONE_WRITER>
class sc_buffer : public sc_signal<T, WRITER_POLICY>
{
  public:
    sc_buffer() : sc_signal<T, WRITER_POLICY>(sc_gen_unique_name("buffer")) {}
    explicit sc_buffer(const char *name) :
            sc_signal<T, WRITER_POLICY>(sc_gen_unique_name(name))
    {}

    virtual void
    write(const T &t)
    {
#       if !defined(SC_NO_WRITE_CHECK)
        {
            this->_checker.checkWriter();
        }
#       endif
        this->m_new_val = t;
        this->request_update();
    }

    sc_buffer<T, WRITER_POLICY> &
    operator = (const T &arg)
    {
        write(arg);
        return *this;
    }
    sc_buffer<T, WRITER_POLICY> &
    operator = (const sc_signal<T, WRITER_POLICY> &arg)
    {
        write(arg.read());
        return *this;
    }
    sc_buffer<T, WRITER_POLICY> &
    operator = (const sc_buffer<T, WRITER_POLICY> &arg)
    {
        write(arg.read());
        return *this;
    }

    virtual const char *kind() const { return "sc_buffer"; }

  protected:
    virtual void
    update()
    {
        this->m_cur_val = this->m_new_val;
        this->_signalChange();
    }

  private:
    // Disabled
    sc_buffer(const sc_buffer<T, WRITER_POLICY> &) {}
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_BUFFER_HH__
