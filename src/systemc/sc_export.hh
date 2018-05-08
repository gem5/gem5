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

#ifndef __SYSTEMC_SC_EXPORT_HH__
#define __SYSTEMC_SC_EXPORT_HH__

#include "sc_object.hh"

namespace sc_core
{

class sc_interface;

class sc_export_base : public sc_object
{
  public:
    void warn_unimpl(const char *func) const;
};

template <class IF>
class sc_export : public sc_export_base
{
  public:
    sc_export() { warn_unimpl(__PRETTY_FUNCTION__); }
    explicit sc_export(const char *) { warn_unimpl(__PRETTY_FUNCTION__); }
    virtual ~sc_export() { warn_unimpl(__PRETTY_FUNCTION__); };

    virtual const char *kind() const { return "sc_export"; }

    void operator () (IF &) { warn_unimpl(__PRETTY_FUNCTION__); };
    virtual void bind(IF &) { warn_unimpl(__PRETTY_FUNCTION__); };
    operator IF & () { warn_unimpl(__PRETTY_FUNCTION__); };
    operator const IF & () const { warn_unimpl(__PRETTY_FUNCTION__); };

    IF *
    operator -> ()
    {
        warn_unimpl(__PRETTY_FUNCTION__);
        return nullptr;
    }
    const IF *
    operator -> () const
    {
        warn_unimpl(__PRETTY_FUNCTION__);
        return nullptr;
    }

    virtual sc_interface *
    get_iterface()
    {
        warn_unimpl(__PRETTY_FUNCTION__);
        return nullptr;
    }
    virtual const sc_interface *
    get_interface() const
    {
        warn_unimpl(__PRETTY_FUNCTION__);
        return nullptr;
    }

  protected:
    virtual void before_end_of_elaboration() {}
    virtual void end_of_elaboration() {}
    virtual void start_of_simulation() {}
    virtual void end_of_simulation() {}

  private:
    // Disabled
    sc_export(const sc_export<IF> &);
    sc_export<IF> &operator = (const sc_export<IF> &);
};

} // namespace sc_core

#endif  //__SYSTEMC_SC_EXPORT_HH__
