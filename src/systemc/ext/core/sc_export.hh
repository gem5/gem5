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

#ifndef __SYSTEMC_EXT_CORE_SC_EXPORT_HH__
#define __SYSTEMC_EXT_CORE_SC_EXPORT_HH__

#include "../channel/messages.hh"
#include "../utils/sc_report_handler.hh"
#include "sc_module.hh" // for sc_gen_unique_name
#include "sc_object.hh"

namespace sc_core
{

class sc_interface;

class sc_export_base : public sc_object
{
  public:
    sc_export_base(const char *n);
    ~sc_export_base();

    virtual sc_interface *get_iterface() = 0;
    virtual const sc_interface *get_interface() const = 0;

  protected:
    friend class sc_gem5::Module;

    virtual void before_end_of_elaboration() = 0;
    virtual void end_of_elaboration() = 0;
    virtual void start_of_simulation() = 0;
    virtual void end_of_simulation() = 0;
};

template <class IF>
class sc_export : public sc_export_base
{
  public:
    sc_export() :
        sc_export_base(sc_gen_unique_name("export")), interface(nullptr)
    {}
    explicit sc_export(const char *n) :
        sc_export_base(n), interface(nullptr)
    {}
    virtual ~sc_export() {}

    virtual const char *kind() const { return "sc_export"; }

    void operator () (IF &i) { bind(i); }
    virtual void
    bind(IF &i)
    {
        if (interface) {
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_ALREADY_BOUND_, name());
            return;
        }
        interface = &i;
    }
    operator IF & ()
    {
        if (!interface)
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_, name());
        return *interface;
    }
    operator const IF & () const { return *interface; }

    IF *
    operator -> ()
    {
        if (!interface)
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_, name());
        return interface;
    }
    const IF *
    operator -> () const
    {
        if (!interface)
            SC_REPORT_ERROR(SC_ID_SC_EXPORT_HAS_NO_INTERFACE_, name());
        return interface;
    }

    sc_interface *get_iterface() override { return interface; }
    const sc_interface *get_interface() const override { return interface; }

  protected:
    void before_end_of_elaboration() {}
    void
    end_of_elaboration()
    {
        if (!interface) {
            std::string msg = "export not bound: export '";
            msg = msg + name() + "' (" + kind() + ")";
            SC_REPORT_ERROR("(E109) complete binding failed", msg.c_str());
        }
    }
    void start_of_simulation() {}
    void end_of_simulation() {}

  private:
    IF *interface;

    // Disabled
    sc_export(const sc_export<IF> &);
    sc_export<IF> &operator = (const sc_export<IF> &);
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_EXPORT_HH__
