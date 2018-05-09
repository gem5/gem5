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

#ifndef __SYSTEMC_EXT_CORE_SC_EVENT_HH__
#define __SYSTEMC_EXT_CORE_SC_EVENT_HH__

#include <vector>

#include "sc_time.hh"

namespace sc_core
{

class sc_event;
class sc_event_and_expr;
class sc_event_or_expr;
class sc_object;
class sc_port_base;

class sc_event_finder
{
  protected:
    void warn_unimpl(const char *func) const;
};

template <class IF>
class sc_event_finder_t : public sc_event_finder
{
  public:
    sc_event_finder_t(const sc_port_base &,
                      const sc_event & (IF::*event_method)() const)
    {
        warn_unimpl(__PRETTY_FUNCTION__);
    }
};

class sc_event_and_list
{
  public:
    sc_event_and_list();
    sc_event_and_list(const sc_event_and_list &);
    sc_event_and_list(const sc_event &);
    sc_event_and_list &operator = (const sc_event_and_list &);

    int size() const;
    void swap(sc_event_and_list &);

    sc_event_and_list &operator &= (const sc_event &);
    sc_event_and_list &operator &= (const sc_event_and_list &);

    sc_event_and_expr operator & (const sc_event &) const;
    sc_event_and_expr operator & (const sc_event_and_list &);
};

class sc_event_or_list
{
  public:
    sc_event_or_list();
    sc_event_or_list(const sc_event_or_list &);
    sc_event_or_list(const sc_event &);
    sc_event_or_list& operator = (const sc_event_or_list &);
    ~sc_event_or_list();

    int size() const;
    void swap(sc_event_or_list &);

    sc_event_or_list &operator |= (const sc_event &);
    sc_event_or_list &operator |= (const sc_event_or_list &);

    sc_event_or_expr operator | (const sc_event &) const;
    sc_event_or_expr operator | (const sc_event_or_list &) const;
};

class sc_event_and_expr
{
  public:
    operator const sc_event_and_list &() const;
};

sc_event_and_expr operator & (sc_event_and_expr, sc_event const &);
sc_event_and_expr operator & (sc_event_and_expr, sc_event_and_list const &);

class sc_event_or_expr
{
  public:
    operator const sc_event_or_list &() const;
};

sc_event_or_expr operator | (sc_event_or_expr, sc_event const &);
sc_event_or_expr operator | (sc_event_or_expr, sc_event_or_list const &);

class sc_event
{
  public:
    sc_event();
    explicit sc_event(const char *);
    ~sc_event();

    const char *name() const;
    const char *basename() const;
    bool in_hierarchy() const;
    sc_object *get_parent_object() const;

    void notify();
    void notify(const sc_time &);
    void notify(double, sc_time_unit);
    void cancel();

    sc_event_and_expr operator & (const sc_event &) const;
    sc_event_and_expr operator & (const sc_event_and_list &) const;
    sc_event_or_expr operator | (const sc_event &) const;
    sc_event_or_expr operator | (const sc_event_or_list &) const;

  private:
    // Disabled
    sc_event(const sc_event &) {}
    sc_event &operator = (const sc_event &) { return *this; }
};

const std::vector<sc_event *> &sc_get_top_level_events();
sc_event *sc_find_event(const char *);

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_INTERFACE_HH__
