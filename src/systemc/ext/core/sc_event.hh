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

#include <cassert>
#include <set>
#include <sstream>
#include <vector>

#include "../channel/messages.hh"
#include "../utils/sc_report_handler.hh"
#include "sc_port.hh"
#include "sc_time.hh"

namespace sc_gem5
{

class Event;
class DynamicSensitivityEventAndList;
class DynamicSensitivityEventOrList;
class InternalScEvent;

}

namespace sc_core
{

class sc_event;
class sc_event_and_expr;
class sc_event_or_expr;
class sc_interface;
class sc_object;
class sc_port_base;

class sc_event_and_list
{
  public:
    sc_event_and_list();
    sc_event_and_list(const sc_event_and_list &);
    sc_event_and_list(const sc_event &);
    sc_event_and_list &operator = (const sc_event_and_list &);
    ~sc_event_and_list();

    int size() const;
    void swap(sc_event_and_list &);

    sc_event_and_list &operator &= (const sc_event &);
    sc_event_and_list &operator &= (const sc_event_and_list &);

    sc_event_and_expr operator & (const sc_event &) const;
    sc_event_and_expr operator & (const sc_event_and_list &) const;

  private:
    friend class sc_event_and_expr;
    friend class sc_gem5::DynamicSensitivityEventAndList;

    explicit sc_event_and_list(bool auto_delete);

    void insert(sc_event const &e);
    void insert(sc_event_and_list const &eal);

    std::set<const sc_event *> events;
    bool autoDelete;
    mutable unsigned busy;
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

  private:
    friend class sc_event_or_expr;
    friend class sc_gem5::DynamicSensitivityEventOrList;

    explicit sc_event_or_list(bool auto_delete);

    void insert(sc_event const &e);
    void insert(sc_event_or_list const &eol);

    std::set<const sc_event *> events;
    bool autoDelete;
    mutable unsigned busy;
};

class sc_event_and_expr
{
  public:
    sc_event_and_expr(sc_event_and_expr const &e);
    operator const sc_event_and_list &() const;

    void insert(sc_event const &e) const;
    void insert(sc_event_and_list const &eal) const;

    ~sc_event_and_expr();

  private:
    friend class sc_event_and_list;
    friend class sc_event;

    sc_event_and_expr();
    mutable sc_event_and_list *list;
};

sc_event_and_expr operator & (sc_event_and_expr, sc_event const &);
sc_event_and_expr operator & (sc_event_and_expr, sc_event_and_list const &);

class sc_event_or_expr
{
  public:
    sc_event_or_expr(sc_event_or_expr const &e);
    operator const sc_event_or_list &() const;

    void insert(sc_event const &e) const;
    void insert(sc_event_or_list const &eol) const;

    ~sc_event_or_expr();

  private:
    friend class sc_event_or_list;
    friend class sc_event;

    sc_event_or_expr();
    mutable sc_event_or_list *list;
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

    // Nonstandard
    // Returns whether this event is currently triggered.
    bool triggered() const;

    // Deprecated
    void notify_delayed();
    void notify_delayed(const sc_time &);

    sc_event_and_expr operator & (const sc_event &) const;
    sc_event_and_expr operator & (const sc_event_and_list &) const;
    sc_event_or_expr operator | (const sc_event &) const;
    sc_event_or_expr operator | (const sc_event_or_list &) const;

  protected:
    explicit sc_event(bool);
    explicit sc_event(bool, const char *);

  private:
    // Disabled
    sc_event(const sc_event &) {}
    sc_event &operator = (const sc_event &) { return *this; }

    friend class ::sc_gem5::Event;
    ::sc_gem5::Event *_gem5_event;
};

class sc_event_finder
{
  protected:
    virtual ~sc_event_finder() {}

  public:
    // Should be "implementation defined" but used in the tests.
    virtual const sc_event &find_event(sc_interface *if_p=NULL) const = 0;
    virtual const sc_port_base *port() const = 0;
};

template <class IF>
class sc_event_finder_t : public sc_event_finder
{
  public:
    sc_event_finder_t(const sc_port_base &p,
                      const sc_event & (IF::*_method)() const) :
        _method(_method)
    {
        _port = dynamic_cast<const sc_port_b<IF> *>(&p);
        assert(_port);
    }

    virtual ~sc_event_finder_t() {}

    const sc_port_base *port() const override { return _port; }

    const sc_event &find_event(sc_interface *if_p=NULL) const override;

  private:
    const sc_port_b<IF> *_port;
    const sc_event &(IF::*_method)() const;
};

const std::vector<sc_event *> &sc_get_top_level_events();
sc_event *sc_find_event(const char *);

} // namespace sc_core

namespace sc_gem5
{

class InternalScEvent : public ::sc_core::sc_event
{
  public:
    InternalScEvent();
    InternalScEvent(const char *);
};

} // namespace sc_gem5

namespace sc_core
{

template <class IF>
const sc_event &
sc_event_finder_t<IF>::find_event(sc_interface *if_p) const
{
    static const sc_gem5::InternalScEvent none;
    const IF *iface = if_p ? dynamic_cast<const IF *>(if_p) :
        dynamic_cast<const IF *>(_port->get_interface());
    if (!iface) {
        std::ostringstream ss;
        ss << "port is not bound: port '" << _port->name() << "' (" <<
            _port->kind() << ")";
        SC_REPORT_ERROR(SC_ID_FIND_EVENT_, ss.str().c_str());
        return none;
    }
    return (const_cast<IF *>(iface)->*_method)();
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_INTERFACE_HH__
