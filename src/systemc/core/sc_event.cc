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

#include "base/logging.hh"
#include "systemc/ext/core/sc_event.hh"

namespace sc_core
{

void
sc_event_finder::warn_unimpl(const char *func) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_list::sc_event_and_list()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_list::sc_event_and_list(const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_list::sc_event_and_list(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_list &
sc_event_and_list::operator = (const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

int
sc_event_and_list::size() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

void
sc_event_and_list::swap(sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_list &
sc_event_and_list::operator &= (const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_event_and_list &
sc_event_and_list::operator &= (const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_event_and_expr
sc_event_and_list::operator & (const sc_event &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_and_expr();
}

sc_event_and_expr
sc_event_and_list::operator & (const sc_event_and_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_and_expr();
}

sc_event_or_list::sc_event_or_list()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_or_list::sc_event_or_list(const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_or_list::sc_event_or_list(const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_or_list&
sc_event_or_list::operator = (const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_event_or_list::~sc_event_or_list()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

int
sc_event_or_list::size() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

void
sc_event_or_list::swap(sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_or_list &
sc_event_or_list::operator |= (const sc_event &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_event_or_list &
sc_event_or_list::operator |= (const sc_event_or_list &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_event_or_expr
sc_event_or_list::operator | (const sc_event &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_or_expr();
}

sc_event_or_expr
sc_event_or_list::operator | (const sc_event_or_list &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_or_expr();
}

sc_event_and_expr::operator const sc_event_and_list &() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_event_and_list *)nullptr;
}

sc_event_and_expr
operator & (sc_event_and_expr expr, sc_event const &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return expr;
}

sc_event_and_expr
operator & (sc_event_and_expr expr, sc_event_and_list const &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return expr;
}

sc_event_or_expr::operator const sc_event_or_list &() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_event_or_list *)nullptr;
}

sc_event_or_expr
operator | (sc_event_or_expr expr, sc_event const &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return expr;
}

sc_event_or_expr
operator | (sc_event_or_expr expr, sc_event_or_list const &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return expr;
}

sc_event::sc_event()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event::sc_event(const char *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event::~sc_event()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const char *
sc_event::name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

const char *
sc_event::basename() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_event::in_hierarchy() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

sc_object *
sc_event::get_parent_object() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (sc_object *)nullptr;
}

void
sc_event::notify()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_event::notify(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_event::notify(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_event::cancel()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_event_and_expr
sc_event::operator & (const sc_event &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_and_expr();
}

sc_event_and_expr
sc_event::operator & (const sc_event_and_list &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_and_expr();
}

sc_event_or_expr
sc_event::operator | (const sc_event &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_or_expr();
}

sc_event_or_expr
sc_event::operator | (const sc_event_or_list &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_event_or_expr();
}

const std::vector<sc_event *> &
sc_get_top_level_events()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_event *> *)nullptr;
}

sc_event *
sc_find_event(const char *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (sc_event *)nullptr;
}

} // namespace sc_core
