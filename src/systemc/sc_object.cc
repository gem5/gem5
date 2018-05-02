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

#include "systemc/sc_object.hh"

#include "base/logging.hh"

namespace sc_core
{

const char *
sc_object::name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "sc_object";
}

const char *
sc_object::basename() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "sc_object";
}

const char *
sc_object::kind() const
{
    return "sc_object";
}

void
sc_object::print(std::ostream &out) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_object::dump(std::ostream &out) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const std::vector<sc_object *> &
sc_object::get_child_objects() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_object *> *)nullptr;
}

const std::vector<sc_event *> &
sc_object::get_child_events() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_event *> *)nullptr;
}

sc_object *
sc_object::get_parent_object() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return NULL;
}

bool
sc_object::add_attribute(sc_attr_base &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

sc_attr_base *
sc_object::get_attribute(const std::string &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return NULL;
}

sc_attr_base *
sc_object::remove_attribute(const std::string &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return NULL;
}

void
sc_object::remove_all_attributes()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

int
sc_object::num_attributes() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

sc_attr_cltn &
sc_object::attr_cltn()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_attr_cltn *)NULL;
}

const sc_attr_cltn &
sc_object::attr_cltn() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_attr_cltn *)NULL;
}

sc_object::sc_object()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_object::sc_object(const char *name)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_object::sc_object(const sc_object &arg)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_object &
sc_object::operator = (const sc_object &)
{
    return *this;
}

sc_object::~sc_object()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const std::vector<sc_object *> &
sc_get_top_level_objects()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::vector<sc_object *> *)nullptr;
}

sc_object *
sc_find_object(const char *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return NULL;
}

} // namespace sc_core
