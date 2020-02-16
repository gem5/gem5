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

#include <vector>

#include "systemc/core/object.hh"
#include "systemc/ext/core/sc_object.hh"

namespace sc_core
{

namespace
{

std::vector<sc_object *> top_level_objects;

} // anonymous namespace

const char *
sc_object::name() const
{
    return _gem5_object->name();
}

const char *
sc_object::basename() const
{
    return _gem5_object->basename();
}

void
sc_object::print(std::ostream &out) const
{
    _gem5_object->print(out);
}

void
sc_object::dump(std::ostream &out) const
{
    _gem5_object->dump(out);
}

const std::vector<sc_object *> &
sc_object::get_child_objects() const
{
    return _gem5_object->get_child_objects();
}

const std::vector<sc_event *> &
sc_object::get_child_events() const
{
    return _gem5_object->get_child_events();
}

sc_object *
sc_object::get_parent_object() const
{
    return _gem5_object->get_parent_object();
}

bool
sc_object::add_attribute(sc_attr_base &attr)
{
    return _gem5_object->add_attribute(attr);
}

sc_attr_base *
sc_object::get_attribute(const std::string &name)
{
    return _gem5_object->get_attribute(name);
}

sc_attr_base *
sc_object::remove_attribute(const std::string &name)
{
    return _gem5_object->remove_attribute(name);
}

void
sc_object::remove_all_attributes()
{
    return _gem5_object->remove_all_attributes();
}

int
sc_object::num_attributes() const
{
    return _gem5_object->num_attributes();
}

sc_attr_cltn &
sc_object::attr_cltn()
{
    return _gem5_object->attr_cltn();
}

const sc_attr_cltn &
sc_object::attr_cltn() const
{
    return _gem5_object->attr_cltn();
}

sc_simcontext *
sc_object::simcontext() const
{
    return _gem5_object->simcontext();
}

sc_object::sc_object()
{
    _gem5_object = new sc_gem5::Object(this);
}

sc_object::sc_object(const char *name)
{
    _gem5_object = new sc_gem5::Object(this, name);
}

sc_object::sc_object(const sc_object &other)
{
    _gem5_object = new sc_gem5::Object(this, *other._gem5_object);
}

sc_object &
sc_object::operator = (const sc_object &other)
{
    *_gem5_object = *other._gem5_object;
    return *this;
}

sc_object::~sc_object()
{
    delete _gem5_object;
}

const std::vector<sc_object *> &
sc_get_top_level_objects()
{
    return sc_gem5::topLevelObjects;
}

sc_object *
sc_find_object(const char *name)
{
    return sc_gem5::findObject(name);
}

} // namespace sc_core
