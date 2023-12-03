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

#include "systemc/core/object.hh"

#include <algorithm>
#include <stack>

#include "systemc/core/event.hh"
#include "systemc/core/module.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_simcontext.hh"

namespace sc_gem5
{

namespace
{

ObjectsIt
findObjectIn(Objects &objects, const std::string &name)
{
    ObjectsIt it;
    for (it = objects.begin(); it != objects.end(); it++)
        if (!strcmp((*it)->name(), name.c_str()))
            break;

    return it;
}

void
addObject(Objects *objects, sc_core::sc_object *object)
{
    objects->emplace(objects->end(), object);
}

void
popObject(Objects *objects, const std::string &name)
{
    ObjectsIt it = findObjectIn(*objects, name);
    assert(it != objects->end());
    std::swap(objects->back(), *it);
    objects->pop_back();
}

bool
nameIsUnique(Objects *objects, Events *events, const std::string &name)
{
    for (auto obj : *objects)
        if (!strcmp(obj->basename(), name.c_str()))
            return false;
    for (auto event : *events)
        if (!strcmp(event->basename(), name.c_str()))
            return false;
    return true;
}

} // anonymous namespace

Object::Object(sc_core::sc_object *_sc_obj) : Object(_sc_obj, nullptr) {}

Object::Object(sc_core::sc_object *_sc_obj, const char *obj_name)
    : _sc_obj(_sc_obj), _basename(obj_name ? obj_name : ""), parent(nullptr)
{
    if (_basename == "")
        _basename = ::sc_core::sc_gen_unique_name("object");

    parent = pickParentObj();

    Module *n = newModule();
    if (n) {
        // We are a module in the process of being constructed.
        n->finish(this);
    }

    std::string original_name = _basename;
    _basename = sc_gem5::pickUniqueName(parent, original_name);

    if (parent)
        addObject(&parent->_gem5_object->children, _sc_obj);
    else
        addObject(&topLevelObjects, _sc_obj);

    addObject(&allObjects, _sc_obj);

    sc_core::sc_object *sc_p = parent;
    std::string path = "";
    while (sc_p) {
        path = std::string(sc_p->basename()) + std::string(".") + path;
        sc_p = sc_p->get_parent_object();
    }

    if (_basename != original_name) {
        std::string message = path + original_name +
                              ". Latter declaration will be renamed to " +
                              path + _basename;
        SC_REPORT_WARNING(sc_core::SC_ID_INSTANCE_EXISTS_, message.c_str());
    }
    _name = path + _basename;
}

Object::Object(sc_core::sc_object *_sc_obj, const Object &arg)
    : Object(_sc_obj, arg._basename.c_str())
{}

Object &
Object::operator=(const Object &)
{
    return *this;
}

Object::~Object()
{
    // Promote all children to be top level objects.
    for (auto child : children) {
        addObject(&topLevelObjects, child);
        child->_gem5_object->parent = nullptr;
    }
    children.clear();

    for (auto event : events)
        Event::getFromScEvent(event)->clearParent();

    if (parent)
        popObject(&parent->_gem5_object->children, _name);
    else
        popObject(&topLevelObjects, _name);
    popObject(&allObjects, _name);
}

const char *
Object::name() const
{
    return _name.c_str();
}

const char *
Object::basename() const
{
    return _basename.c_str();
}

void
Object::print(std::ostream &out) const
{
    out << name();
}

void
Object::dump(std::ostream &out) const
{
    out << "name = " << name() << "\n";
    out << "kind = " << _sc_obj->kind() << "\n";
}

const std::vector<sc_core::sc_object *> &
Object::get_child_objects() const
{
    return children;
}

const std::vector<sc_core::sc_event *> &
Object::get_child_events() const
{
    return events;
}

sc_core::sc_object *
Object::get_parent_object() const
{
    return parent;
}

bool
Object::add_attribute(sc_core::sc_attr_base &attr)
{
    return cltn.push_back(&attr);
}

sc_core::sc_attr_base *
Object::get_attribute(const std::string &attr)
{
    return cltn[attr];
}

sc_core::sc_attr_base *
Object::remove_attribute(const std::string &attr)
{
    return cltn.remove(attr);
}

void
Object::remove_all_attributes()
{
    cltn.remove_all();
}

int
Object::num_attributes() const
{
    return cltn.size();
}

sc_core::sc_attr_cltn &
Object::attr_cltn()
{
    return cltn;
}

const sc_core::sc_attr_cltn &
Object::attr_cltn() const
{
    return cltn;
}

sc_core::sc_simcontext *
Object::simcontext() const
{
    return sc_core::sc_get_curr_simcontext();
}

EventsIt
Object::addChildEvent(sc_core::sc_event *e)
{
    return events.emplace(events.end(), e);
}

void
Object::delChildEvent(sc_core::sc_event *e)
{
    EventsIt it = std::find(events.begin(), events.end(), e);
    assert(it != events.end());
    std::swap(*it, events.back());
    events.pop_back();
}

std::string
Object::pickUniqueName(std::string base)
{
    std::string seed = base;
    while (!nameIsUnique(&children, &events, base))
        base = ::sc_core::sc_gen_unique_name(seed.c_str());

    return base;
}

std::string
pickUniqueName(::sc_core::sc_object *parent, std::string base)
{
    if (parent)
        return Object::getFromScObject(parent)->pickUniqueName(base);

    std::string seed = base;
    while (!nameIsUnique(&topLevelObjects, &topLevelEvents, base))
        base = ::sc_core::sc_gen_unique_name(seed.c_str());

    return base;
}

Objects topLevelObjects;
Objects allObjects;

const std::vector<sc_core::sc_object *> &
getTopLevelScObjects()
{
    return topLevelObjects;
}

sc_core::sc_object *
findObject(const char *name, const Objects &objects)
{
    ObjectsIt it = findObjectIn(allObjects, name);
    return it == allObjects.end() ? nullptr : *it;
}

namespace
{

std::stack<sc_core::sc_object *> objParentStack;

} // anonymous namespace

sc_core::sc_object *
pickParentObj()
{
    if (!objParentStack.empty())
        return objParentStack.top();

    Process *p = scheduler.current();
    if (p)
        return p;

    return nullptr;
}

void
pushParentObj(sc_core::sc_object *obj)
{
    objParentStack.push(obj);
}

void
popParentObj()
{
    objParentStack.pop();
}

} // namespace sc_gem5
