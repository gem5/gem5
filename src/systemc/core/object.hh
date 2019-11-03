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

#ifndef __SYSTEMC_CORE_OBJECT_HH__
#define __SYSTEMC_CORE_OBJECT_HH__

#include <string>
#include <vector>

#include "systemc/ext/core/sc_attr.hh"
#include "systemc/ext/core/sc_object.hh"

namespace sc_gem5
{

class Object;

typedef std::vector<sc_core::sc_object *> Objects;
typedef std::vector<sc_core::sc_event *> Events;
typedef Objects::iterator ObjectsIt;
typedef Events::iterator EventsIt;

class Object
{
  public:
    Object(sc_core::sc_object *_sc_obj);
    Object(sc_core::sc_object *_sc_obj, const char *);
    Object(sc_core::sc_object *_sc_obj, const Object &);
    Object &operator = (const Object &);

    virtual ~Object();

    /*
     * sc_object methods.
     */
    const char *name() const;
    const char *basename() const;

    void print(std::ostream & =std::cout) const;
    void dump(std::ostream & =std::cout) const;

    const std::vector<sc_core::sc_object *> &get_child_objects() const;
    const std::vector<sc_core::sc_event *> &get_child_events() const;
    sc_core::sc_object *get_parent_object() const;

    bool add_attribute(sc_core::sc_attr_base &);
    sc_core::sc_attr_base *get_attribute(const std::string &);
    sc_core::sc_attr_base *remove_attribute(const std::string &);
    void remove_all_attributes();
    int num_attributes() const;
    sc_core::sc_attr_cltn &attr_cltn();
    const sc_core::sc_attr_cltn &attr_cltn() const;

    sc_core::sc_simcontext *simcontext() const;

    static Object *
    getFromScObject(sc_core::sc_object *sc_obj)
    {
        return sc_obj->_gem5_object;
    }

    sc_core::sc_object *sc_obj() { return _sc_obj; }

    EventsIt addChildEvent(sc_core::sc_event *e);
    void delChildEvent(sc_core::sc_event *e);

    std::string pickUniqueName(std::string name);

  private:
    sc_core::sc_object *_sc_obj;

    std::string _basename;
    std::string _name;

    Objects children;
    Events events;
    sc_core::sc_object *parent;

    sc_core::sc_attr_cltn cltn;
};

std::string pickUniqueName(::sc_core::sc_object *parent, std::string name);

extern Objects topLevelObjects;
extern Objects allObjects;

sc_core::sc_object *findObject(
        const char *name, const Objects &objects=topLevelObjects);

sc_core::sc_object *pickParentObj();
void pushParentObj(sc_core::sc_object *obj);
void popParentObj();

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_OBJECT_HH__
