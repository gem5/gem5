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

#ifndef __SYSTEMC_EXT_CORE_SC_OBJECT_HH__
#define __SYSTEMC_EXT_CORE_SC_OBJECT_HH__

#include <iostream>
#include <string>
#include <vector>

namespace sc_gem5
{

class Object;

} // namespace sc_gem5

namespace sc_core
{

class sc_event;
class sc_attr_base;
class sc_attr_cltn;
class sc_simcontext;

class sc_object
{
  public:
    const char *name() const;
    const char *basename() const;

    virtual const char *kind() const { return "sc_object"; }

    virtual void print(std::ostream & =std::cout) const;
    virtual void dump(std::ostream & =std::cout) const;

    virtual const std::vector<sc_object *> &get_child_objects() const;
    virtual const std::vector<sc_event *> &get_child_events() const;
    sc_object *get_parent_object() const;

    bool add_attribute(sc_attr_base &);
    sc_attr_base *get_attribute(const std::string &);
    sc_attr_base *remove_attribute(const std::string &);
    void remove_all_attributes();
    int num_attributes() const;
    sc_attr_cltn &attr_cltn();
    const sc_attr_cltn &attr_cltn() const;

    // Deprecated
    sc_simcontext *simcontext() const;

  protected:
    sc_object();
    sc_object(const char *);
    sc_object(const sc_object &);
    sc_object &operator = (const sc_object &);
    virtual ~sc_object();

  private:
    friend class sc_gem5::Object;
    sc_gem5::Object *_gem5_object;
};

const std::vector<sc_object *> &sc_get_top_level_objects();
sc_object *sc_find_object(const char *);

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_OBJECT_HH__
