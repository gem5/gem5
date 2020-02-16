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

#include <utility>

#include "systemc/ext/core/sc_attr.hh"

namespace sc_core
{

sc_attr_base::sc_attr_base(const std::string &_name) : _name(_name) {}
sc_attr_base::sc_attr_base(const sc_attr_base &other) : _name(other._name) {}
sc_attr_base::~sc_attr_base() {}

const std::string &sc_attr_base::name() const { return _name; }

sc_attr_cltn::iterator
sc_attr_cltn::begin()
{
    return cltn.begin();
}

sc_attr_cltn::const_iterator
sc_attr_cltn::begin() const
{
    return cltn.begin();
}

sc_attr_cltn::iterator
sc_attr_cltn::end()
{
    return cltn.end();
}

sc_attr_cltn::const_iterator
sc_attr_cltn::end() const
{
    return cltn.end();
}

sc_attr_cltn::sc_attr_cltn() {}
sc_attr_cltn::sc_attr_cltn(const sc_attr_cltn &other) : cltn(other.cltn) {}
sc_attr_cltn::~sc_attr_cltn() {}

bool
sc_attr_cltn::push_back(sc_attr_base *attr)
{
    if (!attr || (*this)[attr->name()])
        return false;

    cltn.push_back(attr);
    return true;
}

sc_attr_base *
sc_attr_cltn::operator [] (const std::string &name)
{
    for (auto &attr: cltn)
        if (attr->name() == name)
            return attr;
    return nullptr;
}

const sc_attr_base *
sc_attr_cltn::operator [] (const std::string &name) const
{
    for (auto &attr: cltn)
        if (attr->name() == name)
            return attr;
    return nullptr;
}

sc_attr_base *
sc_attr_cltn::remove(const std::string &name)
{
    for (auto &attr: cltn) {
        if (attr->name() == name) {
            sc_attr_base *ret = attr;
            std::swap(attr, cltn.back());
            cltn.pop_back();
            return ret;
        }
    }
    return nullptr;
}

void sc_attr_cltn::remove_all() { cltn.clear(); }

int sc_attr_cltn::size() const { return cltn.size(); }

} // namespace sc_core
