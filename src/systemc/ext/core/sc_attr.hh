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

#ifndef __SYSTEMC_EXT_CORE_SC_ATTR_HH__
#define __SYSTEMC_EXT_CORE_SC_ATTR_HH__

#include <string>

namespace sc_core
{

class sc_attr_base
{
  public:
    sc_attr_base(const std::string &);
    sc_attr_base(const sc_attr_base &);
    virtual ~sc_attr_base();

    const std::string &name() const;

  protected:
    void warn_unimpl(const char *func);

  private:
    // Disabled
    sc_attr_base();
    sc_attr_base &operator = (const sc_attr_base &);
};

template <class T>
class sc_attribute : public sc_attr_base
{
  public:
    sc_attribute(const std::string &_name) : sc_attr_base(_name)
    {
        warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_attribute(const std::string &_name, const T &t) :
        sc_attr_base(_name), value(t)
    {
        warn_unimpl(__PRETTY_FUNCTION__);
    }
    sc_attribute(const sc_attribute<T> &other) :
        sc_attr_base(other.name()), value(other.value)
    {
        warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual ~sc_attribute() { warn_unimpl(__PRETTY_FUNCTION__); }
    T value;

  private:
    // Disabled
    sc_attribute() {}
    sc_attribute<T> &operator = (const sc_attribute<T> &) { return *this; }
};

class sc_attr_cltn
{
  public:
    typedef sc_attr_base *elem_type;
    typedef elem_type *iterator;
    typedef const elem_type *const_iterator;

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_ATTR_HH__
