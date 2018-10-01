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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_FIFO_IN_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_FIFO_IN_HH__

#include "../core/sc_event.hh"
#include "../core/sc_port.hh"
#include "sc_fifo_in_if.hh"

namespace sc_core
{

class sc_event;
class sc_event_finder;

template <class T>
class sc_fifo_in : public sc_port<sc_fifo_in_if<T>, 0>
{
  public:
    sc_fifo_in() : sc_port<sc_fifo_in_if<T>, 0>(),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    explicit sc_fifo_in(const char *name) :
        sc_port<sc_fifo_in_if<T>, 0>(name),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    virtual ~sc_fifo_in() {}

    // Deprecated binding constructors.
    explicit sc_fifo_in(const sc_fifo_in_if<T> &interface) :
        sc_port<sc_fifo_in_if<T>, 0>(interface),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    sc_fifo_in(const char *name, const sc_fifo_in_if<T> &interface) :
        sc_port<sc_fifo_in_if<T>, 0>(name, interface),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    explicit sc_fifo_in(sc_port_b<sc_fifo_in_if<T> > &parent) :
        sc_port<sc_fifo_in_if<T>, 0>(parent),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    sc_fifo_in(const char *name, sc_port_b<sc_fifo_in_if<T> > &parent) :
        sc_port<sc_fifo_in_if<T>, 0>(name, parent),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    explicit sc_fifo_in(sc_port<sc_fifo_in_if<T>, 0> &parent) :
        sc_port<sc_fifo_in_if<T>, 0>(parent),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}
    sc_fifo_in(const char *name, sc_port<sc_fifo_in_if<T>, 0> &parent) :
        sc_port<sc_fifo_in_if<T>, 0>(name, parent),
        _dataWrittenFinder(*this, &sc_fifo_in_if<T>::data_written_event)
    {}

    void read(T &t) { (*this)->read(t); }
    T read() { return (*this)->read(); }
    bool nb_read(T &t) { return (*this)->nb_read(t); }
    const sc_event &
    data_written_event() const
    {
        return (*this)->data_written_event();
    }
    sc_event_finder &data_written() const { return _dataWrittenFinder; }
    int num_available() const { return (*this)->num_available(); }
    virtual const char *kind() const { return "sc_fifo_in"; }

  private:
    // Disabled
    sc_fifo_in(const sc_fifo_in<T> &) : sc_port<sc_fifo_in_if<T>, 0>() {}
    sc_fifo_in<T> &operator = (const sc_fifo_in<T> &) { return *this; }

    mutable sc_event_finder_t<sc_fifo_in_if<T> > _dataWrittenFinder;
};

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_FIFO_IN_HH__
