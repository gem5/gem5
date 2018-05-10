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

#ifndef __SYSTEMC_EXT_CHANNEL_SC_FIFO_HH__
#define __SYSTEMC_EXT_CHANNEL_SC_FIFO_HH__

#include "../core/sc_module.hh" // for sc_gen_unique_name
#include "../core/sc_prim.hh"
#include "sc_fifo_in_if.hh"
#include "sc_fifo_out_if.hh"
#include "warn_unimpl.hh"

namespace sc_core
{

class sc_port_base;
class sc_event;

template <class T>
class sc_fifo : public sc_fifo_in_if<T>,
                public sc_fifo_out_if<T>,
                public sc_prim_channel
{
  public:
    explicit sc_fifo(int size=16) :
            sc_fifo_in_if<T>(), sc_fifo_out_if<T>(),
            sc_prim_channel(sc_gen_unique_name("fifo"))
    {}
    explicit sc_fifo(const char *name, int size=16) :
            sc_fifo_in_if<T>(), sc_fifo_out_if<T>(),
            sc_prim_channel(name)
    {}
    virtual ~sc_fifo() {}

    virtual void
    register_port(sc_port_base &, const char *)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

    virtual void
    read(T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual T
    read()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(T *)nullptr;
    }
    virtual bool
    nb_read(T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    operator T()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(T *)nullptr;
    }

    virtual void
    write(const T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual bool
    nb_write(const T&)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return false;
    }
    sc_fifo<T> &
    operator = (const T &)
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *this;
    }

    virtual const sc_event &
    data_Written_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }
    virtual const sc_event &
    data_read_event() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return *(const sc_event *)nullptr;
    }

    virtual int
    num_available() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return 0;
    }
    virtual int
    num_free() const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
        return 0;
    }

    virtual void
    print(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual void
    dump(std::ostream & =std::cout) const
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }
    virtual const char *kind() const { return "sc_fifo"; }

  protected:
    virtual void
    update()
    {
        sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    }

  private:
    // Disabled
    sc_fifo(const sc_fifo<T> &) :
            sc_fifo_in_if<T>(), sc_fifo_in_if<T>(), sc_prim_channel()
    {}
    sc_fifo &operator = (const sc_fifo<T> &) { return *this; }
};

template <class T>
inline std::ostream &
operator << (std::ostream &os, const sc_fifo<T> &)
{
    sc_channel_warn_unimpl(__PRETTY_FUNCTION__);
    return os;
}

} // namespace sc_core

#endif  //__SYSTEMC_EXT_CHANNEL_SC_FIFO_HH__
