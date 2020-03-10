/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
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

#ifndef __SIM_VPTR_HH__
#define __SIM_VPTR_HH__

#include "mem/port_proxy.hh"

class ThreadContext;

template <class T>
class VPtr
{
  public:
    typedef T Type;

  protected:
    ThreadContext *tc;
    Addr ptr;
    Addr buffer[(sizeof(T)-1)/sizeof(Addr) + 1];

  public:
    explicit VPtr(ThreadContext *_tc, Addr p = 0)
        : tc(_tc), ptr(p)
    {
        refresh();
    }

    template <class U>
    VPtr(const VPtr<U> &vp)
        : tc(vp.tc), ptr(vp.ptr)
    {
        refresh();
    }

    ~VPtr()
    {}

    void
    refresh()
    {
        if (!ptr)
            return;

        PortProxy &proxy = tc->getVirtProxy();
        proxy.readBlob(ptr, buffer, sizeof(T));
    }

    bool
    operator!() const
    {
        return ptr == 0;
    }

    VPtr<T>
    operator+(int offset)
    {
        return VPtr<T>(tc, ptr + offset);
    }

    const VPtr<T> &
    operator+=(int offset)
    {
        ptr += offset;
        refresh();

        return *this;
    }

    const VPtr<T> &
    operator=(Addr p)
    {
        ptr = p;
        refresh();

        return *this;
    }

    template <class U>
    const VPtr<T> &
    operator=(const VPtr<U> &vp)
    {
        tc = vp.tc;
        ptr = vp.ptr;
        refresh();

        return *this;
    }

    operator T *()
    {
        return (T *)buffer;
    }

    T *
    operator->()
    {
        return (T *)buffer;
    }

    T &
    operator*()
    {
        return *(T *)buffer;
    }
};

#endif // __SIM_VPTR_HH__
