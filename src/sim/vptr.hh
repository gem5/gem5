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

#ifndef __ARCH_ALPHA_VPTR_HH__
#define __ARCH_ALPHA_VPTR_HH__

#include "arch/vtophys.hh"
#include "arch/isa_traits.hh"

class ExecContext;

template <class T>
class VPtr
{
  public:
    typedef T Type;

  private:
    ExecContext *xc;
    Addr ptr;

  public:
    ExecContext *GetXC() const { return xc; }
    Addr GetPointer() const { return ptr; }

  public:
    explicit VPtr(ExecContext *_xc, Addr p = 0) : xc(_xc), ptr(p) { }
    template <class U>
    VPtr(const VPtr<U> &vp) : xc(vp.GetXC()), ptr(vp.GetPointer()) {}
    ~VPtr() {}

    bool operator!() const
    {
        return ptr == 0;
    }

    VPtr<T> operator+(int offset)
    {
        VPtr<T> ptr(*this);
        ptr += offset;

        return ptr;
    }

    const VPtr<T> &operator+=(int offset)
    {
        ptr += offset;
        assert((ptr & (TheISA::PageBytes - 1)) + sizeof(T)
               < TheISA::PageBytes);

        return *this;
    }

    const VPtr<T> &operator=(Addr p)
    {
        assert((p & (TheISA::PageBytes - 1)) + sizeof(T)
               < TheISA::PageBytes);
        ptr = p;

        return *this;
    }

    template <class U>
    const VPtr<T> &operator=(const VPtr<U> &vp)
    {
        xc = vp.GetXC();
        ptr = vp.GetPointer();

        return *this;
    }

    operator T *()
    {
        panic("Needs to be rewritten\n");
/*	void *addr = vtomem(xc, ptr, sizeof(T));
        return (T *)addr;
        */
    }

    T *operator->()
    {
        panic("Needs to be rewritten\n");
/*	void *addr = vtomem(xc, ptr, sizeof(T));
        return (T *)addr;
        */
    }

    T &operator*()
    {
        panic("Needs to be rewritten\n");
/*	void *addr = vtomem(xc, ptr, sizeof(T));
        return *(T *)addr;
        */
    }
};

#endif // __ARCH_ALPHA_VPTR_HH__
