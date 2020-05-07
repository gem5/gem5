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

#ifndef __SIM_PROXY_PTR_HH__
#define __SIM_PROXY_PTR_HH__

#include <cstdint>
#include <memory>
#include <type_traits>

#include "base/logging.hh"
#include "base/types.hh"
#include "sim/guest_abi.hh"

template <typename Proxy>
class ProxyPtrBuffer
{
  private:
    std::shared_ptr<Proxy> proxy;

    Addr ptr;
    size_t size;
    std::unique_ptr<uint8_t[]> data;

    bool dirty = false;

    void markClean() { dirty = false; }

  public:

    std::shared_ptr<Proxy> getProxy() const { return proxy; }

    void markDirty() { dirty = true; }
    bool isDirty() { return dirty; }

    template <typename T>
    T &
    as()
    {
        assert(sizeof(T) <= size);
        markDirty();
        return *reinterpret_cast<T *>(data.get());
    }

    template <typename T>
    const T &
    asConst() const
    {
        assert(sizeof(T) <= size);
        return *reinterpret_cast<T *>(data.get());
    }

    void
    flush(bool force=false)
    {
        if (force || isDirty()) {
            proxy->writeBlob(ptr, data.get(), size);
            markClean();
        }
    }

    void
    load()
    {
        panic_if(isDirty(), "Overwriting dirty ProxyPtr.");
        proxy->readBlob(ptr, data.get(), size);
    }

    Addr addr() const { return ptr; }

    ProxyPtrBuffer(std::shared_ptr<Proxy> _proxy, Addr _ptr, size_t _size) :
        proxy(_proxy), ptr(_ptr), size(_size), data(new uint8_t[_size])
    {
        load();
    }

    ~ProxyPtrBuffer() { flush(); }
};

template <typename T, typename Proxy>
class ConstProxyPtr
{
  protected:
    std::shared_ptr<Proxy> proxy;
    std::shared_ptr<ProxyPtrBuffer<Proxy>> buffer;

    template <typename O, typename P>
    friend class ProxyPtr;

    void nullCheck() const { panic_if(!buffer, "Accessing null ProxyPtr."); }

    void
    setAddr(Addr ptr)
    {
        if (ptr)
            buffer.reset(new ProxyPtrBuffer<Proxy>(proxy, ptr, sizeof(T)));
        else
            buffer.reset((ProxyPtrBuffer<Proxy> *)nullptr);
    }

    ConstProxyPtr(Addr _ptr, std::shared_ptr<Proxy> _proxy) : proxy(_proxy)
    {
        setAddr(_ptr);
    }

    using CPP = ConstProxyPtr<T, Proxy>;

  public:
    using Type = T;

    template <typename ...Args,
              typename std::enable_if<std::is_constructible<
                  Proxy, Args&&...>::value, int>::type = 0>
    explicit ConstProxyPtr(Addr _ptr, Args&&... args) :
        proxy(std::make_shared<Proxy>(args...))
    {
        setAddr(_ptr);
    }
    template <typename ...Args,
              typename std::enable_if<std::is_constructible<
                  Proxy, Args&&...>::value, int>::type = 0>
    explicit ConstProxyPtr(Args&&... args) :
        proxy(std::make_shared<Proxy>(args...))
    {
        setAddr(0);
    }

    template <typename O, typename Enabled=
        typename std::enable_if<std::is_assignable<T *, O *>::value>::type>
    ConstProxyPtr(const ConstProxyPtr<O, Proxy> &other) :
        proxy(other.proxy), buffer(other.buffer)
    {}

    ConstProxyPtr(const CPP &other) :
        proxy(other.proxy), buffer(other.buffer)
    {}

    void
    load()
    {
        nullCheck();
        buffer->load();
    }

    Addr addr() const { return buffer ? buffer->addr() : 0; }
    operator bool() const { return (bool)buffer; }

    template <typename A>
    typename std::enable_if<std::is_integral<A>::value, CPP>::type
    operator + (A a) const
    {
        return CPP(addr() + a * sizeof(T), proxy);
    }

    template <typename A>
    typename std::enable_if<std::is_integral<A>::value, CPP>::type
    operator - (A a) const
    {
        return CPP(addr() - a * sizeof(T), proxy);
    }

    ptrdiff_t
    operator - (const CPP &other) const
    {
        return (addr() - other.addr()) / sizeof(T);
    }

    CPP &
    operator = (const CPP &other)
    {
        proxy = other.proxy;
        buffer = other.buffer;
        return *this;
    }

    CPP &
    operator = (const Addr &a)
    {
        setAddr(a);
        return *this;
    }

    operator const T *() const
    {
        return buffer ? &buffer->template asConst<T>() : nullptr;
    }

    const T &
    operator *() const
    {
        nullCheck();
        return buffer->template asConst<T>();
    }
    const T *
    operator ->() const
    {
        nullCheck();
        return &buffer->template asConst<T>();
    }
};

template <typename T, typename Proxy, typename A>
typename std::enable_if<std::is_integral<A>::value,
                        ConstProxyPtr<T, Proxy>>::type
operator + (A a, const ConstProxyPtr<T, Proxy> &other)
{
    return other + a;
}

template <typename T, typename Proxy>
class ProxyPtr : public ConstProxyPtr<T, Proxy>
{
  protected:
    using CPP = ConstProxyPtr<T, Proxy>;
    using PP = ProxyPtr<T, Proxy>;

    ProxyPtr(Addr _ptr, std::shared_ptr<Proxy> _proxy) : CPP(_ptr, _proxy) {}

  public:
    template <typename ...Args,
              typename std::enable_if<std::is_constructible<
                  Proxy, Args&&...>::value, int>::type = 0>
    explicit ProxyPtr(Addr _ptr, Args&&... args) : CPP(_ptr, args...) {}
    template <typename ...Args,
              typename std::enable_if<std::is_constructible<
                  Proxy, Args&&...>::value, int>::type = 0>
    explicit ProxyPtr(Args&&... args) : CPP(0, args...) {}

    template <typename O, typename Enabled=
        typename std::enable_if<std::is_assignable<T *, O *>::value>::type>
    ProxyPtr(const ProxyPtr<O, Proxy> &other) : CPP(other) {}

    ProxyPtr(const PP &other) : CPP(other) {}
    operator bool() const { return (bool)this->buffer; }

    void
    flush(bool force=false)
    {
        this->nullCheck();
        this->buffer->flush(force);
    }

    template <typename A>
    typename std::enable_if<std::is_integral<A>::value, PP>::type
    operator + (A a) const
    {
        return PP(this->addr() + a * sizeof(T), this->proxy);
    }

    template <typename A>
    typename std::enable_if<std::is_integral<A>::value, PP>::type
    operator - (A a) const
    {
        return PP(this->addr() - a * sizeof(T), this->proxy);
    }

    ptrdiff_t
    operator - (const PP &other) const
    {
        return (this->addr() - other.addr()) / sizeof(T);
    }

    PP &
    operator = (const PP &other)
    {
        this->proxy = other.proxy;
        this->buffer = other.buffer;
        return *this;
    }

    PP &
    operator = (const Addr &a)
    {
        this->setAddr(a);
        return *this;
    }

    using CPP::operator const T *;
    operator T *() const
    {
        return this->buffer ? &this->buffer->template as<T>() : nullptr;
    }

    using CPP::operator *;
    T &
    operator *() const
    {
        this->nullCheck();
        return this->buffer->template as<T>();
    }

    using CPP::operator ->;
    T *
    operator ->() const
    {
        this->nullCheck();
        return &this->buffer->template as<T>();
    }
};

template <typename T, typename Proxy, typename A>
typename std::enable_if<std::is_integral<A>::value, ProxyPtr<T, Proxy>>::type
operator + (A a, const ProxyPtr<T, Proxy> &other)
{
    return other + a;
}

namespace GuestABI
{

template <typename ABI, typename T, typename Proxy>
struct Argument<ABI, ProxyPtr<T, Proxy>>
{
    static ProxyPtr<T, Proxy>
    get(ThreadContext *tc, typename ABI::State &state)
    {
        return ProxyPtr<T, Proxy>(Argument<ABI, Addr>::get(tc, state), tc);
    }
};

template <typename ABI, typename T, typename Proxy>
struct Argument<ABI, ConstProxyPtr<T, Proxy>>
{
    static ConstProxyPtr<T, Proxy>
    get(ThreadContext *tc, typename ABI::State &state)
    {
        return ConstProxyPtr<T, Proxy>(
                Argument<ABI, Addr>::get(tc, state), tc);
    }
};

} // namespace GuestABI

template <typename T, typename Proxy>
std::ostream &
operator << (std::ostream &os, const ConstProxyPtr<T, Proxy> &vptr)
{
    ccprintf(os, "%#x", vptr.addr());
    return os;
}

class SETranslatingPortProxy;

template <typename T>
using ConstVPtr = ConstProxyPtr<T, SETranslatingPortProxy>;
template <typename T>
using VPtr = ProxyPtr<T, SETranslatingPortProxy>;

#endif // __SIM_PROXY_PTR_HH__
