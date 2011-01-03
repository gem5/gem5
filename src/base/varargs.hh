/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 */

#ifndef __BASE_VARARGS_HH__
#define __BASE_VARARGS_HH__

#include "base/refcnt.hh"

#define VARARGS_DECLARATION(receiver)                  \
    VarArgs::Argument<receiver> a01 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a02 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a03 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a04 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a05 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a06 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a07 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a08 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a09 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a10 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a11 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a12 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a13 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a14 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a15 = VarArgs::Null(), \
    VarArgs::Argument<receiver> a16 = VarArgs::Null()

#define VARARGS_DEFINITION(receiver) \
    VarArgs::Argument<receiver> a01, \
    VarArgs::Argument<receiver> a02, \
    VarArgs::Argument<receiver> a03, \
    VarArgs::Argument<receiver> a04, \
    VarArgs::Argument<receiver> a05, \
    VarArgs::Argument<receiver> a06, \
    VarArgs::Argument<receiver> a07, \
    VarArgs::Argument<receiver> a08, \
    VarArgs::Argument<receiver> a09, \
    VarArgs::Argument<receiver> a10, \
    VarArgs::Argument<receiver> a11, \
    VarArgs::Argument<receiver> a12, \
    VarArgs::Argument<receiver> a13, \
    VarArgs::Argument<receiver> a14, \
    VarArgs::Argument<receiver> a15, \
    VarArgs::Argument<receiver> a16

#define VARARGS_ALLARGS                     \
    a01, a02, a03, a04, a05, a06, a07, a08, \
    a09, a10, a11, a12, a13, a14, a15, a16

#define VARARGS_ADDARGS(receiver) do { \
    do {                           \
        if (!a01) break;           \
        a01.add_arg(receiver);     \
        if (!a02) break;           \
        a02.add_arg(receiver);     \
        if (!a03) break;           \
        a03.add_arg(receiver);     \
        if (!a04) break;           \
        a04.add_arg(receiver);     \
        if (!a05) break;           \
        a05.add_arg(receiver);     \
        if (!a06) break;           \
        a06.add_arg(receiver);     \
        if (!a07) break;           \
        a07.add_arg(receiver);     \
        if (!a08) break;           \
        a08.add_arg(receiver);     \
        if (!a09) break;           \
        a09.add_arg(receiver);     \
        if (!a10) break;           \
        a10.add_arg(receiver);     \
        if (!a11) break;           \
        a11.add_arg(receiver);     \
        if (!a12) break;           \
        a12.add_arg(receiver);     \
        if (!a13) break;           \
        a13.add_arg(receiver);     \
        if (!a14) break;           \
        a14.add_arg(receiver);     \
        if (!a15) break;           \
        a15.add_arg(receiver);     \
        if (!a16) break;           \
        a16.add_arg(receiver);     \
    } while (0);                   \
    receiver.end_args();           \
} while (0)

namespace VarArgs {

struct Null {};

template <typename T>
struct Traits
{
    enum { enabled = true };
};

template <>
struct Traits<Null>
{
    enum { enabled = false };
};

template <class RECV>
struct Base : public RefCounted
{
    virtual void add_arg(RECV &receiver) const = 0;
};

template <typename T, class RECV>
struct Any : public Base<RECV>
{
    const T &argument;

    Any(const T &arg) : argument(arg) {}

    virtual void
    add_arg(RECV &receiver) const
    {
        receiver.add_arg(argument);
    }
};

template <typename T, class RECV>
struct Any<T *, RECV> : public Base<RECV>
{
    const T *argument;

    Any(const T *arg) : argument(arg) {}

    virtual void
    add_arg(RECV &receiver) const
    {
        receiver.add_arg(argument);
    }
};

template <class RECV>
struct Argument : public RefCountingPtr<Base<RECV> >
{
    typedef RefCountingPtr<VarArgs::Base<RECV> > Base;

    Argument() { }
    Argument(const Null &null) { }
    template <typename T>
    Argument(const T& arg) : Base(new Any<T, RECV>(arg)) { }
    template <typename T>
    Argument(const T* arg) : Base(new Any<T *, RECV>(arg)) { }

    void
    add_arg(RECV &receiver) const
    {
        if (this->data)
            this->data->add_arg(receiver);
    }
};

template<class RECV>
class List
{
  public:
    typedef VarArgs::Argument<RECV> Argument;
    typedef std::list<Argument> list;
    typedef typename list::iterator iterator;
    typedef typename list::const_iterator const_iterator;
    typedef typename list::size_type size_type;

  protected:
    list l;

  public:
    List() {}
    List(Argument a01, Argument a02, Argument a03, Argument a04,
         Argument a05, Argument a06, Argument a07, Argument a08,
         Argument a09, Argument a10, Argument a11, Argument a12,
         Argument a13, Argument a14, Argument a15, Argument a16)
    {
        if (!a01) return;
        l.push_back(a01);
        if (!a02) return;
        l.push_back(a02);
        if (!a03) return;
        l.push_back(a03);
        if (!a04) return;
        l.push_back(a04);
        if (!a05) return;
        l.push_back(a05);
        if (!a06) return;
        l.push_back(a06);
        if (!a07) return;
        l.push_back(a07);
        if (!a08) return;
        l.push_back(a08);
        if (!a09) return;
        l.push_back(a09);
        if (!a10) return;
        l.push_back(a10);
        if (!a11) return;
        l.push_back(a11);
        if (!a12) return;
        l.push_back(a12);
        if (!a13) return;
        l.push_back(a13);
        if (!a14) return;
        l.push_back(a14);
        if (!a15) return;
        l.push_back(a15);
        if (!a16) return;
        l.push_back(a16);
    }

    size_type size() const { return l.size(); }
    bool empty() const { return l.empty(); }

    iterator begin() { return l.begin(); }
    const_iterator begin() const { return l.begin(); }

    iterator end() { return l.end(); }
    const_iterator end() const { return l.end(); }

    void
    push_back(const Argument &arg)
    {
        if (arg)
            l.push_back(arg);
    }

    void
    push_front(const Argument &arg)
    {
        if (arg)
            l.push_front(arg);
    }

    template <typename T>
    void
    push_back(const T &arg)
    {
        if (Traits<T>::enabled)
            l.push_back(arg);
    }

    template <typename T>
    void
    push_front(const T &arg)
    {
        if (Traits<T>::enabled)
            l.push_front(arg);
    }

    Argument& front() { return l.front(); }
    const Argument& front() const { return l.front(); }
    Argument& back() { return l.back(); }
    const Argument& back() const { return l.back(); }

    void erase(iterator position) { return l.erase(position); }
    void erase(iterator first, iterator last) { return l.erase(first, last); }
    void clear() { return l.clear(); }
    void pop_front() { return l.pop_front(); }
    void pop_back() { return l.pop_back(); }
    void reverse() { l.reverse(); }

    /*
     * Functions specific to variable arguments
     */
    void
    add_args(RECV &recv) const
    {
        const_iterator i = l.begin();
        const_iterator end = l.end();
        while (i != end) {
            i->add_arg(recv);
            ++i;
        }

        recv.end_args();
    }
};

} // namespace VarArgs

#endif /* __BASE_VARARGS_HH__ */
