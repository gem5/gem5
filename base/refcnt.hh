/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#ifndef __REFCNT_HH__
#define __REFCNT_HH__

class RefCounted
{
  private:
    int count;

  private:
    RefCounted(const RefCounted &);

  public:
    RefCounted() : count(0) {}
    virtual ~RefCounted() {}

    void incref() { ++count; }
    void decref() { if (--count <= 0) delete this; }
};

template <class T>
class RefCountingPtr
{
  private:
    T *data;

    void copy(T *d) {
        data = d;
        if (data)
            data->incref();
    }
    void del() {
        if (data)
            data->decref();
    }

  public:
    RefCountingPtr() : data(NULL) {}
    RefCountingPtr(T *data) { copy(data); }
    RefCountingPtr(const RefCountingPtr &r) { copy(r.data); }
    ~RefCountingPtr() { del(); }

    T *operator->() { return data; }
    T &operator*() { return *data; }
    T *get() { return data; }

    const T *operator->() const { return data; }
    const T &operator*() const { return *data; }
    const T *get() const { return data; }

    RefCountingPtr &operator=(T *p) {
        if (data != p) {
            del();
            copy(p);
        }
        return *this;
    }

    RefCountingPtr &operator=(const RefCountingPtr &r) {
        if (data != r.data) {
            del();
            copy(r.data);
        }
        return *this;
    }

    bool operator!() const { return data == 0; }
    operator bool() const { return data != 0; }
};

template<class T>
bool operator==(const RefCountingPtr<T> &l, const RefCountingPtr<T> &r)
{ return l.get() == r.get(); }

template<class T>
bool operator==(const RefCountingPtr<T> &l, const T *r)
{ return l.get() == r; }

template<class T>
bool operator==(const T &l, const RefCountingPtr<T> &r)
{ return l == r.get(); }

template<class T>
bool operator!=(const RefCountingPtr<T> &l, const RefCountingPtr<T> &r)
{ return l.get() != r.get(); }

template<class T>
bool operator!=(const RefCountingPtr<T> &l, const T *r)
{ return l.get() != r; }

template<class T>
bool operator!=(const T &l, const RefCountingPtr<T> &r)
{ return l != r.get(); }

#endif // __REFCNT_HH__
