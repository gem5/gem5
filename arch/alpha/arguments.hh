/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __ARGUMENTS_HH__
#define __ARGUMENTS_HH__

#include <assert.h>

#include "arch/alpha/vtophys.hh"
#include "base/refcnt.hh"
#include "sim/host.hh"

class ExecContext;

class AlphaArguments
{
  protected:
    ExecContext *xc;
    int number;
    uint64_t getArg(bool fp = false);

  protected:
    class Data : public RefCounted
    {
      public:
        Data(){}
        ~Data();

      private:
        std::list<char *> data;

      public:
        char *alloc(size_t size);
    };

    RefCountingPtr<Data> data;

  public:
    AlphaArguments(ExecContext *ctx, int n = 0)
        : xc(ctx), number(n), data(NULL)
        { assert(number >= 0); data = new Data;}
    AlphaArguments(const AlphaArguments &args)
        : xc(args.xc), number(args.number), data(args.data) {}
    ~AlphaArguments() {}

    ExecContext *getExecContext() const { return xc; }

    const AlphaArguments &operator=(const AlphaArguments &args) {
        xc = args.xc;
        number = args.number;
        data = args.data;
        return *this;
    }

    AlphaArguments &operator++() {
        ++number;
        assert(number >= 0);
        return *this;
    }

    AlphaArguments operator++(int) {
        AlphaArguments args = *this;
        ++number;
        assert(number >= 0);
        return args;
    }

    AlphaArguments &operator--() {
        --number;
        assert(number >= 0);
        return *this;
    }

    AlphaArguments operator--(int) {
        AlphaArguments args = *this;
        --number;
        assert(number >= 0);
        return args;
    }

    const AlphaArguments &operator+=(int index) {
        number += index;
        assert(number >= 0);
        return *this;
    }

    const AlphaArguments &operator-=(int index) {
        number -= index;
        assert(number >= 0);
        return *this;
    }

    AlphaArguments operator[](int index) {
        return AlphaArguments(xc, index);
    }

    template <class T>
    operator T() {
        assert(sizeof(T) <= sizeof(uint64_t));
        T data = static_cast<T>(getArg());
        return data;
    }

    template <class T>
    operator T *() {
        T *buf = (T *)data->alloc(sizeof(T));
        CopyData(xc, buf, getArg(), sizeof(T));
        return buf;
    }

    operator char *() {
        char *buf = data->alloc(2048);
        CopyString(xc, buf, getArg(), 2048);
        return buf;
    }
};

#endif // __ARGUMENTS_HH__
