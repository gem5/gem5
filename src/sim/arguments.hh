/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __SIM_ARGUMENTS_HH__
#define __SIM_ARGUMENTS_HH__

#include <cassert>
#include <memory>

#include "mem/fs_translating_port_proxy.hh"

class ThreadContext;

class Arguments
{
  protected:
    ThreadContext *tc;
    int number;
    uint64_t getArg(uint16_t size = (uint16_t)(-1), bool fp = false);

  protected:
    class Data
    {
      public:
        Data(){}
        ~Data();

      private:
        std::list<char *> data;

      public:
        char *alloc(size_t size);
    };

    std::shared_ptr<Data> data;

  public:
    Arguments(ThreadContext *ctx, int n = 0)
        : tc(ctx), number(n), data(new Data())
    { assert(number >= 0); }
    Arguments(const Arguments &args)
        : tc(args.tc), number(args.number), data(args.data) {}
    ~Arguments() {}

    ThreadContext *getThreadContext() const { return tc; }

    const Arguments &operator=(const Arguments &args) {
        if (this != &args) {
            tc = args.tc;
            number = args.number;
            data = args.data;
        }
        return *this;
    }

    // for checking if an argument is NULL
    bool operator!() {
        return getArg() == 0;
    }

    Arguments &operator++() {
        ++number;
        assert(number >= 0);
        return *this;
    }

    Arguments operator++(int) {
        Arguments args = *this;
        ++number;
        assert(number >= 0);
        return args;
    }

    Arguments &operator--() {
        --number;
        assert(number >= 0);
        return *this;
    }

    Arguments operator--(int) {
        Arguments args = *this;
        --number;
        assert(number >= 0);
        return args;
    }

    const Arguments &operator+=(int index) {
        number += index;
        assert(number >= 0);
        return *this;
    }

    const Arguments &operator-=(int index) {
        number -= index;
        assert(number >= 0);
        return *this;
    }

    Arguments operator[](int index) {
        return Arguments(tc, index);
    }

    template <class T>
    operator T() {
        assert(sizeof(T) <= sizeof(uint64_t));
        T d = static_cast<T>(getArg(sizeof(T)));
        return d;
    }

    template <class T>
    operator T *() {
        T *buf = (T *)data->alloc(sizeof(T));
        CopyOut(tc, buf, getArg(sizeof(T)), sizeof(T));
        return buf;
    }

    operator char *() {
        char *buf = data->alloc(2048);
        CopyStringOut(tc, buf, getArg(), 2048);
        return buf;
    }
};

#endif // __SIM_ARGUMENTS_HH__
