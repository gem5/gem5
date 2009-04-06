/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#ifndef __BASE_FLAGS_HH__
#define __BASE_FLAGS_HH__

template <typename T>
class Flags
{
  private:
    T _flags;

  public:
    typedef T Type;
    Flags() : _flags(0) {}
    Flags(Type flags) : _flags(flags) {}

    operator const Type() const { return _flags; }

    template <typename U> 
    const Flags<T> &
    operator=(const Flags<U> &flags)
    {
        _flags = flags._flags;
        return *this;
    }

    const Flags<T> &
    operator=(T flags)
    {
        _flags = flags;
        return *this;
    }
    
    bool isSet() const { return _flags; }
    bool isSet(Type flags) const { return (_flags & flags); }
    bool allSet() const { return !(~_flags); }
    bool allSet(Type flags) const { return (_flags & flags) == flags; }
    bool noneSet() const { return _flags == 0; }
    bool noneSet(Type flags) const { return (_flags & flags) == 0; }
    void clear() { _flags = 0; }
    void clear(Type flags) { _flags &= ~flags; }
    void set(Type flags) { _flags |= flags; }
    void set(Type f, bool val) { _flags = (_flags & ~f) | (val ? f : 0); }
    void
    update(Type flags, Type mask)
    {
        _flags = (_flags & ~mask) | (flags & mask);
    }
};

#endif // __BASE_FLAGS_HH__
