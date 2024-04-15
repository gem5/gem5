/*
 * Copyright 2022 Google, Inc.
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

#ifndef __CPU_REGFILE_HH__
#define __CPU_REGFILE_HH__

#include <algorithm>
#include <cassert>
#include <cstring>
#include <vector>

#include "cpu/reg_class.hh"

namespace gem5
{

class RegFile
{
  private:
    std::vector<uint8_t> data;
    const size_t _size;
    const size_t _regShift;
    const size_t _regBytes;

  public:
    const RegClass &regClass;

    RegFile(const RegClass &info, const size_t new_size)
        : data(new_size << info.regShift()),
          _size(new_size),
          _regShift(info.regShift()),
          _regBytes(info.regBytes()),
          regClass(info)
    {}

    RegFile(const RegClass &info) : RegFile(info, info.numRegs()) {}

    size_t
    size() const
    {
        return _size;
    }

    size_t
    regShift() const
    {
        return _regShift;
    }

    size_t
    regBytes() const
    {
        return _regBytes;
    }

    template <typename Reg = RegVal>
    Reg &
    reg(size_t idx)
    {
        assert(sizeof(Reg) == _regBytes && idx < _size);
        return *reinterpret_cast<Reg *>(data.data() + (idx << _regShift));
    }

    template <typename Reg = RegVal>
    const Reg &
    reg(size_t idx) const
    {
        assert(sizeof(Reg) == _regBytes && idx < _size);
        return *reinterpret_cast<const Reg *>(data.data() +
                                              (idx << _regShift));
    }

    void *
    ptr(size_t idx)
    {
        return data.data() + (idx << _regShift);
    }

    const void *
    ptr(size_t idx) const
    {
        return data.data() + (idx << _regShift);
    }

    void
    get(size_t idx, void *val) const
    {
        std::memcpy(val, ptr(idx), _regBytes);
    }

    void
    set(size_t idx, const void *val)
    {
        std::memcpy(ptr(idx), val, _regBytes);
    }

    void
    clear()
    {
        std::fill(data.begin(), data.end(), 0);
    }
};

} // namespace gem5

#endif // __CPU_REGFILE_HH__
