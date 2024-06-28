/*
 * Copyright (c) 2016-2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __CPU_INST_RES_HH__
#define __CPU_INST_RES_HH__

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <variant>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

class InstResult
{
  private:
    using BlobPtr = std::unique_ptr<const uint8_t[]>;

    std::variant<BlobPtr, RegVal> value;
    const RegClass *_regClass = nullptr;

    bool blob() const { return std::holds_alternative<BlobPtr>(value); }
    bool valid() const { return _regClass != nullptr; }

    // Raw accessors with no safety checks.
    RegVal getRegVal() const { return std::get<RegVal>(value); }
    const void *getBlob() const { return std::get<BlobPtr>(value).get(); }

    // Store copies of blobs, not a pointer to the original.
    void
    set(const void *val)
    {
        uint8_t *temp = nullptr;
        if (val) {
            const size_t size = _regClass->regBytes();
            temp = new uint8_t[size];
            std::memcpy(temp, val, size);
        }
        value = BlobPtr(temp);
    }

    void set(RegVal val) { value = val; }

    void
    set(const InstResult &other)
    {
        other.blob() ? set(other.getBlob()) : set(other.getRegVal());
    }

  public:
    /** Default constructor creates an invalid result. */
    InstResult() {}
    InstResult(const InstResult &other) : _regClass(other._regClass)
    {
        set(other);
    }

    InstResult(const RegClass &reg_class, RegVal val) :
        _regClass(&reg_class)
    {
        set(val);
    }

    InstResult(const RegClass &reg_class, const void *val) :
        _regClass(&reg_class)
    {
        set(val);
    }

    InstResult &
    operator=(const InstResult &that)
    {
        _regClass = that._regClass;
        set(that);

        return *this;
    }

    /**
     * Result comparison
     * Two invalid results always differ.
     */
    bool
    operator==(const InstResult& that) const
    {
        if (blob() != that.blob() || _regClass != that._regClass)
            return false;

        if (blob()) {
            const void *my_blob = getBlob();
            const void *their_blob = that.getBlob();

            // Invalid results always differ.
            if (!my_blob || !their_blob)
                return false;

            // Check the contents of the blobs, not their addresses.
            return std::memcmp(getBlob(), that.getBlob(),
                    _regClass->regBytes()) == 0;
        } else {
            return getRegVal() == that.getRegVal();
        }
    }

    bool
    operator!=(const InstResult& that) const
    {
        return !operator==(that);
    }

    const RegClass &regClass() const { return *_regClass; }
    bool isValid() const { return valid(); }
    bool isBlob() const { return blob(); }

    RegVal
    asRegVal() const
    {
        assert(!blob());
        return getRegVal();
    }

    const void *
    asBlob() const
    {
        assert(blob());
        return getBlob();
    }

    std::string
    asString() const
    {
        if (blob()) {
            return _regClass->valString(getBlob());
        } else {
            RegVal reg = getRegVal();
            return _regClass->valString(&reg);
        }
    }

    std::string
    asString(const int64_t& num_bytes) const
    {
        assert(blob());
        return _regClass->valString(getBlob(), num_bytes);
    }
};

} // namespace gem5

#endif // __CPU_INST_RES_HH__
