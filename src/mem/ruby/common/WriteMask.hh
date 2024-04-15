/*
 * Copyright (c) 2020,2021 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2012-15 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_COMMON_WRITEMASK_HH__
#define __MEM_RUBY_COMMON_WRITEMASK_HH__

#include <cassert>
#include <iomanip>
#include <iostream>
#include <vector>

#include "base/amo.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/TypeDefines.hh"

namespace gem5
{

namespace ruby
{

class WriteMask
{
  public:
    typedef std::vector<std::pair<int, AtomicOpFunctor *>> AtomicOpVector;

    WriteMask();

    WriteMask(int size) : mSize(size), mMask(size, false), mAtomic(false) {}

    WriteMask(int size, std::vector<bool> &mask)
        : mSize(size), mMask(mask), mAtomic(false)
    {}

    WriteMask(int size, std::vector<bool> &mask, AtomicOpVector atomicOp)
        : mSize(size), mMask(mask), mAtomic(true), mAtomicOp(atomicOp)
    {}

    ~WriteMask() {}

    void
    clear()
    {
        mMask = std::vector<bool>(mSize, false);
    }

    bool
    test(int offset) const
    {
        assert(offset < mSize);
        return mMask[offset];
    }

    void
    setMask(int offset, int len, bool val = true)
    {
        assert(mSize >= (offset + len));
        for (int i = 0; i < len; i++) {
            mMask[offset + i] = val;
        }
    }

    void
    fillMask()
    {
        for (int i = 0; i < mSize; i++) {
            mMask[i] = true;
        }
    }

    bool
    getMask(int offset, int len) const
    {
        bool tmp = true;
        assert(mSize >= (offset + len));
        for (int i = 0; i < len; i++) {
            tmp = tmp & mMask.at(offset + i);
        }
        return tmp;
    }

    bool
    isOverlap(const WriteMask &readMask) const
    {
        bool tmp = false;
        assert(mSize == readMask.mSize);
        for (int i = 0; i < mSize; i++) {
            if (readMask.mMask.at(i)) {
                tmp = tmp | mMask.at(i);
            }
        }
        return tmp;
    }

    bool
    containsMask(const WriteMask &readMask) const
    {
        bool tmp = true;
        assert(mSize == readMask.mSize);
        for (int i = 0; i < mSize; i++) {
            if (readMask.mMask.at(i)) {
                tmp = tmp & mMask.at(i);
            }
        }
        return tmp;
    }

    bool
    isEmpty() const
    {
        for (int i = 0; i < mSize; i++) {
            if (mMask.at(i)) {
                return false;
            }
        }
        return true;
    }

    bool
    isFull() const
    {
        for (int i = 0; i < mSize; i++) {
            if (!mMask.at(i)) {
                return false;
            }
        }
        return true;
    }

    void
    andMask(const WriteMask &writeMask)
    {
        assert(mSize == writeMask.mSize);
        for (int i = 0; i < mSize; i++) {
            mMask[i] = (mMask.at(i)) && (writeMask.mMask.at(i));
        }

        if (writeMask.mAtomic) {
            mAtomic = true;
            mAtomicOp = writeMask.mAtomicOp;
        }
    }

    void
    orMask(const WriteMask &writeMask)
    {
        assert(mSize == writeMask.mSize);
        for (int i = 0; i < mSize; i++) {
            mMask[i] = (mMask.at(i)) || (writeMask.mMask.at(i));
        }

        if (writeMask.mAtomic) {
            mAtomic = true;
            mAtomicOp = writeMask.mAtomicOp;
        }
    }

    void
    setInvertedMask(const WriteMask &writeMask)
    {
        assert(mSize == writeMask.mSize);
        for (int i = 0; i < mSize; i++) {
            mMask[i] = !writeMask.mMask.at(i);
        }
    }

    int
    firstBitSet(bool val, int offset = 0) const
    {
        for (int i = offset; i < mSize; ++i)
            if (mMask[i] == val)
                return i;
        return mSize;
    }

    int
    count(int offset = 0) const
    {
        int count = 0;
        for (int i = offset; i < mSize; ++i)
            count += mMask[i];
        return count;
    }

    void print(std::ostream &out) const;

    /*
     * Performs atomic operations on the data block pointed to by p. The
     * atomic operations to perform are in the vector mAtomicOp. The
     * effect of each atomic operation is pushed to the atomicChangeLog
     * so that each individual atomic requestor may see the results of their
     * specific atomic operation.
     */
    void performAtomic(uint8_t *p, std::deque<uint8_t *> &atomicChangeLog,
                       bool isAtomicNoReturn = true) const;

    const AtomicOpVector &
    getAtomicOps() const
    {
        return mAtomicOp;
    }

    void
    setAtomicOps(const AtomicOpVector &atomicOps)
    {
        mAtomic = true;
        mAtomicOp = std::move(atomicOps);
    }

  private:
    int mSize;
    std::vector<bool> mMask;
    bool mAtomic;
    AtomicOpVector mAtomicOp;
};

inline std::ostream &
operator<<(std::ostream &out, const WriteMask &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_COMMON_WRITEMASK_HH__
