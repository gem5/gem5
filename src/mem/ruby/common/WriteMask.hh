/*
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

#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/system/RubySystem.hh"

class WriteMask
{
  public:
    WriteMask()
      : mSize(RubySystem::getBlockSizeBytes()), mMask(mSize, false),
        mAtomic(false)
    {}

    WriteMask(int size)
      : mSize(size), mMask(size, false), mAtomic(false)
    {}

    WriteMask(int size, std::vector<bool> & mask)
      : mSize(size), mMask(mask), mAtomic(false)
    {}

    WriteMask(int size, std::vector<bool> &mask,
              std::vector<std::pair<int, AtomicOpFunctor*> > atomicOp)
      : mSize(size), mMask(mask), mAtomic(true), mAtomicOp(atomicOp)
    {}

    ~WriteMask()
    {}

    void
    clear()
    {
        mMask = std::vector<bool>(mSize, false);
    }

    bool
    test(int offset)
    {
        assert(offset < mSize);
        return mMask[offset];
    }

    void
    setMask(int offset, int len)
    {
        assert(mSize >= (offset + len));
        for (int i = 0; i < len; i++) {
            mMask[offset + i] = true;
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
    cmpMask(const WriteMask &readMask) const
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

    bool isEmpty() const
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
    orMask(const WriteMask & writeMask)
    {
        assert(mSize == writeMask.mSize);
        for (int i = 0; i < mSize; i++) {
            mMask[i] = (mMask.at(i)) | (writeMask.mMask.at(i));
        }

        if (writeMask.mAtomic) {
            mAtomic = true;
            mAtomicOp = writeMask.mAtomicOp;
        }
    }

    void print(std::ostream& out) const;

    void
    performAtomic(uint8_t * p) const
    {
        for (int i = 0; i < mAtomicOp.size(); i++) {
            int offset = mAtomicOp[i].first;
            AtomicOpFunctor *fnctr = mAtomicOp[i].second;
            (*fnctr)(&p[offset]);
        }
    }

    void
    performAtomic(DataBlock & blk) const
    {
        for (int i = 0; i < mAtomicOp.size(); i++) {
            int offset = mAtomicOp[i].first;
            uint8_t *p = blk.getDataMod(offset);
            AtomicOpFunctor *fnctr = mAtomicOp[i].second;
            (*fnctr)(p);
        }
    }
  private:
    int mSize;
    std::vector<bool> mMask;
    bool mAtomic;
    std::vector<std::pair<int, AtomicOpFunctor*> > mAtomicOp;
};

inline std::ostream&
operator<<(std::ostream& out, const WriteMask& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_COMMON_WRITEMASK_HH__
