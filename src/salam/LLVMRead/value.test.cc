/*
 * Copyright (c) 2026 Akanksha Chaudhari
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

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>

#include "llvm/IR/Constants.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Type.h"
#include "salam/LLVMRead/value.hh"

namespace
{

// A buffer of exactly `size` bytes that ends where an inaccessible
// page starts, so reading past it faults.
class GuardedBuffer
{
  public:
    explicit GuardedBuffer(size_t size)
    {
        pageSize = sysconf(_SC_PAGESIZE);
        base = static_cast<uint8_t *>(
            mmap(nullptr, 2 * pageSize, PROT_READ | PROT_WRITE,
                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
        EXPECT_NE(base, MAP_FAILED);
        EXPECT_EQ(mprotect(base + pageSize, pageSize, PROT_NONE), 0);
        bytes = base + pageSize - size;
    }

    ~GuardedBuffer() { munmap(base, 2 * pageSize); }

    uint8_t *bytes;

  private:
    uint8_t *base;
    size_t pageSize;
};

std::shared_ptr<SALAM::Value>
makeValue(llvm::Type *type)
{
    auto value = std::make_shared<SALAM::Value>(1, nullptr, false);
    SALAM::irvmap map;
    value->initialize(llvm::UndefValue::get(type), &map);
    return value;
}

// Little-endian bytes of v, as the ARM target stores them.
void
storeTarget(uint8_t *dest, uint64_t v, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        dest[i] = uint8_t(v >> (8 * i));
    }
}

} // anonymous namespace

TEST(SalamValueRegister, IntegersReadOnlyTheirWidth)
{
    llvm::LLVMContext ctx;
    for (unsigned bits : {8u, 16u, 32u, 64u}) {
        size_t size = bits / 8;
        uint64_t expected = 0x8877665544332211ULL;
        if (bits < 64) {
            expected &= (1ULL << bits) - 1;
        }
        GuardedBuffer buf(size);
        storeTarget(buf.bytes, expected, size);
        auto value = makeValue(llvm::Type::getIntNTy(ctx, bits));
        value->setRegisterValue(buf.bytes);
        EXPECT_EQ(value->getUIntRegValue(), expected) << "i" << bits;
    }
}

TEST(SalamValueRegister, FloatingPointReadsOnlyItsWidth)
{
    llvm::LLVMContext ctx;
    float f = -1.5f;
    uint32_t fbits;
    std::memcpy(&fbits, &f, sizeof(f));
    GuardedBuffer fbuf(4);
    storeTarget(fbuf.bytes, fbits, 4);
    auto fvalue = makeValue(llvm::Type::getFloatTy(ctx));
    fvalue->setRegisterValue(fbuf.bytes);
    EXPECT_EQ(fvalue->getFloatFromReg(), f);

    double d = 2.25;
    uint64_t dbits;
    std::memcpy(&dbits, &d, sizeof(d));
    GuardedBuffer dbuf(8);
    storeTarget(dbuf.bytes, dbits, 8);
    auto dvalue = makeValue(llvm::Type::getDoubleTy(ctx));
    dvalue->setRegisterValue(dbuf.bytes);
    EXPECT_EQ(dvalue->getDoubleFromReg(), d);
}

TEST(SalamValueRegister, UnalignedBuffers)
{
    llvm::LLVMContext ctx;
    alignas(8) uint8_t storage[16] = {};
    uint8_t *odd = storage + 1;
    storeTarget(odd, 0x12345678, 4);
    auto value = makeValue(llvm::Type::getInt32Ty(ctx));
    value->setRegisterValue(odd);
    EXPECT_EQ(value->getUIntRegValue(), 0x12345678u);
}
