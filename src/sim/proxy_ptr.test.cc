/*
 * Copyright 2020 Google, Inc.
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

#include <vector>

#include "sim/proxy_ptr.hh"

using namespace gem5;

struct Access
{
    bool read;
    Addr addr;
    Addr size;

    Access(bool _read, Addr _addr, Addr _size) :
        read(_read), addr(_addr), size(_size)
    {}

    bool
    operator == (const Access &other) const
    {
        return read == other.read &&
               addr == other.addr &&
               size == other.size;
    }

    bool
    operator != (const Access &other) const
    {
        return !(*this == other);
    }
};

using Accesses = std::vector<Access>;

class BackingStore
{
  public:
    std::vector<uint8_t> store;
    Addr base;

    BackingStore(Addr _base, size_t _size) : store(_size, 0), base(_base) {}

    void
    rangeCheck(Addr addr, Addr size)
    {
        panic_if(addr < base || addr + size > base + store.size(),
                 "Range [%#x,%#x) outside of [%#x,%#x).",
                 addr, addr + size, base, base + store.size());
    }

    mutable Accesses accesses;

    ::testing::AssertionResult
    expect_access(size_t idx, const Access &other) const
    {
        if (idx >= accesses.size()) {
            return ::testing::AssertionFailure() << "index " << idx <<
                " out of bounds";
        }

        if (accesses[idx] != other) {
            return ::testing::AssertionFailure() << "access[" << idx <<
                "] was " << accesses[idx] << ", expected " << other;
        }
        return ::testing::AssertionSuccess();
    }

    ::testing::AssertionResult
    expect_accesses(Accesses expected) const
    {
        if (accesses.size() != expected.size()) {
            return ::testing::AssertionFailure() <<
                "Wrong number of accesses, was " << accesses.size() <<
                " expected " << expected.size();
        }

        auto failure = ::testing::AssertionFailure();
        bool success = true;
        if (accesses.size() == expected.size()) {
            for (size_t idx = 0; idx < expected.size(); idx++) {
                auto result = expect_access(idx, expected[idx]);
                if (!result) {
                    failure << result.message();
                    success = false;
                }
            }
        }

        if (!success)
            return failure;
        else
            return ::testing::AssertionSuccess();
    }

    void
    writeBlob(Addr ptr, const void *data, int size)
    {
        rangeCheck(ptr, size);
        accesses.emplace_back(false, ptr, size);
        memcpy(store.data() + (ptr - base), data, size);
    }

    void
    readBlob(Addr ptr, void *data, int size)
    {
        rangeCheck(ptr, size);
        accesses.emplace_back(true, ptr, size);
        memcpy(data, store.data() + (ptr - base), size);
    }
};

::testing::AssertionResult
accessed(const char *expr1, const char *expr2,
         const BackingStore &store, const Accesses &expected)
{
    return store.expect_accesses(expected);
}

#define EXPECT_ACCESSES(store, ...) \
    do { \
        Accesses expected({__VA_ARGS__}); \
        EXPECT_PRED_FORMAT2(accessed, store, expected); \
        store.accesses.clear(); \
    } while (false)

std::ostream &
operator << (std::ostream &os, const Access &access)
{
    ccprintf(os, "%s(%#x, %d)", access.read ? "read" : "write",
            access.addr, access.size);
    return os;
}

class TestProxy
{
  public:
    BackingStore &store;

    TestProxy(BackingStore &_store) : store(_store) {}
    // Sneaky constructor for testing guest_abi integration.
    TestProxy(ThreadContext *tc) : store(*(BackingStore *)tc) {}

    void
    writeBlob(Addr ptr, const void *data, int size)
    {
        store.writeBlob(ptr, data, size);
    }

    void
    readBlob(Addr ptr, void *data, int size)
    {
        store.readBlob(ptr, data, size);
    }
};

template <typename T>
using TestPtr = ProxyPtr<T, TestProxy>;

template <typename T>
using ConstTestPtr = ConstProxyPtr<T, TestProxy>;

TEST(ProxyPtr, Clean)
{
    BackingStore store(0x1000, 0x1000);

    EXPECT_ACCESSES(store);

    {
        ConstTestPtr<uint32_t> test_ptr(0x1100, store);

        EXPECT_ACCESSES(store, { true, test_ptr.addr(), sizeof(uint32_t) });
    }

    EXPECT_ACCESSES(store);

    {
        TestPtr<uint32_t> test_ptr(0x1100, store);

        EXPECT_ACCESSES(store, { true, test_ptr.addr(), sizeof(uint32_t) });
    }

    EXPECT_ACCESSES(store);
}

TEST(ProxyPtr, Dirty)
{
    BackingStore store(0x1000, 0x1100);

    EXPECT_ACCESSES(store);

    {
        TestPtr<uint32_t> test_ptr(0x1100, store);

        *test_ptr = 0xa5a5a5a5;

        EXPECT_ACCESSES(store, { true, test_ptr.addr(), sizeof(uint32_t) });
    }

    EXPECT_ACCESSES(store, { false, 0x1100, sizeof(uint32_t) });
    EXPECT_EQ(store.store[0x100], 0xa5);
    EXPECT_EQ(store.store[0x101], 0xa5);
    EXPECT_EQ(store.store[0x102], 0xa5);
    EXPECT_EQ(store.store[0x103], 0xa5);
}


TEST(ProxyPtr, LoadAndFlush)
{
    BackingStore store(0x1000, 0x1100);

    store.store[0x100] = 0xa5;
    store.store[0x101] = 0xa5;
    store.store[0x102] = 0xa5;
    store.store[0x103] = 0xa5;

    TestPtr<uint32_t> test_ptr(0x1100, store);

    // Check that the backing store is unmodified.
    EXPECT_EQ(store.store[0x100], 0xa5);
    EXPECT_EQ(store.store[0x101], 0xa5);
    EXPECT_EQ(store.store[0x102], 0xa5);
    EXPECT_EQ(store.store[0x103], 0xa5);

    // Change the value in our local buffered copy.
    *test_ptr = 0x5a5a5a5a;

    // Verify that the backing store hasn't been changed.
    EXPECT_EQ(store.store[0x100], 0xa5);
    EXPECT_EQ(store.store[0x101], 0xa5);
    EXPECT_EQ(store.store[0x102], 0xa5);
    EXPECT_EQ(store.store[0x103], 0xa5);

    // Flush out our modifications.
    test_ptr.flush();

    // Verify that they've been written back to the store.
    EXPECT_EQ(store.store[0x100], 0x5a);
    EXPECT_EQ(store.store[0x101], 0x5a);
    EXPECT_EQ(store.store[0x102], 0x5a);
    EXPECT_EQ(store.store[0x103], 0x5a);

    // Update the store and try to flush again.
    store.store[0x100] = 0xaa;
    test_ptr.flush();

    // Verify that no flush happened, since our ptr was "clean".
    EXPECT_EQ(store.store[0x100], 0xaa);

    // Force a flush.
    test_ptr.flush(true);

    // Verify that the flush happened even though the ptr was "clean".
    EXPECT_EQ(store.store[0x100], 0x5a);

    // Update the store.
    store.store[0x100] = 0xa5;
    store.store[0x101] = 0xa5;
    store.store[0x102] = 0xa5;
    store.store[0x103] = 0xa5;

    // Verify that our local copy hasn't changed.
    EXPECT_EQ(*(const uint32_t *)test_ptr, 0x5a5a5a5a);

    // Reload the pointer from the store.
    test_ptr.load();
    EXPECT_EQ(*(const uint32_t *)test_ptr, 0xa5a5a5a5);
}

TEST(ProxyPtr, ConstOperators)
{
    bool is_same;

    BackingStore store(0x1000, 0x1000);

    const Addr addr1 = 0x1100;
    const Addr addr2 = 0x1200;

    using PtrType = uint32_t;

    ConstTestPtr<PtrType> test_ptr1(addr1, store);
    EXPECT_EQ(test_ptr1.addr(), addr1);

    ConstTestPtr<PtrType> test_ptr2(addr2, store);
    EXPECT_EQ(test_ptr2.addr(), addr2);

    // Pointer +/- integer.
    auto next_ptr = test_ptr1 + 2;
    EXPECT_EQ(next_ptr.addr(), addr1 + 2 * sizeof(PtrType));

    auto reverse_next_ptr = 2 + test_ptr1;
    EXPECT_EQ(reverse_next_ptr.addr(), addr1 + 2 * sizeof(PtrType));

    auto prev_ptr = test_ptr1 - 2;
    EXPECT_EQ(prev_ptr.addr(), addr1 - 2 * sizeof(PtrType));

    // Pointer-pointer subtraction.
    auto diff = test_ptr2 - test_ptr1;
    EXPECT_EQ(diff, (addr2 - addr1) / sizeof(PtrType));

    // Assignment.
    ConstTestPtr<PtrType> target(addr2, store);
    EXPECT_EQ(target.addr(), addr2);

    target = test_ptr1;
    EXPECT_EQ(target.addr(), addr1);

    // Conversions.
    EXPECT_TRUE(test_ptr1);
    ConstTestPtr<PtrType> null(0, store);
    EXPECT_FALSE(null);

    EXPECT_NE((const PtrType *)test_ptr1, nullptr);
    EXPECT_EQ((const PtrType *)null, nullptr);

    // Dereferences.
    is_same = std::is_same_v<decltype(*test_ptr1), const PtrType &>;
    EXPECT_TRUE(is_same);

    store.store[0x100] = 0x55;
    store.store[0x101] = 0x55;
    store.store[0x102] = 0x55;
    store.store[0x103] = 0x55;

    // Force an update since we changed the backing store behind our ptrs back.
    test_ptr1.load();

    EXPECT_EQ(*test_ptr1, 0x55555555);

    store.store[0x100] = 0x11;
    store.store[0x101] = 0x22;
    store.store[0x102] = 0x33;
    store.store[0x103] = 0x44;

    struct TestStruct
    {
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
    };

    ConstTestPtr<TestStruct> struct_ptr(addr1, store);
    EXPECT_EQ(struct_ptr->a, 0x11);
    EXPECT_EQ(struct_ptr->b, 0x22);
    EXPECT_EQ(struct_ptr->c, 0x33);
    EXPECT_EQ(struct_ptr->d, 0x44);

    is_same = std::is_same_v<decltype((struct_ptr->a)), const uint8_t &>;
    EXPECT_TRUE(is_same);
}

TEST(ProxyPtr, NonConstOperators)
{
    bool is_same;

    BackingStore store(0x1000, 0x1000);

    const Addr addr1 = 0x1100;
    const Addr addr2 = 0x1200;

    using PtrType = uint32_t;

    TestPtr<PtrType> test_ptr1(addr1, store);
    EXPECT_EQ(test_ptr1.addr(), addr1);

    TestPtr<PtrType> test_ptr2(addr2, store);
    EXPECT_EQ(test_ptr2.addr(), addr2);

    // Pointer +/- integer.
    auto next_ptr = test_ptr1 + 2;
    EXPECT_EQ(next_ptr.addr(), addr1 + 2 * sizeof(PtrType));

    auto reverse_next_ptr = 2 + test_ptr1;
    EXPECT_EQ(reverse_next_ptr.addr(), addr1 + 2 * sizeof(PtrType));

    auto prev_ptr = test_ptr1 - 2;
    EXPECT_EQ(prev_ptr.addr(), addr1 - 2 * sizeof(PtrType));

    // Pointer-pointer subtraction.
    auto diff = test_ptr2 - test_ptr1;
    EXPECT_EQ(diff, (addr2 - addr1) / sizeof(PtrType));

    // Assignment.
    TestPtr<PtrType> target(addr2, store);
    EXPECT_EQ(target.addr(), addr2);

    target = test_ptr1;
    EXPECT_EQ(target.addr(), addr1);

    // Conversions.
    EXPECT_TRUE(test_ptr1);
    TestPtr<PtrType> null(0, store);
    EXPECT_FALSE(null);

    EXPECT_NE((PtrType *)test_ptr1, nullptr);
    EXPECT_EQ((PtrType *)null, nullptr);
    EXPECT_NE((const PtrType *)test_ptr1, nullptr);
    EXPECT_EQ((const PtrType *)null, nullptr);

    // Dereferences.
    is_same = std::is_same_v<decltype(*test_ptr1), PtrType &>;
    EXPECT_TRUE(is_same);

    // Flush test_ptr1, which has been conservatively marked as dirty.
    test_ptr1.flush();

    store.store[0x100] = 0x55;
    store.store[0x101] = 0x55;
    store.store[0x102] = 0x55;
    store.store[0x103] = 0x55;

    // Force an update since we changed the backing store behind our ptrs back.
    test_ptr1.load();

    EXPECT_EQ(*test_ptr1, 0x55555555);

    store.store[0x100] = 0x11;
    store.store[0x101] = 0x22;
    store.store[0x102] = 0x33;
    store.store[0x103] = 0x44;

    struct TestStruct
    {
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
    };

    TestPtr<TestStruct> struct_ptr(addr1, store);
    EXPECT_EQ(struct_ptr->a, 0x11);
    EXPECT_EQ(struct_ptr->b, 0x22);
    EXPECT_EQ(struct_ptr->c, 0x33);
    EXPECT_EQ(struct_ptr->d, 0x44);

    is_same = std::is_same_v<decltype((struct_ptr->a)), uint8_t &>;
    EXPECT_TRUE(is_same);
}

struct TestABI
{
    using UintPtr = uint64_t;
    using State = int;
};

namespace gem5
{

namespace guest_abi
{

template <>
struct Argument<TestABI, Addr>
{
    static Addr
    get(ThreadContext *tc, typename TestABI::State &state)
    {
        return 0x1000;
    }
};

} // namespace guest_abi
} // namespace gem5

bool abiCalled = false;
bool abiCalledConst = false;

void
abiTestFunc(ThreadContext *tc, TestPtr<uint8_t> ptr)
{
    abiCalled = true;
    EXPECT_EQ(ptr.addr(), 0x1000);
}

void
abiTestFuncConst(ThreadContext *tc, ConstTestPtr<uint8_t> ptr)
{
    abiCalledConst = true;
    EXPECT_EQ(ptr.addr(), 0x1000);
}

TEST(ProxyPtrTest, GuestABI)
{
    BackingStore store(0x1000, 0x1000);

    EXPECT_FALSE(abiCalled);
    EXPECT_FALSE(abiCalledConst);

    invokeSimcall<TestABI>((ThreadContext *)&store, abiTestFunc);

    EXPECT_TRUE(abiCalled);
    EXPECT_FALSE(abiCalledConst);

    invokeSimcall<TestABI>((ThreadContext *)&store, abiTestFuncConst);

    EXPECT_TRUE(abiCalled);
    EXPECT_TRUE(abiCalledConst);
}
