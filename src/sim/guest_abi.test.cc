/*
 * Copyright 2019 Google, Inc.
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

#include <type_traits>
#include <utility>

#include "sim/guest_abi.hh"

using namespace gem5;

namespace gem5
{
// Fake ThreadContext which holds data and captures results.
class ThreadContext
{
  public:
    static const int ints[];
    static const double floats[];

    static const int DefaultIntResult;
    static const double DefaultFloatResult;

    int intResult = DefaultIntResult;
    double floatResult = DefaultFloatResult;

    int intOffset = 0;
};

const int ThreadContext::ints[] = {
    0, 1, 2, 3, 4, 5, 6, 7
};
const double ThreadContext::floats[] = {
    10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0
};

const int ThreadContext::DefaultIntResult = 0;
const double ThreadContext::DefaultFloatResult = 0.0;

} // namespace gem5

// ABI anchor for an ABI which has 1D progress. Conceptually, this could be
// because integer and floating point arguments are stored in the same
// registers.
struct TestABI_1D
{
    using State = int;
};

// ABI anchor for an ABI which uses the prepare() hook.
struct TestABI_Prepare
{
    using State = int;
};

// ABI anchor for an ABI which has 2D progress. Conceptually, this could be
// because integer and floating point arguments are stored in separate
// registers.
struct TestABI_2D
{
    using State = std::pair<int, int>;
};

struct TestABI_TcInit
{
    struct State
    {
        int pos;
        State(const ThreadContext *tc) : pos(tc->intOffset) {}
    };
};

namespace gem5
{

namespace guest_abi
{

// Hooks for the 1D ABI arguments and return value. Add 1 or 1.0 to return
// values so we can tell they went through the right set of hooks.
template <>
struct Argument<TestABI_1D, int>
{
    static int
    get(ThreadContext *tc, TestABI_1D::State &state)
    {
        return tc->ints[state++];
    }
};

template <typename Arg>
struct Argument<TestABI_1D, Arg,
    typename std::enable_if_t<std::is_floating_point_v<Arg>>>
{
    static Arg
    get(ThreadContext *tc, TestABI_1D::State &state)
    {
        return tc->floats[state++];
    }
};

template <>
struct Result<TestABI_1D, int>
{
    static void
    store(ThreadContext *tc, const int &ret)
    {
        tc->intResult = ret + 1;
    }
};

template <typename Ret>
struct Result<TestABI_1D, Ret,
    typename std::enable_if_t<std::is_floating_point_v<Ret>>>
{
    static void
    store(ThreadContext *tc, const Ret &ret)
    {
        tc->floatResult = ret + 1.0;
    }
};

// Hooks for the ABI which uses prepare(). It uses the same rules as the
// 1D ABI for arguments, but allocates space for and discards return values
// and returns integer arguments in reverse order.
template <>
struct Argument<TestABI_Prepare, int>
{
    static int
    get(ThreadContext *tc, TestABI_Prepare::State &state)
    {
        return tc->ints[--state];
    }

    static void
    prepare(ThreadContext *tc, TestABI_Prepare::State &state)
    {
        state++;
    }
};

template <typename Ret>
struct Result<TestABI_Prepare, Ret>
{
    static void store(ThreadContext *tc, const Ret &ret) {}
    static void
    prepare(ThreadContext *tc, TestABI_Prepare::State &state)
    {
        state++;
    }
};

// Hooks for the 2D ABI arguments and return value. Add 2 or 2.0 to return
// values so we can tell they went through the right set of hooks.

template <>
struct Argument<TestABI_2D, int>
{
    static int
    get(ThreadContext *tc, TestABI_2D::State &state)
    {
        return tc->ints[state.first++];
    }
};

template <typename Arg>
struct Argument<TestABI_2D, Arg,
    typename std::enable_if_t<std::is_floating_point_v<Arg>>>
{
    static Arg
    get(ThreadContext *tc, TestABI_2D::State &state)
    {
        return tc->floats[state.second++];
    }
};

template <>
struct Result<TestABI_2D, int>
{
    static void
    store(ThreadContext *tc, const int &ret)
    {
        tc->intResult = ret + 2;
    }
};

template <typename Ret>
struct Result<TestABI_2D, Ret,
    typename std::enable_if_t<std::is_floating_point_v<Ret>>>
{
    static void
    store(ThreadContext *tc, const Ret &ret)
    {
        tc->floatResult = ret + 2.0;
    }
};

// Hooks for the TcInit ABI arguments.
template <>
struct Argument<TestABI_TcInit, int>
{
    static int
    get(ThreadContext *tc, TestABI_TcInit::State &state)
    {
        return tc->ints[state.pos++];
    }
};

} // namespace guest_abi
} // namespace gem5

// Test function which verifies that its arguments reflect the 1D ABI and
// which doesn't return anything.
void
testIntVoid(ThreadContext *tc, int a, float b, int c, double d,
            guest_abi::VarArgs<int,float,double> varargs)
{
    EXPECT_EQ(a, tc->ints[0]);
    EXPECT_EQ(b, tc->floats[1]);
    EXPECT_EQ(c, tc->ints[2]);
    EXPECT_EQ(d, tc->floats[3]);

    EXPECT_EQ(varargs.get<int>(), tc->ints[4]);
    EXPECT_EQ(varargs.get<float>(), tc->floats[5]);
    EXPECT_EQ(varargs.get<double>(), tc->floats[6]);
}

// Test functions which verify that the return allocating ABI allocates space
// for its return value successfully.
void
testPrepareVoid(ThreadContext *tc, int a, int b)
{
    EXPECT_EQ(a, tc->ints[1]);
    EXPECT_EQ(b, tc->ints[0]);
}

int
testPrepareInt(ThreadContext *tc, int a, int b)
{
    EXPECT_EQ(a, tc->ints[2]);
    EXPECT_EQ(b, tc->ints[1]);
    return 0;
}

// Test function which verifies that its arguments reflect the 2D ABI and
// which doesn't return anything.
void
test2DVoid(ThreadContext *tc, int a, float b, int c, double d,
           guest_abi::VarArgs<int,float,double> varargs)
{
    EXPECT_EQ(a, tc->ints[0]);
    EXPECT_EQ(b, tc->floats[0]);
    EXPECT_EQ(c, tc->ints[1]);
    EXPECT_EQ(d, tc->floats[1]);

    EXPECT_EQ(varargs.get<int>(), tc->ints[2]);
    EXPECT_EQ(varargs.get<float>(), tc->floats[2]);
    EXPECT_EQ(varargs.get<double>(), tc->floats[3]);
}

void
testTcInit(ThreadContext *tc, int a)
{
    EXPECT_EQ(tc->intOffset, 2);
    EXPECT_EQ(a, tc->ints[2]);
}

// Test functions which returns various types of values.
const int IntRetValue = 50;
const float FloatRetValue = 3.14;
const double DoubleRetValue = 12.34;

int testIntRet(ThreadContext *tc) { return IntRetValue; }
float testFloatRet(ThreadContext *tc) { return FloatRetValue; }
double testDoubleRet(ThreadContext *tc) { return DoubleRetValue; }


// The actual test bodies.
TEST(GuestABITest, ABI_1D_args)
{
    ThreadContext tc;
    invokeSimcall<TestABI_1D>(&tc, testIntVoid);
    EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
    EXPECT_EQ(tc.floatResult, tc.DefaultFloatResult);
}

TEST(GuestABITest, ABI_Prepare)
{
    ThreadContext tc;
    invokeSimcall<TestABI_Prepare>(&tc, testPrepareVoid);
    invokeSimcall<TestABI_Prepare>(&tc, testPrepareInt);
}

TEST(GuestABITest, ABI_2D_args)
{
    ThreadContext tc;
    invokeSimcall<TestABI_2D>(&tc, test2DVoid);
    EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
    EXPECT_EQ(tc.floatResult, tc.DefaultFloatResult);
}

TEST(GuestABITest, ABI_TC_init)
{
    ThreadContext tc;
    tc.intOffset = 2;
    invokeSimcall<TestABI_TcInit>(&tc, testTcInit);
}

TEST(GuestABITest, ABI_returns)
{
    // 1D returns.
    {
        ThreadContext tc;
        int ret = invokeSimcall<TestABI_1D>(&tc, testIntRet);
        EXPECT_EQ(ret, IntRetValue);
        EXPECT_EQ(tc.intResult, IntRetValue + 1);
        EXPECT_EQ(tc.floatResult, tc.DefaultFloatResult);
    }
    {
        ThreadContext tc;
        float ret = invokeSimcall<TestABI_1D>(&tc, testFloatRet);
        EXPECT_EQ(ret, FloatRetValue);
        EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
        EXPECT_EQ(tc.floatResult, FloatRetValue + 1.0);
    }
    {
        ThreadContext tc;
        double ret = invokeSimcall<TestABI_1D>(&tc, testDoubleRet);
        EXPECT_EQ(ret, DoubleRetValue);
        EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
        EXPECT_EQ(tc.floatResult, DoubleRetValue + 1.0);
    }
    {
        // Disable storing the return value in the ThreadContext.
        ThreadContext tc;
        int ret = invokeSimcall<TestABI_1D, false>(&tc, testIntRet);
        EXPECT_EQ(ret, IntRetValue);
        EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
        EXPECT_EQ(tc.floatResult, tc.DefaultFloatResult);
    }


    // 2D returns.
    {
        ThreadContext tc;
        int ret = invokeSimcall<TestABI_2D>(&tc, testIntRet);
        EXPECT_EQ(ret, IntRetValue);
        EXPECT_EQ(tc.intResult, IntRetValue + 2);
        EXPECT_EQ(tc.floatResult, tc.DefaultFloatResult);
    }
    {
        ThreadContext tc;
        float ret = invokeSimcall<TestABI_2D>(&tc, testFloatRet);
        EXPECT_EQ(ret, FloatRetValue);
        EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
        EXPECT_EQ(tc.floatResult, FloatRetValue + 2.0);
    }
    {
        ThreadContext tc;
        double ret = invokeSimcall<TestABI_2D>(&tc, testDoubleRet);
        EXPECT_EQ(ret, DoubleRetValue);
        EXPECT_EQ(tc.intResult, tc.DefaultIntResult);
        EXPECT_EQ(tc.floatResult, DoubleRetValue + 2.0);
    }
}

TEST(GuestABITest, dumpSimcall)
{
    ThreadContext tc;
    std::string dump = dumpSimcall<TestABI_1D>("test", &tc, testIntVoid);
    EXPECT_EQ(dump, "test(0, 11, 2, 13, ...)");
}

TEST(GuestABITest, isVarArgs)
{
    EXPECT_TRUE(guest_abi::IsVarArgsV<guest_abi::VarArgs<int>>);
    EXPECT_FALSE(guest_abi::IsVarArgsV<int>);
    EXPECT_FALSE(guest_abi::IsVarArgsV<double>);
    struct FooStruct {};
    EXPECT_FALSE(guest_abi::IsVarArgsV<FooStruct>);
    union FooUnion {};
    EXPECT_FALSE(guest_abi::IsVarArgsV<FooUnion>);
}
