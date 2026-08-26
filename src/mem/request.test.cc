/*
 * Copyright (c) 2026 harukz
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

#include "mem/request.hh"

namespace gem5
{

TEST(Request, CopyConstructorPreservesStreamId)
{
    Request original;
    original.setStreamId(42);

    Request copied(original);

    ASSERT_TRUE(copied.hasStreamId());
    EXPECT_EQ(copied.streamId(), 42);
}

TEST(Request, CopyConstructorPreservesSubstreamId)
{
    Request original;
    original.setStreamId(42);
    original.setSubstreamId(7);

    Request copied(original);

    ASSERT_TRUE(copied.hasSubstreamId());
    EXPECT_EQ(copied.substreamId(), 7);
}

TEST(Request, CopyConstructorPreservesInstCount)
{
    Request original;
    original.setInstCount(123);

    Request copied(original);

    ASSERT_TRUE(copied.hasInstCount());
    EXPECT_EQ(copied.getInstCount(), 123);
}

TEST(Request, CopyConstructorPreservesHtmAbortCause)
{
    Request original;
    original.setPaddr(0);
    original.setFlags(Request::HTM_ABORT);
    original.setHtmAbortCause(HtmFailureFaultCause::EXPLICIT);

    Request copied(original);

    ASSERT_TRUE(copied.hasHtmAbortCause());
    EXPECT_EQ(
        copied.getHtmAbortCause(),
        HtmFailureFaultCause::EXPLICIT);
}

TEST(Request, CopyConstructorPreservesSystemReq)
{
    Request original;
    original.setSystemReq(true);

    Request copied(original);

    EXPECT_TRUE(copied.systemReq());
}

TEST(Request, CopyConstructorPreservesGPUFuncAccess)
{
    Request original;
    original.setGPUFuncAccess(true);

    Request copied(original);

    EXPECT_TRUE(copied.getGPUFuncAccess());
}
} // namespace gem5
