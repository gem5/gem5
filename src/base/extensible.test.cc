/*
 * Copyright 2023 Google, Inc.
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

#include <memory>

#include "base/extensible.hh"

using namespace gem5;

namespace
{

class TestTarget : public Extensible<TestTarget>
{
};

class IntegerExtension : public Extension<TestTarget, IntegerExtension>
{
  public:
    explicit IntegerExtension(uint32_t data) : data_(data) {}

    std::unique_ptr<gem5::ExtensionBase>
    clone() const override
    {
        return std::unique_ptr<IntegerExtension>(new IntegerExtension(data_));
    }

    uint32_t
    getData() const
    {
        return data_;
    }

  private:
    uint32_t data_;
};

class BoolExtension : public Extension<TestTarget, BoolExtension>
{
  public:
    explicit BoolExtension(bool data) : data_(data) {}

    std::unique_ptr<gem5::ExtensionBase>
    clone() const override
    {
        return std::unique_ptr<BoolExtension>(new BoolExtension(data_));
    }

    bool
    getData() const
    {
        return data_;
    }

  private:
    bool data_;
};

} // namespace

TEST(ExtensibleTest, ExtensionID)
{
    std::shared_ptr<IntegerExtension> ext1(new IntegerExtension(0xabcd));
    EXPECT_EQ(0, ext1->getExtensionID());

    std::shared_ptr<BoolExtension> ext2(new BoolExtension(true));
    EXPECT_EQ(1, ext2->getExtensionID());
}

TEST(ExtensibleTest, SetAndRemoveExtension)
{
    const uint32_t data = 0xbeef;
    std::shared_ptr<IntegerExtension> ext(new IntegerExtension(data));
    std::unique_ptr<TestTarget> target(new TestTarget);
    target->setExtension(ext);
    EXPECT_EQ(data, target->getExtension<IntegerExtension>()->getData());

    target->removeExtension<IntegerExtension>();
    EXPECT_EQ(nullptr, target->getExtension<IntegerExtension>());
}

TEST(ExtensibleTest, ReplaceExtension)
{
    const uint32_t data = 0xbeef;
    std::shared_ptr<IntegerExtension> ext(new IntegerExtension(data));
    std::unique_ptr<TestTarget> target(new TestTarget);
    target->setExtension(ext);
    EXPECT_EQ(data, target->getExtension<IntegerExtension>()->getData());

    const uint32_t new_data = 0xa5a5;
    std::shared_ptr<IntegerExtension> new_ext(new IntegerExtension(new_data));
    target->setExtension(new_ext);
    EXPECT_EQ(new_data, target->getExtension<IntegerExtension>()->getData());
}
