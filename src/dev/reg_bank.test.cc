/*
 * Copyright (c) 2020, 2024 Arm Limited
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

#pragma GCC diagnostic push

// __GNUC__ defined for both clang and gcc
// -Wdeprecated-copy has been added in clang10.0.0 and gcc9.0
#if defined(__GNUC__)
#    if (defined(__clang__) && __GNUC__ >= 10) || \
        (!defined(__clang__) && __GNUC__ >= 9)
#        pragma GCC diagnostic ignored "-Wdeprecated-copy"
#    endif
#endif

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#pragma GCC diagnostic pop

#include <vector>

#include "base/gtest/logging.hh"
#include "dev/reg_bank.hh"

using namespace gem5;

// Compare the elements of an array against expected values.
using testing::ElementsAre;
// This version is needed with enough elements, empirically more than 10.
using testing::ElementsAreArray;

using testing::AllOf;
using testing::HasSubstr;


/*
 * The RegisterRaz (read as zero) type.
 */

class RegisterRazTest : public testing::Test
{
  protected:
    static constexpr size_t BufSize = 12;
    static constexpr size_t BufOffset = 4;
    static constexpr size_t RazSize = 4;

    std::array<uint8_t, BufSize> buf;
    RegisterBankLE::RegisterRaz raz;

    RegisterRazTest() : raz("raz", RazSize)
    {
        buf.fill(0xff);
    }
};
// Needed by C++14 and lower
constexpr size_t RegisterRazTest::RazSize;

TEST_F(RegisterRazTest, Name)
{
    EXPECT_EQ(raz.name(), "raz");
}

TEST_F(RegisterRazTest, Size)
{
    EXPECT_EQ(raz.size(), RazSize);
}

// Accessing the entire register at once.
TEST_F(RegisterRazTest, FullAccess)
{
    raz.write(buf.data() + BufOffset);
    raz.read(buf.data() + BufOffset);
    EXPECT_THAT(buf, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                       0x00, 0x00, 0x00, 0x00,
                                       0xff, 0xff, 0xff, 0xff}));
}

// Partial access, excluding the start of the register.
TEST_F(RegisterRazTest, PartialAccessHigh)
{
    raz.write(buf.data() + BufOffset, 1, 3);
    raz.read(buf.data() + BufOffset, 1, 3);
    EXPECT_THAT(buf, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                       0x00, 0x00, 0x00, 0xff,
                                       0xff, 0xff, 0xff, 0xff}));
}

// Partial access, excluding the end of the register.
TEST_F(RegisterRazTest, PartialAccessLow)
{
    raz.write(buf.data() + BufOffset, 0, 3);
    raz.read(buf.data() + BufOffset, 0, 3);
    EXPECT_THAT(buf, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                       0x00, 0x00, 0x00, 0xff,
                                       0xff, 0xff, 0xff, 0xff}));
}

// Partial access, excluding both ends of the register.
TEST_F(RegisterRazTest, PartialAccessMid)
{
    raz.write(buf.data() + BufOffset, 1, 2);
    raz.read(buf.data() + BufOffset, 1, 2);
    EXPECT_THAT(buf, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                       0x00, 0x00, 0xff, 0xff,
                                       0xff, 0xff, 0xff, 0xff}));
}

TEST_F(RegisterRazTest, Serialize)
{
    std::ostringstream os;
    raz.serialize(os);
    EXPECT_EQ(os.str(), "");
}

TEST_F(RegisterRazTest, Unserialize)
{
    std::string s;
    EXPECT_TRUE(raz.unserialize(s));
}


/*
 * The RegisterRao (read as one) type.
 */

class RegisterRaoTest : public testing::Test
{
  protected:
    static constexpr size_t BufSize = 12;
    static constexpr size_t BufOffset = 4;
    static constexpr size_t RaoSize = 4;

    std::array<uint8_t, BufSize> buf;
    RegisterBankLE::RegisterRao rao;

    RegisterRaoTest() : rao("rao", RaoSize)
    {
        buf.fill(0x00);
    }
};
// Needed by C++14 and lower
constexpr size_t RegisterRaoTest::RaoSize;

TEST_F(RegisterRaoTest, Name)
{
    EXPECT_EQ(rao.name(), "rao");
}

TEST_F(RegisterRaoTest, Size)
{
    EXPECT_EQ(rao.size(), RaoSize);
}

// Accessing the entire register at once.
TEST_F(RegisterRaoTest, FullAccess)
{
    rao.write(buf.data() + BufOffset);
    rao.read(buf.data() + BufOffset);
    EXPECT_THAT(buf, ElementsAreArray({0x00, 0x00, 0x00, 0x00,
                                       0xff, 0xff, 0xff, 0xff,
                                       0x00, 0x00, 0x00, 0x00}));
}

// Partial access, excluding the start of the register.
TEST_F(RegisterRaoTest, PartialAccessHigh)
{
    rao.write(buf.data() + BufOffset, 1, 3);
    rao.read(buf.data() + BufOffset, 1, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x00, 0x00, 0x00, 0x00,
                                       0xff, 0xff, 0xff, 0x00,
                                       0x00, 0x00, 0x00, 0x00}));
}

// Partial access, excluding the end of the register.
TEST_F(RegisterRaoTest, PartialAccessLow)
{
    rao.write(buf.data() + BufOffset, 0, 3);
    rao.read(buf.data() + BufOffset, 0, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x00, 0x00, 0x00, 0x00,
                                       0xff, 0xff, 0xff, 0x00,
                                       0x00, 0x00, 0x00, 0x00}));
}

// Partial access, excluding both ends of the register.
TEST_F(RegisterRaoTest, PartialAccessMid)
{
    rao.write(buf.data() + BufOffset, 1, 2);
    rao.read(buf.data() + BufOffset, 1, 2);
    EXPECT_THAT(buf, ElementsAreArray({0x00, 0x00, 0x00, 0x00,
                                       0xff, 0xff, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00}));
}

TEST_F(RegisterRaoTest, Serialize)
{
    std::ostringstream os;
    rao.serialize(os);
    EXPECT_EQ(os.str(), "");
}

TEST_F(RegisterRaoTest, Unserialize)
{
    std::string s;
    EXPECT_TRUE(rao.unserialize(s));
}


/*
 * The RegisterBuf type.
 */

class RegisterBufTest : public testing::Test
{
  protected:
    static constexpr size_t RegSize = 4;

    std::array<uint8_t, RegSize * 3> buf;
    std::array<uint8_t, RegSize * 3> backing;

    RegisterBankLE::RegisterBuf reg;

  public:
    RegisterBufTest()
      : buf{0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc},
        backing{0x10, 0x20, 0x30, 0x40, 0x50, 0x60,
                0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0},
        reg("buf_reg", backing.data() + RegSize, RegSize)
    {}
};
// Needed by C++14 and lower
constexpr size_t RegisterBufTest::RegSize;

TEST_F(RegisterBufTest, Name)
{
    EXPECT_EQ(reg.name(), "buf_reg");
}

TEST_F(RegisterBufTest, Size)
{
    EXPECT_EQ(reg.size(), RegSize);
}

// Read the entire register.
TEST_F(RegisterBufTest, FullRead)
{
    reg.read(buf.data() + RegSize);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x50, 0x60, 0x70, 0x80,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x60, 0x70, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Write the entire register.
TEST_F(RegisterBufTest, FullWrite)
{
    reg.write(buf.data() + RegSize);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x5, 0x6, 0x7, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x5, 0x6, 0x7, 0x8,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial read, excluding the start of the register.
TEST_F(RegisterBufTest, PartialReadHigh)
{
    reg.read(buf.data() + RegSize, 1, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x60, 0x70, 0x80, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x60, 0x70, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial write, excluding the start of the register.
TEST_F(RegisterBufTest, PartialWriteHigh)
{
    reg.write(buf.data() + RegSize, 1, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x5, 0x6, 0x7, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x5, 0x6, 0x7,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial read, excluding the end of the register.
TEST_F(RegisterBufTest, PartialReadLow)
{
    reg.read(buf.data() + RegSize, 0, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x50, 0x60, 0x70, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x60, 0x70, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial write, excluding the end of the register.
TEST_F(RegisterBufTest, PartialWriteLow)
{
    reg.write(buf.data() + RegSize, 0, 3);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x5, 0x6, 0x7, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x5, 0x6, 0x7, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial read, excluding both ends of the register.
TEST_F(RegisterBufTest, PartialReadMid)
{
    reg.read(buf.data() + RegSize, 1, 2);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x60, 0x70, 0x7, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x60, 0x70, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

// Partial write, excluding both ends of the register.
TEST_F(RegisterBufTest, PartialWriteMid)
{
    reg.write(buf.data() + RegSize, 1, 2);
    EXPECT_THAT(buf, ElementsAreArray({0x1, 0x2, 0x3, 0x4,
                                       0x5, 0x6, 0x7, 0x8,
                                       0x9, 0xa, 0xb, 0xc}));
    EXPECT_THAT(backing, ElementsAreArray({0x10, 0x20, 0x30, 0x40,
                                           0x50, 0x5, 0x6, 0x80,
                                           0x90, 0xa0, 0xb0, 0xc0}));
}

TEST_F(RegisterBufTest, Serialize)
{
    std::ostringstream os;
    reg.serialize(os);
    EXPECT_EQ(os.str(), "");
}

TEST_F(RegisterBufTest, Unserialize)
{
    std::string s;
    EXPECT_TRUE(reg.unserialize(s));
}


/*
 * The RegisterLBuf type. Since it's so similar to RegisterBuf, just do a
 * basic check that it's applying it's locally managed buffer to it's parent
 * type.
 */

class RegisterLBufTest : public testing::Test
{
  protected:
    static constexpr size_t RegSize = 12;

    RegisterBankLE::RegisterLBuf<12> reg;
    std::array<uint8_t, 4> to_write;

  public:
    RegisterLBufTest() : reg("lbuf_reg"), to_write{0x1, 0x2, 0x3, 0x4}
    {
        reg.buffer.fill(0xff);
    }
};

TEST_F(RegisterLBufTest, Name)
{
    EXPECT_EQ(reg.name(), "lbuf_reg");
}

TEST_F(RegisterLBufTest, PartialWrite)
{
    reg.write(to_write.data(), 4, 4);
    EXPECT_THAT(reg.buffer, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                              0x1, 0x2, 0x3, 0x4,
                                              0xff, 0xff, 0xff, 0xff}));
}

TEST_F(RegisterLBufTest, Serialize)
{
    std::ostringstream os;
    for (int i = 0; i < reg.buffer.size(); i++)
        reg.buffer[i] = i;
    reg.serialize(os);
    EXPECT_EQ(os.str(), "0 1 2 3 4 5 6 7 8 9 10 11");
}

TEST_F(RegisterLBufTest, UnserializeSucess)
{
    std::string s = "0 1 2 3 4 5 6 7 8 9 10 11";
    EXPECT_TRUE(reg.unserialize(s));
    EXPECT_THAT(reg.buffer, ElementsAreArray({0, 1, 2, 3, 4, 5,
                                              6, 7, 8, 9, 10, 11}));
}

TEST_F(RegisterLBufTest, UnserializeFailure)
{
    std::string s = "0 1 2 3 4 5 6 7 8 9 10";
    EXPECT_FALSE(reg.unserialize(s));
    EXPECT_THAT(reg.buffer, ElementsAreArray({0xff, 0xff, 0xff, 0xff,
                                              0xff, 0xff, 0xff, 0xff,
                                              0xff, 0xff, 0xff, 0xff}));
}


/*
 * The templated Register<> type which takes a backing type and endianness
 * as template parameters.
 */

class TypedRegisterTest : public testing::Test
{
  protected:
    using BackingType = uint16_t;
    static constexpr size_t RegSize = sizeof(BackingType);

    // We'll typically test with the little endian version, since it only
    // matters for a few methods.
    RegisterBankLE::Register<BackingType> reg;
    RegisterBankBE::Register<BackingType> regBE;

    std::array<uint8_t, RegSize * 3> buf;

    TypedRegisterTest() : reg("le_reg", 0x1122), regBE("be_reg", 0x1122),
        buf{0x1, 0x2, 0x3, 0x4, 0x5, 0x6}
    {}
};
// Needed by C++14 and lower
constexpr size_t TypedRegisterTest::RegSize;

TEST_F(TypedRegisterTest, DefaultConstructor)
{
    RegisterBankLE::Register<uint32_t> def("def");
    EXPECT_EQ(def.get(), 0);
}

TEST_F(TypedRegisterTest, Name)
{
    EXPECT_EQ(reg.name(), "le_reg");
}

TEST_F(TypedRegisterTest, Size)
{
    EXPECT_EQ(reg.size(), RegSize);
}

TEST_F(TypedRegisterTest, Writable)
{
    // By default, all bits of the registers are writeable.
    EXPECT_EQ(reg.writeable(), 0xffff);
}

// Verify that get returns the initial value of the reg.
TEST_F(TypedRegisterTest, GetInitial)
{
    EXPECT_EQ(reg.get(), 0x1122);
}

TEST_F(TypedRegisterTest, Get)
{
    reg.get() = 0x1020;
    EXPECT_EQ(reg.get(), 0x1020);
    reg.get() = 0x3040;
    EXPECT_EQ(reg.get(), 0x3040);
}

// Do a full big endian read using the default read handler.
TEST_F(TypedRegisterTest, BigEndianDefaultFullRead)
{
    regBE.read(buf.data() + RegSize);
    EXPECT_EQ(regBE.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x11, 0x22, 0x5, 0x6));
}

// Do a full big endian write using the default write handler.
TEST_F(TypedRegisterTest, BigEndianDefaultFullWrite)
{
    regBE.write(buf.data() + RegSize);
    EXPECT_EQ(regBE.get(), 0x0304);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Do a partial big endian read of the low half of the register.
TEST_F(TypedRegisterTest, BigEndianDefaultPartialReadLow)
{
    regBE.read(buf.data() + RegSize, 0, 1);
    EXPECT_EQ(regBE.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x11, 0x4, 0x5, 0x6));
}

// Do a partial big endian read of the high half of the register.
TEST_F(TypedRegisterTest, BigEndianDefaultPartialReadHigh)
{
    regBE.read(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(regBE.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x22, 0x4, 0x5, 0x6));
}

// Do a partial big endian write of the low half of the register.
TEST_F(TypedRegisterTest, BigEndianDefaultPartialWriteLow)
{
    regBE.write(buf.data() + RegSize, 0, 1);
    EXPECT_EQ(regBE.get(), 0x0322);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Do a partial big endian write of the High half of the register.
TEST_F(TypedRegisterTest, BigEndianDefaultPartialWriteHigh)
{
    regBE.write(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(regBE.get(), 0x1103);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Do a full little endian read using the default read handler.
TEST_F(TypedRegisterTest, LittleEndianDefaultFullRead)
{
    reg.read(buf.data() + RegSize);
    EXPECT_EQ(reg.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x22, 0x11, 0x5, 0x6));
}

// Do a full little endian write using the default write handler.
TEST_F(TypedRegisterTest, LittleEndianDefaultFullWrite)
{
    reg.write(buf.data() + RegSize);
    EXPECT_EQ(reg.get(), 0x0403);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Do a partial little endian read of the low half of the register.
TEST_F(TypedRegisterTest, LittleEndianDefaultPartialReadLow)
{
    reg.read(buf.data() + RegSize, 0, 1);
    EXPECT_EQ(reg.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x22, 0x4, 0x5, 0x6));
}

// Do a partial little endian read of the high half of the register.
TEST_F(TypedRegisterTest, LittleEndianDefaultPartialReadHigh)
{
    reg.read(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(reg.get(), 0x1122);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x11, 0x4, 0x5, 0x6));
}

// Do a partial little endian write of the low half of the register.
TEST_F(TypedRegisterTest, LittleEndianDefaultPartialWriteLow)
{
    reg.write(buf.data() + RegSize, 0, 1);
    EXPECT_EQ(reg.get(), 0x1103);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Do a partial little endian write of the High half of the register.
TEST_F(TypedRegisterTest, LittleEndianDefaultPartialWriteHigh)
{
    reg.write(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(reg.get(), 0x0322);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
}

// Set a mask for use on writes.
TEST_F(TypedRegisterTest, SetWriteable)
{
    reg.writeable(0xff00);
    reg.write(buf.data() + RegSize);
    EXPECT_EQ(reg.get(), 0x0422);

    regBE.writeable(0xff00);
    regBE.write(buf.data() + RegSize);
    EXPECT_EQ(regBE.get(), 0x0322);
}

// Make a register read only.
TEST_F(TypedRegisterTest, ReadOnly)
{
    reg.readonly();
    reg.write(buf.data() + RegSize);
    EXPECT_EQ(reg.get(), 0x1122);
}

// Update a register with an explicit mask.
TEST_F(TypedRegisterTest, UpdateWithMask)
{
    reg.update(0xeeee, 0x0ff0);
    EXPECT_EQ(reg.get(), 0x1ee2);
}

// Update a register using the register's built in mask.
TEST_F(TypedRegisterTest, UpdateDefaultMask)
{
    reg.writeable(0xf00f);
    reg.update(0xeeee);
    EXPECT_EQ(reg.get(), 0xe12e);
}

// Set a custom read handler for a register.
TEST_F(TypedRegisterTest, Reader)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;
    BackingType ret = 0x3344;

    reg.reader([&reg_ptr, &ret](auto &r){
        reg_ptr = &r;
        return ret;
    });

    reg.read(buf.data() + RegSize);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x44, 0x33, 0x5, 0x6));
    EXPECT_EQ(reg_ptr, &reg);
}

// Set a custom read handler for a register which is a class method.
TEST_F(TypedRegisterTest, ReaderMF)
{
    using Reg = RegisterBankLE::Register<BackingType>;

    struct ReadStruct
    {
        Reg *reg_ptr = nullptr;
        BackingType ret = 0x3344;

        BackingType
        reader(Reg &r)
        {
            reg_ptr = &r;
            return ret;
        }
    } read_struct;

    reg.reader(&read_struct, &ReadStruct::reader);

    reg.read(buf.data() + RegSize);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x44, 0x33, 0x5, 0x6));
    EXPECT_EQ(read_struct.reg_ptr, &reg);
}

// Set a custom write handler for a register.
TEST_F(TypedRegisterTest, Writer)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;
    BackingType value = 0;

    reg.writer([&reg_ptr, &value](auto &r, const BackingType &v) {
        reg_ptr = &r;
        value = v;
    });

    reg.write(buf.data() + RegSize);
    EXPECT_EQ(reg_ptr, &reg);
    EXPECT_EQ(value, 0x0403);
}

// Set a custom write handler for a register which is a class method.
TEST_F(TypedRegisterTest, WriterMF)
{
    using Reg = RegisterBankLE::Register<BackingType>;

    struct WriteStruct
    {
        Reg *reg_ptr = nullptr;
        BackingType value = 0;

        void
        writer(Reg &r, const BackingType &v)
        {
            reg_ptr = &r;
            value = v;
        }
    } write_struct;

    reg.writer(&write_struct, &WriteStruct::writer);

    reg.write(buf.data() + RegSize);
    EXPECT_EQ(write_struct.reg_ptr, &reg);
    EXPECT_THAT(write_struct.value, 0x0403);
}

// Set a custom partial read handler for a register.
TEST_F(TypedRegisterTest, PartialReader)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;
    int first = 0;
    int last = 0;
    BackingType ret = 0x3344;

    reg.partialReader([&reg_ptr, &first, &last, ret](auto &r, int f, int l) {
        reg_ptr = &r;
        first = f;
        last = l;
        return ret;
    });

    reg.read(buf.data() + RegSize, 1, 1);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x33, 0x4, 0x5, 0x6));
    EXPECT_EQ(reg_ptr, &reg);
    EXPECT_EQ(first, 15);
    EXPECT_EQ(last, 8);
}

// Set a custom partial read handler for a register which is a class method.
TEST_F(TypedRegisterTest, PartialReaderMF)
{
    using Reg = RegisterBankLE::Register<BackingType>;

    struct ReadStruct
    {
        Reg *reg_ptr = nullptr;
        int first = 0;
        int last = 0;
        BackingType ret = 0x3344;

        BackingType
        reader(Reg &r, int f, int l)
        {
            reg_ptr = &r;
            first = f;
            last = l;
            return ret;
        }
    } read_struct;

    reg.partialReader(&read_struct, &ReadStruct::reader);

    reg.read(buf.data() + RegSize, 1, 1);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x33, 0x4, 0x5, 0x6));
    EXPECT_EQ(read_struct.reg_ptr, &reg);
    EXPECT_EQ(read_struct.first, 15);
    EXPECT_EQ(read_struct.last, 8);
}

// Set a custom partial write handler for a register.
TEST_F(TypedRegisterTest, PartialWriter)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;
    BackingType value = 0;
    int first = 0;
    int last = 0;

    reg.partialWriter([&reg_ptr, &value, &first, &last](
                auto &r, const BackingType &v, int f, int l) {
        reg_ptr = &r;
        value = v;
        first = f;
        last = l;
    });

    reg.write(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(reg_ptr, &reg);
    EXPECT_EQ(value, 0x300);
    EXPECT_EQ(first, 15);
    EXPECT_EQ(last, 8);
}

// Set a custom partial write handler for a register which is a class method.
TEST_F(TypedRegisterTest, PartialWriterMF)
{
    using Reg = RegisterBankLE::Register<BackingType>;

    struct WriteStruct
    {
        Reg *reg_ptr = nullptr;
        BackingType value = 0;
        int first = 0;
        int last = 0;

        void
        writer(Reg &r, const BackingType &v, int f, int l)
        {
            reg_ptr = &r;
            value = v;
            first = f;
            last = l;
        }
    } write_struct;

    reg.partialWriter(&write_struct, &WriteStruct::writer);

    reg.write(buf.data() + RegSize, 1, 1);
    EXPECT_EQ(write_struct.reg_ptr, &reg);
    EXPECT_EQ(write_struct.value, 0x300);
    EXPECT_EQ(write_struct.first, 15);
    EXPECT_EQ(write_struct.last, 8);
}

// Default partial reader with a custom read handler.
TEST_F(TypedRegisterTest, PartialReaderReader)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;
    BackingType ret = 0x3344;

    reg.reader([&reg_ptr, &ret](auto &r){
        reg_ptr = &r;
        return ret;
    });

    reg.read(buf.data() + RegSize, 1, 1);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x33, 0x4, 0x5, 0x6));
    EXPECT_EQ(reg_ptr, &reg);
}

// Default partial writer with custome read and write handlers.
TEST_F(TypedRegisterTest, PartialWriterReaderWriter)
{
    RegisterBankLE::Register<BackingType> *read_reg_ptr = nullptr;
    BackingType read_ret = 0x3344;

    RegisterBankLE::Register<BackingType> *write_reg_ptr = nullptr;
    BackingType write_value = 0;

    reg.reader([&read_reg_ptr, read_ret](auto &r){
        read_reg_ptr = &r;
        return read_ret;
    }).writer([&write_reg_ptr, &write_value](auto &r, const BackingType &v) {
        write_reg_ptr = &r;
        write_value = v;
    });

    reg.write(buf.data() + RegSize, 1, 1);
    EXPECT_THAT(buf, ElementsAre(0x1, 0x2, 0x3, 0x4, 0x5, 0x6));
    EXPECT_EQ(read_reg_ptr, &reg);
    EXPECT_EQ(write_reg_ptr, &reg);
    EXPECT_EQ(write_value, 0x0344);
}

// Use the default resetter for a register.
TEST_F(TypedRegisterTest, DefaultResetter)
{
    BackingType initial_value = reg.get();

    reg.get() = initial_value + 1;
    EXPECT_EQ(reg.get(), initial_value + 1);

    reg.reset();

    EXPECT_EQ(reg.get(), initial_value);
}

// Set initial value later than constructor
TEST_F(TypedRegisterTest, LateInitialValueAssignment)
{
    BackingType initial_value = reg.get();
    BackingType new_initial_value = initial_value + 1;

    reg.get() = new_initial_value;
    reg.resetInitialValue();

    EXPECT_EQ(reg.get(), new_initial_value);
    EXPECT_EQ(reg.initialValue(), new_initial_value);

    reg.get() = new_initial_value + 1;
    EXPECT_EQ(reg.get(), new_initial_value + 1);
    EXPECT_EQ(reg.initialValue(), new_initial_value);

    reg.reset();

    EXPECT_EQ(reg.get(), new_initial_value);
    EXPECT_EQ(reg.initialValue(), new_initial_value);
}

// Set a custom resetter for a register.
TEST_F(TypedRegisterTest, Resetter)
{
    RegisterBankLE::Register<BackingType> *reg_ptr = nullptr;

    reg.resetter([&reg_ptr](auto &r) {
        reg_ptr = &r;
    });

    reg.reset();

    EXPECT_EQ(reg_ptr, &reg);
}

// Set a custom resetter for a register which is a class method.
TEST_F(TypedRegisterTest, ResetterMF)
{
    using Reg = RegisterBankLE::Register<BackingType>;

    struct ResetStruct
    {
        Reg *reg_ptr = nullptr;

        void
        resetter(Reg &r)
        {
            reg_ptr = &r;
        }
    } reset_struct;

    reg.resetter(&reset_struct, &ResetStruct::resetter);

    reg.reset();

    EXPECT_EQ(reset_struct.reg_ptr, &reg);
}

TEST_F(TypedRegisterTest, Serialize)
{
    std::ostringstream os;
    reg.serialize(os);
    EXPECT_EQ(os.str(), "4386");
}

TEST_F(TypedRegisterTest, UnserializeSucess)
{
    std::string s = "1234";
    EXPECT_TRUE(reg.unserialize(s));
    EXPECT_EQ(reg.get(), 1234);
}

TEST_F(TypedRegisterTest, UnserializeFailure)
{
    std::string s = "not_a_number";
    EXPECT_FALSE(reg.unserialize(s));
}

/*
 * The RegisterBank itself.
 */

class RegisterBankTest : public testing::Test
{
  protected:
    class TestRegBank : public RegisterBankLE
    {
      public:
        TestRegBank(const std::string &new_name, Addr new_base) :
            RegisterBankLE(new_name, new_base)
        {}
    };

    enum AccessType
    {
        Read,
        Write,
        PartialRead,
        PartialWrite
    };

    struct Access
    {
        AccessType type;
        uint32_t value = 0;
        int first = 0;
        int last = 0;
        uint32_t ret = 0;

        Access(AccessType _type) : type(_type) {}
        Access(AccessType _type, uint32_t _value,
                int _first, int _last, uint32_t _ret) :
            type(_type), value(_value),
            first(_first), last(_last), ret(_ret)
        {}

        bool
        operator == (const Access &other) const
        {
            return type == other.type && value == other.value &&
                first == other.first && last == other.last &&
                ret == other.ret;
        }
    };

    // A 32 bit register which keeps track of what happens to it.
    class TestReg : public TestRegBank::Register32
    {
      public:
        std::vector<Access> accesses;

        TestReg(const std::string &new_name, uint32_t initial) :
            TestRegBank::Register32(new_name, initial)
        {
            reader([this](auto &r) {
                Access access(Read);
                access.ret = defaultReader(r);
                accesses.push_back(access);
                return access.ret;
            });
            writer([this](auto &r, const uint32_t &v) {
                Access access(Write);
                access.value = v;
                defaultWriter(r, v);
                accesses.push_back(access);
            });
            partialReader([this](auto &r, int f, int l) {
                Access access(PartialRead);
                access.first = f;
                access.last = l;
                access.ret = defaultPartialReader(r, f, l);
                accesses.push_back(access);
                return access.ret;
            });
            partialWriter([this](auto &r, const uint32_t &v, int f, int l) {
                Access access(PartialWrite);
                access.value = v;
                access.first = f;
                access.last = l;
                defaultPartialWriter(r, v, f, l);
                accesses.push_back(access);
            });
        }
    };

    TestReg reg0, reg1, reg2;
    TestRegBank emptyBank, fullBank;

    std::array<uint8_t, 12> buf;

    RegisterBankTest() :
        reg0("reg0", 0xd3d2d1d0), reg1("reg1", 0xe3e2e1e0),
        reg2("reg2", 0xf3f2f1f0),
        emptyBank("empty", 0x12345), fullBank("full", 0x1000),
        buf{0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
            0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc}
    {
        fullBank.addRegisters({reg0, reg1, reg2});
    }
};

// Some basic accessors.

TEST_F(RegisterBankTest, Name)
{
    EXPECT_EQ(emptyBank.name(), "empty");
    EXPECT_EQ(fullBank.name(), "full");
}

TEST_F(RegisterBankTest, Base)
{
    EXPECT_EQ(emptyBank.base(), 0x12345);
    EXPECT_EQ(fullBank.base(), 0x1000);
}

// Adding registers, and the size accessor. With registers, size is boring.
TEST_F(RegisterBankTest, AddRegistersSize)
{
    EXPECT_EQ(emptyBank.size(), 0);
    emptyBank.addRegister(reg0);
    EXPECT_EQ(emptyBank.size(), 4);
    emptyBank.addRegisters({reg1, reg2});
    EXPECT_EQ(emptyBank.size(), 12);
}

TEST_F(RegisterBankTest, AddRegistersWithOffsetChecks)
{
    emptyBank.addRegister({0x12345});
    EXPECT_EQ(emptyBank.size(), 0);
    emptyBank.addRegister({0x12345, reg0});
    EXPECT_EQ(emptyBank.size(), 4);
    emptyBank.addRegister({0x12349});
    EXPECT_EQ(emptyBank.size(), 4);

    emptyBank.addRegisters({{0x12349, reg1}, {0x1234d}, {0x1234d, reg2}});
    EXPECT_EQ(emptyBank.size(), 12);
}

/**
 * This test is using addRegistersAt method to store
 * overlapping registers to the empty bank. This should not
 * be permitted and the method should panic
 *
 *            [  reg2  ]
 *       [  reg1  ]    |
 *  [  reg0  ]    |    |
 *  |        |    |    |
 * 0x0      0x4  0x6  0x8
 */
TEST_F(RegisterBankTest, AddRegistersAtOffsetDeath)
{
    gtestLogOutput.str("");

    auto base = emptyBank.base();
    EXPECT_ANY_THROW(
        emptyBank.addRegistersAt<RegisterBankLE::RegisterRao>(
            {{base + 0x0, reg0},
             {base + 0x2, reg1},
             {base + 0x4, reg2}}));

    std::string actual = gtestLogOutput.str();
    EXPECT_THAT(actual, HasSubstr("Overlapping register"));
    EXPECT_THAT(actual, HasSubstr("reg1"));
}

/**
 * This test is using addRegistersAt method to store
 * contiguous registers to the empty bank, similarly
 * to what we would do when relying on addRegisters.
 * The test will check size is updated consistently
 * with the latter method
 *
 *  [  reg0  ][  reg1  ][  reg2  ]
 *  |         |         |        |
 * 0x0       0x4       0x8      0xc
 */
TEST_F(RegisterBankTest, AddRegistersAtOffsetContiguous)
{
    auto base = emptyBank.base();
    EXPECT_EQ(emptyBank.size(), 0);
    emptyBank.addRegistersAt<RegisterBankLE::RegisterRao>(
        {{base + 0x0, reg0},
         {base + 0x4, reg1},
         {base + 0x8, reg2}});
    EXPECT_EQ(emptyBank.size(), 0xc);
}

/**
 * This test is using addRegistersAt method to store
 * non-contiguous registers to the empty bank.
 * As the RegisterRao data type is passed as a template
 * argument, the gaps between the registers are filled
 * with rao registers.
 * We check raos are correctly inserted
 *
 *  [reg0][rao0][reg1][rao1][reg2]
 *  |           |           |    |
 * 0x0         0x8         0x10 0x14
 */
TEST_F(RegisterBankTest, AddRegistersAtOffsetSparse)
{
    auto base = emptyBank.base();
    EXPECT_EQ(emptyBank.size(), 0);
    emptyBank.addRegistersAt<RegisterBankLE::RegisterRao>(
        {{base + 0x0, reg0},
         {base + 0x8, reg1},
         {base + 0x10, reg2}});
    EXPECT_EQ(emptyBank.size(), 0x14);

    emptyBank.read(base + 0x0, buf.data(), 4);
    EXPECT_EQ(reg0.get(), *reinterpret_cast<uint32_t*>(buf.data()));

    emptyBank.read(base + 0x4, buf.data(), 4);
    EXPECT_EQ(0xffffffff, *reinterpret_cast<uint32_t*>(buf.data()));

    emptyBank.read(base + 0x8, buf.data(), 4);
    EXPECT_EQ(reg1.get(), *reinterpret_cast<uint32_t*>(buf.data()));

    emptyBank.read(base + 0xc, buf.data(), 4);
    EXPECT_EQ(0xffffffff, *reinterpret_cast<uint32_t*>(buf.data()));

    emptyBank.read(base + 0x10, buf.data(), 4);
    EXPECT_EQ(reg2.get(), *reinterpret_cast<uint32_t*>(buf.data()));
}

TEST_F(RegisterBankTest, BadRegisterOffsetDeath)
{
    gtestLogOutput.str("");
    EXPECT_ANY_THROW(emptyBank.addRegisters({{0xabcd, reg0}, reg1}));

    std::string actual = gtestLogOutput.str();
    EXPECT_THAT(actual, HasSubstr("empty.reg0"));
    EXPECT_THAT(actual, HasSubstr("to be 0xabcd"));
    EXPECT_THAT(actual, HasSubstr("is 0x12345"));
}

TEST_F(RegisterBankTest, BadBankOffsetDeath)
{
    gtestLogOutput.str("");
    EXPECT_ANY_THROW(emptyBank.addRegisters({{0xabcd}, reg0}));

    std::string actual = gtestLogOutput.str();
    EXPECT_THAT(actual, HasSubstr("empty "));
    EXPECT_THAT(actual, HasSubstr("to be 0xabcd"));
    EXPECT_THAT(actual, HasSubstr("is 0x12345"));
}

// Reads.

TEST_F(RegisterBankTest, ReadOneAlignedFirst)
{
    fullBank.read(0x1000, buf.data() + 4, 4);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xd0, 0xd1, 0xd2, 0xd3,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0)
                ));
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, ReadOneAlignedMid)
{
    fullBank.read(0x1004, buf.data() + 4, 4);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xe0, 0xe1, 0xe2, 0xe3,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, ReadOneAlignedLast)
{
    fullBank.read(0x1008, buf.data() + 4, 4);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xf0, 0xf1, 0xf2, 0xf3,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0)
                ));
}

TEST_F(RegisterBankTest, ReadTwoAligned)
{
    fullBank.read(0x1004, buf.data() + 2, 8);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0xe0, 0xe1,
                                       0xe2, 0xe3, 0xf0, 0xf1,
                                       0xf2, 0xf3, 0xbb, 0xcc}));
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0)
                ));
}

TEST_F(RegisterBankTest, ReadContained)
{
    fullBank.read(0x1001, buf.data() + 4, 2);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xd1, 0xd2, 0x77, 0x88,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(PartialRead, 0, 23, 8, 0x00d2d100)
                ));
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, ReadOneSpanning)
{
    fullBank.read(0x1002, buf.data() + 4, 4);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xd2, 0xd3, 0xe0, 0xe1,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(PartialRead, 0, 31, 16, 0xd3d20000)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0),
                Access(PartialRead, 0, 15, 0, 0x0000e1e0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, ReadTwoSpanning)
{
    fullBank.read(0x1002, buf.data() + 2, 8);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0xd2, 0xd3,
                                       0xe0, 0xe1, 0xe2, 0xe3,
                                       0xf0, 0xf1, 0xbb, 0xcc}));
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(PartialRead, 0, 31, 16, 0xd3d20000)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0),
                Access(PartialRead, 0, 15, 0, 0x0000f1f0)
                ));
}

TEST_F(RegisterBankTest, ReadPartialFull)
{
    fullBank.read(0x1002, buf.data() + 4, 6);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xd2, 0xd3, 0xe0, 0xe1,
                                       0xe2, 0xe3, 0xbb, 0xcc}));
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(PartialRead, 0, 31, 16, 0xd3d20000)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, ReadFullPartial)
{
    fullBank.read(0x1004, buf.data() + 4, 6);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xe0, 0xe1, 0xe2, 0xe3,
                                       0xf0, 0xf1, 0xbb, 0xcc}));
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0),
                Access(PartialRead, 0, 15, 0, 0x0000f1f0)
                ));
}

TEST_F(RegisterBankTest, ReadLastPartial)
{
    fullBank.read(0x100a, buf.data() + 4, 2);
    EXPECT_THAT(buf, ElementsAreArray({0x11, 0x22, 0x33, 0x44,
                                       0xf2, 0xf3, 0x77, 0x88,
                                       0x99, 0xaa, 0xbb, 0xcc}));
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0),
                Access(PartialRead, 0, 31, 16, 0xf3f20000)
                ));
}

// Write.

TEST_F(RegisterBankTest, WriteOneAlignedFirst)
{
    fullBank.write(0x1000, buf.data() + 4, 4);
    EXPECT_EQ(reg0.get(), 0x88776655);
    EXPECT_EQ(reg1.get(), 0xe3e2e1e0);
    EXPECT_EQ(reg2.get(), 0xf3f2f1f0);
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Write, 0x88776655, 0, 0, 0)
                ));
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, WriteOneAlignedMid)
{
    fullBank.write(0x1004, buf.data() + 4, 4);
    EXPECT_EQ(reg0.get(), 0xd3d2d1d0);
    EXPECT_EQ(reg1.get(), 0x88776655);
    EXPECT_EQ(reg2.get(), 0xf3f2f1f0);
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Write, 0x88776655, 0, 0, 0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, WriteOneAlignedLast)
{
    fullBank.write(0x1008, buf.data() + 4, 4);
    EXPECT_EQ(reg0.get(), 0xd3d2d1d0);
    EXPECT_EQ(reg1.get(), 0xe3e2e1e0);
    EXPECT_EQ(reg2.get(), 0x88776655);
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Write, 0x88776655, 0, 0, 0)
                ));
}

TEST_F(RegisterBankTest, WriteTwoAligned)
{
    fullBank.write(0x1004, buf.data() + 2, 8);
    EXPECT_EQ(reg0.get(), 0xd3d2d1d0);
    EXPECT_EQ(reg1.get(), 0x66554433);
    EXPECT_EQ(reg2.get(), 0xaa998877);
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Write, 0x66554433, 0, 0, 0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Write, 0xaa998877, 0, 0, 0)
                ));
}

TEST_F(RegisterBankTest, WriteContained)
{
    fullBank.write(0x1001, buf.data() + 4, 2);
    EXPECT_EQ(reg0.get(), 0xd36655d0);
    EXPECT_EQ(reg1.get(), 0xe3e2e1e0);
    EXPECT_EQ(reg2.get(), 0xf3f2f1f0);
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(Write, 0xd36655d0, 0, 0, 0),
                Access(PartialWrite, 0x00665500, 23, 8, 0)
                ));
    EXPECT_TRUE(reg1.accesses.empty());
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, WriteOneSpanning)
{
    fullBank.write(0x1002, buf.data() + 4, 4);
    EXPECT_EQ(reg0.get(), 0x6655d1d0);
    EXPECT_EQ(reg1.get(), 0xe3e28877);
    EXPECT_EQ(reg2.get(), 0xf3f2f1f0);
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(Write, 0x6655d1d0, 0, 0, 0),
                Access(PartialWrite, 0x66550000, 31, 16, 0)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xe3e2e1e0),
                Access(Write, 0xe3e28877, 0, 0, 0),
                Access(PartialWrite, 0x00008877, 15, 0, 0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, WriteTwoSpanning)
{
    fullBank.write(0x1002, buf.data() + 2, 8);
    EXPECT_EQ(reg0.get(), 0x4433d1d0);
    EXPECT_EQ(reg1.get(), 0x88776655);
    EXPECT_EQ(reg2.get(), 0xf3f2aa99);
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(Write, 0x4433d1d0, 0, 0, 0),
                Access(PartialWrite, 0x44330000, 31, 16, 0)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Write, 0x88776655, 0, 0, 0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0),
                Access(Write, 0xf3f2aa99, 0, 0, 0),
                Access(PartialWrite, 0x0000aa99, 15, 0, 0)
                ));
}

TEST_F(RegisterBankTest, WritePartialFull)
{
    fullBank.write(0x1002, buf.data() + 4, 6);
    EXPECT_EQ(reg0.get(), 0x6655d1d0);
    EXPECT_EQ(reg1.get(), 0xaa998877);
    EXPECT_EQ(reg2.get(), 0xf3f2f1f0);
    EXPECT_THAT(reg0.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xd3d2d1d0),
                Access(Write, 0x6655d1d0, 0, 0, 0),
                Access(PartialWrite, 0x66550000, 31, 16, 0)
                ));
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Write, 0xaa998877, 0, 0, 0)
                ));
    EXPECT_TRUE(reg2.accesses.empty());
}

TEST_F(RegisterBankTest, WriteFullPartial)
{
    fullBank.write(0x1004, buf.data() + 4, 6);
    EXPECT_EQ(reg0.get(), 0xd3d2d1d0);
    EXPECT_EQ(reg1.get(), 0x88776655);
    EXPECT_EQ(reg2.get(), 0xf3f2aa99);
    EXPECT_TRUE(reg0.accesses.empty());
    EXPECT_THAT(reg1.accesses, ElementsAre(
                Access(Write, 0x88776655, 0, 0, 0)
                ));
    EXPECT_THAT(reg2.accesses, ElementsAre(
                Access(Read, 0, 0, 0, 0xf3f2f1f0),
                Access(Write, 0xf3f2aa99, 0, 0, 0),
                Access(PartialWrite, 0x0000aa99, 15, 0, 0)
                ));
}


/*
 * VectorRegisterBank test
 */

class VectorRegisterBankTest : public testing::Test
{
  protected:
    using Register = RegisterBankLE::Register32LE;

    static constexpr size_t BankSize = 3;
    static constexpr size_t BankOffset = 8;
    const std::string BankName = "bank";
    static constexpr Addr BankBase = 0x10000;

    Register reg0, reg1, reg2;
    std::vector<RegisterBankLE> bank;

    VectorRegisterBankTest() :
        reg0("reg0", 0xd3d2d1d0), reg1("reg1", 0xe3e2e1e0),
        reg2("reg2", 0xf3f2f1f0)
    {
        bank.reserve(BankSize);
        for (int i = 0; i < BankSize; i++) {
            bank.emplace_back(
                BankName + std::to_string(i), BankBase + i * BankOffset);
        }
        bank[0].addRegister(reg0);
        bank[1].addRegister(reg1);
        bank[2].addRegister(reg2);
    }
};

TEST_F(VectorRegisterBankTest, CheckVectorRegisterBankSize)
{
    EXPECT_EQ(bank.size(), 3);
}

TEST_F(VectorRegisterBankTest, CheckVectorRegisterBankName)
{
    EXPECT_EQ(bank[0].name(), BankName + std::to_string(0));
    EXPECT_EQ(bank[1].name(), BankName + std::to_string(1));
    EXPECT_EQ(bank[2].name(), BankName + std::to_string(2));
}

TEST_F(VectorRegisterBankTest, GetVectorRegisterBankData)
{
    uint32_t val0, val1, val2;
    bank[0].read(BankBase, &val0, 4);
    EXPECT_EQ(val0, reg0.get());
    bank[1].read(BankBase + 1 * BankOffset, &val1, 4);
    EXPECT_EQ(val1, reg1.get());
    bank[2].read(BankBase + 2 * BankOffset, &val2, 4);
    EXPECT_EQ(val2, reg2.get());
}
