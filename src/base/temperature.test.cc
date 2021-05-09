/*
 * Copyright (c) 2021 Arm Limited
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

#include <gtest/gtest.h>

#include <sstream>

#include "base/temperature.hh"

using namespace gem5;

TEST(TemperatureTest, Constructor)
{
    Temperature temp;
    EXPECT_EQ(temp, Temperature(0.0));
}

TEST(TemperatureTest, Conversion)
{
    Temperature freezing(273.15);

    EXPECT_EQ(Temperature::fromKelvin(42.0), Temperature(42.0));
    EXPECT_EQ(Temperature::fromCelsius(0.0), freezing);
    EXPECT_EQ(Temperature::fromFahrenheit(32.0), freezing);

    EXPECT_EQ(freezing.toKelvin(), 273.15);
    EXPECT_EQ(freezing.toCelsius(), 0.0);
    EXPECT_EQ(Temperature(0).toFahrenheit(), -459.67);
}

TEST(TemperatureTest, Comparison)
{
    EXPECT_TRUE(Temperature(2.0) < Temperature(3.0));
    EXPECT_FALSE(Temperature(2.0) < Temperature(2.0));
    EXPECT_FALSE(Temperature(2.0) < Temperature(1.0));

    EXPECT_FALSE(Temperature(2.0) > Temperature(3.0));
    EXPECT_FALSE(Temperature(2.0) > Temperature(2.0));
    EXPECT_TRUE(Temperature(2.0) > Temperature(1.0));

    EXPECT_TRUE(Temperature(2.0) <= Temperature(3.0));
    EXPECT_TRUE(Temperature(2.0) <= Temperature(2.0));
    EXPECT_FALSE(Temperature(2.0) <= Temperature(1.0));

    EXPECT_FALSE(Temperature(2.0) >= Temperature(3.0));
    EXPECT_TRUE(Temperature(2.0) >= Temperature(2.0));
    EXPECT_TRUE(Temperature(2.0) >= Temperature(1.0));

    EXPECT_TRUE(Temperature(2.0) == Temperature(2.0));
    EXPECT_FALSE(Temperature(2.0) == Temperature(3.0));

    EXPECT_FALSE(Temperature(2.0) != Temperature(2.0));
    EXPECT_TRUE(Temperature(2.0) != Temperature(3.0));
}

TEST(TemperatureTest, BinaryOperators)
{
    EXPECT_EQ(Temperature(2.0) + Temperature(1.0), Temperature(3.0));
    EXPECT_EQ(Temperature(2.0) - Temperature(1.0), Temperature(1.0));

    EXPECT_EQ(Temperature(8.0) * 2.0, Temperature(16.0));
    EXPECT_EQ(2.0 * Temperature(8.0), Temperature(16.0));
    EXPECT_EQ(Temperature(8.0) / 2.0, Temperature(4.0));
}

TEST(TemperatureTest, AssignmentOperators)
{
    Temperature temp1(0);
    temp1 += Temperature(1.0);
    EXPECT_EQ(temp1, Temperature(1.0));

    Temperature temp2(1.0);
    temp2 -= Temperature(1.0);
    EXPECT_EQ(temp2, Temperature(0.0));

    Temperature temp3(32.0);
    temp3 *= 2.0;
    EXPECT_EQ(temp3, Temperature(64.0));

    Temperature temp4(32.0);
    temp4 /= 2.0;
    EXPECT_EQ(temp4, Temperature(16.0));
}

TEST(TemperatureTest, OutStream)
{
    Temperature temp(42);
    std::ostringstream ss;
    ss << "T: " << temp << std::endl;
    EXPECT_EQ("T: 42K\n", ss.str());
}
