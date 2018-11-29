/*
 * Copyright (c) 2015 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include <gtest/gtest.h>

#include "base/pixel.hh"

static Pixel pixel_red(0xff, 0x00, 0x00);
static Pixel pixel_green(0x00, 0xff, 0x00);
static Pixel pixel_blue(0x00, 0x00, 0xff);

TEST(FBTest, PixelConversionRGBA8888)
{
    EXPECT_EQ(PixelConverter::rgba8888_le.fromPixel(pixel_red),
              0x000000ffU);
    EXPECT_EQ(PixelConverter::rgba8888_le.fromPixel(pixel_green),
              0x0000ff00U);
    EXPECT_EQ(PixelConverter::rgba8888_le.fromPixel(pixel_blue),
              0x00ff0000U);

    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(0x000000ffU),
              pixel_red);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(0x0000ff00U),
              pixel_green);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(0x00ff0000U),
              pixel_blue);
}

TEST(FBTest, PixelConversionRGB565)
{
    EXPECT_EQ(PixelConverter::rgb565_le.fromPixel(pixel_red),   0x001fU);
    EXPECT_EQ(PixelConverter::rgb565_le.fromPixel(pixel_green), 0x07e0U);
    EXPECT_EQ(PixelConverter::rgb565_le.fromPixel(pixel_blue),  0xf800U);

    EXPECT_EQ(PixelConverter::rgb565_le.toPixel(0x001fU), pixel_red);
    EXPECT_EQ(PixelConverter::rgb565_le.toPixel(0x07e0U), pixel_green);
    EXPECT_EQ(PixelConverter::rgb565_le.toPixel(0xf800U), pixel_blue);
}

TEST(FBTest, PixelToMemRGBA8888LE)
{
    uint8_t data[] = { 0xde, 0xad, 0xbe, 0xef };
    PixelConverter::rgba8888_le.fromPixel(data, pixel_red);
    EXPECT_EQ(data[0], 0xff);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[3], 0x00);
    EXPECT_EQ(data[3], 0x00);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(data), pixel_red);

    PixelConverter::rgba8888_le.fromPixel(data, pixel_green);
    EXPECT_EQ(data[0], 0x00);
    EXPECT_EQ(data[1], 0xff);
    EXPECT_EQ(data[3], 0x00);
    EXPECT_EQ(data[3], 0x00);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(data), pixel_green);

    PixelConverter::rgba8888_le.fromPixel(data, pixel_blue);
    EXPECT_EQ(data[0], 0x00);
    EXPECT_EQ(data[1], 0x00);
    EXPECT_EQ(data[2], 0xff);
    EXPECT_EQ(data[3], 0x00);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(data), pixel_blue);
}

TEST(FBTest, MemToPixelRGBA8888LE)
{
    uint8_t red[] = { 0xff, 0x00, 0x00, 0x00 };
    uint8_t green[] = { 0x00, 0xff, 0x00, 0x00 };
    uint8_t blue[] = { 0x00, 0x00, 0xff, 0x00 };

    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(red), pixel_red);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(green), pixel_green);
    EXPECT_EQ(PixelConverter::rgba8888_le.toPixel(blue), pixel_blue);
}

TEST(FBTest, MemToPixelRGBA8888BE)
{
    uint8_t red[] = { 0x00, 0x00, 0x00, 0xff };
    uint8_t green[] = { 0x00, 0x00, 0xff, 0x00 };
    uint8_t blue[] = { 0x00, 0xff, 0x00, 0x00 };

    EXPECT_EQ(PixelConverter::rgba8888_be.toPixel(red), pixel_red);
    EXPECT_EQ(PixelConverter::rgba8888_be.toPixel(green), pixel_green);
    EXPECT_EQ(PixelConverter::rgba8888_be.toPixel(blue), pixel_blue);
}
