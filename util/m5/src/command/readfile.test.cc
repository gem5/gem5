/*
 * Copyright 2020 Google Inc.
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

#include "args.hh"
#include "command.hh"
#include "dispatch_table.hh"

uint64_t test_read_file_size;
uint64_t test_max_buf_size;

uint64_t test_total_read;

uint64_t
test_m5_read_file(void *buffer, uint64_t len, uint64_t offset)
{
    // The "file" we're reading is just a series of incrementing 32 bit
    // integers.

    // If the buffer is entirely past the end of our "file", return 0.
    if (offset >= test_read_file_size)
        return 0;

    // If the buffer extends beyond our "file" truncate it.
    if (offset + len > test_read_file_size)
        len = test_read_file_size - offset;

    // If more data was requested than we want to send at once, truncate len.
    if (test_max_buf_size && len > test_max_buf_size)
        len = test_max_buf_size;

    int chunk_size = sizeof(uint32_t);

    // How much of len is still unaccounted for.
    uint64_t remaining = len;

    // How much overlaps with the preceeding chunk?
    int at_start = chunk_size - (offset % chunk_size);
    // If we don't even cover the entire previous chunk...
    if (at_start > len)
        at_start = len;
    remaining -= at_start;

    // How much overlaps with the following chunk?
    int at_end = remaining % chunk_size;
    remaining -= at_end;

    // The number of chunks are the number we cover fully, plus one for each
    // end were we partially overlap.
    uint64_t num_chunks = remaining / chunk_size +
        (at_start ? 1 : 0) + (at_end ? 1 : 0);

    // Build this part of the file.
    uint32_t *chunks = new uint32_t [num_chunks];

    uint32_t chunk_idx = offset / chunk_size;
    for (uint64_t i = 0; i < num_chunks; i++)
        chunks[i] = chunk_idx++;

    // Copy out to the requested buffer.
    memcpy(buffer, ((uint8_t *)chunks) + (chunk_size - at_start), len);

    // Clean up.
    delete [] chunks;

    test_total_read += len;
    return len;
}

DispatchTable dt = { .m5_read_file = &test_m5_read_file };

std::string cout_output;

bool
run(std::initializer_list<std::string> arg_args, bool bad_file=false)
{
    test_total_read = 0;

    Args args(arg_args);

    // Redirect cout into a stringstream.
    std::stringstream buffer;
    std::streambuf *orig = std::cout.rdbuf(buffer.rdbuf());

    // Simulate a problem writing to cout.
    if (bad_file)
        std::cout.setstate(std::cout.badbit);

    bool res = Command::run(dt, args);

    if (bad_file)
        std::cout.clear();

    // Capture the contents of the stringstream and restore cout.
    cout_output = buffer.str();
    std::cout.rdbuf(orig);

    return res;
}

void
test_verify_data()
{
    EXPECT_EQ(test_total_read, test_read_file_size);
    EXPECT_EQ(cout_output.size(), test_read_file_size);

    auto *data32 = (const uint32_t *)cout_output.data();
    uint64_t len = cout_output.size();

    int chunk_size = sizeof(uint32_t);

    uint64_t num_chunks = len / chunk_size;
    int leftovers = len % chunk_size;

    uint32_t chunk_idx;
    for (chunk_idx = 0; chunk_idx < num_chunks; chunk_idx++)
        EXPECT_EQ(*data32++, chunk_idx);

    if (leftovers)
        EXPECT_EQ(memcmp(&chunk_idx, data32, leftovers), 0);
}

TEST(Readfile, OneArgument)
{
    // Call with an argument.
    EXPECT_FALSE(run({"readfile", "foo"}));
    EXPECT_EQ(test_total_read, 0);
}

TEST(Readfile, SmallFile)
{
    // Read a small "file".
    test_read_file_size = 16;
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"readfile"}));
    test_verify_data();
}

TEST(Readfile, MultipleChunks)
{
    // Read a "file" which will need to be split into multiple whole chunks.
    test_read_file_size = 256 * 1024 * 4;
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"readfile"}));
    test_verify_data();
}

TEST(Readfile, MultipleAndPartialChunks)
{
    // Read a "file" which will be split into some whole and one partial chunk.
    test_read_file_size = 256 * 1024 * 2 + 256;
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"readfile"}));
    test_verify_data();
}

TEST(Readfile, OddSizedChunks)
{
    // Read a "file" in chunks that aren't nicely aligned.
    test_read_file_size = 256 * 1024;
    test_max_buf_size = 13;
    EXPECT_TRUE(run({"readfile"}));
    test_verify_data();
}

TEST(Readfile, CappedReadSize)
{
    // Read a "file", returning less than the requested amount of data.
    test_read_file_size = 256 * 1024 * 2 + 256;
    test_max_buf_size = 256;
    EXPECT_TRUE(run({"readfile"}));
    test_verify_data();
}

TEST(ReadfileDeathTest, BadFile)
{
    test_read_file_size = 16;
    test_max_buf_size = 0;
    EXPECT_EXIT(run({"readfile"}, true), ::testing::ExitedWithCode(2),
            "Failed to write file");
}
