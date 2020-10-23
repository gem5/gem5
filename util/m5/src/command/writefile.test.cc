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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

#include "args.hh"
#include "command.hh"
#include "dispatch_table.hh"

uint64_t test_total_written;
std::string test_host_file_name;

std::vector<uint8_t> test_written_data;
uint64_t test_max_buf_size;

uint64_t
test_m5_write_file(void *buffer, uint64_t len, uint64_t offset,
                   const char *filename)
{
    if (test_max_buf_size && len > test_max_buf_size)
        len = test_max_buf_size;

    test_total_written += len;

    if (test_host_file_name == "")
        test_host_file_name = filename;
    else
        EXPECT_EQ(test_host_file_name, filename);

    if (offset == 0)
        test_written_data.clear();

    size_t required_size = offset + len;
    if (test_written_data.size() < required_size)
        test_written_data.resize(required_size);

    std::memcpy(test_written_data.data() + offset, buffer, len);

    return len;
}

DispatchTable dt = { .m5_write_file = &test_m5_write_file };

std::string cout_output;

bool
run(std::initializer_list<std::string> arg_args)
{
    test_total_written = 0;
    test_host_file_name = "";
    test_written_data.clear();

    Args args(arg_args);

    // Redirect cout into a stringstream.
    std::stringstream buffer;
    std::streambuf *orig = std::cout.rdbuf(buffer.rdbuf());

    bool res = Command::run(dt, args);

    // Capture the contents of the stringstream and restore cout.
    cout_output = buffer.str();
    std::cout.rdbuf(orig);

    return res;
}

class TempFile
{
  private:
    size_t _size;
    int fd;
    std::string _path;
    void *_buf;

  public:
    TempFile(size_t _size) : _size(_size)
    {
        // Generate a temporary filename.
        char *tmp_name = strdup("/tmp/writefile.test.XXXXXXXX");
        fd = mkstemp(tmp_name);
        _path = tmp_name;
        free(tmp_name);

        // Make the file the appropriate length.
        assert(!ftruncate(fd, _size));

        // mmap the file.
        _buf = mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        assert(_buf);

        // Fill it with an incrementing 32 bit integers.

        int chunk_size = sizeof(uint32_t);
        size_t num_chunks = _size / chunk_size;
        int leftovers = _size % chunk_size;

        uint32_t *buf32 = (uint32_t *)_buf;
        uint32_t val = 0;
        for (size_t i = 0; i < num_chunks; i++)
            *buf32++ = val++;
        if (leftovers)
            std::memcpy(buf32, &val, leftovers);

        // Make sure our new contents are out there.
        msync(_buf, _size, MS_SYNC | MS_INVALIDATE);
    };

    ~TempFile()
    {
        unlink(path().c_str());
        close(fd);
    }

    size_t size() const { return _size; }
    const std::string &path() const { return _path; }
    const void *buf() const { return _buf; }

    void
    verify()
    {
        verify(path());
    }

    void
    verify(const std::string &expected_path)
    {
        EXPECT_EQ(test_written_data.size(), size());
        EXPECT_EQ(memcmp(test_written_data.data(), buf(), size()), 0);
        EXPECT_EQ(test_host_file_name, expected_path);

        std::ostringstream os;
        os << "Opening \"" << path() << "\".";
        EXPECT_THAT(cout_output, ::testing::HasSubstr(os.str()));
        os.str("");
        os << "Wrote " << size() << " bytes.";
        EXPECT_THAT(cout_output, ::testing::HasSubstr(os.str()));
    }
};

TEST(Writefile, NoArguments)
{
    // Call with no arguments.
    EXPECT_FALSE(run({"writefile"}));
    EXPECT_EQ(test_total_written, 0);
}

TEST(Writefile, ThreeArguments)
{
    // Call with no arguments.
    EXPECT_FALSE(run({"writefile", "1", "2", "3"}));
    EXPECT_EQ(test_total_written, 0);
}

TEST(Writefile, SmallFile)
{
    // Write a small file.
    TempFile tmp(16);
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"writefile", tmp.path()}));
    tmp.verify();
}

TEST(Writefile, SmallFileHostName)
{
    // Write a small file with a different host file name.
    TempFile tmp(16);
    test_max_buf_size = 0;
    std::string host_path = "/different/host/path";
    EXPECT_TRUE(run({"writefile", tmp.path(), host_path}));
    tmp.verify(host_path);
}

TEST(Writefile, MultipleChunks)
{
    // Write a file which will need to be split into multiple whole chunks.
    TempFile tmp(256 * 1024 * 4);
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"writefile", tmp.path()}));
    tmp.verify();
}

TEST(Writefile, MultipleAndPartialChunks)
{
    // Write a file which will be split into some whole and one partial chunk.
    TempFile tmp(256 * 1024 * 2 + 256);
    test_max_buf_size = 0;
    EXPECT_TRUE(run({"writefile", tmp.path()}));
    tmp.verify();
}

TEST(Writefile, OddSizedChunks)
{
    // Write a file in chunks that aren't nicely aligned.
    TempFile tmp(256 * 1024);
    test_max_buf_size = 13;
    EXPECT_TRUE(run({"writefile", tmp.path()}));
    tmp.verify();
}

TEST(Writefile, CappedWriteSize)
{
    // Write a file, accepting less than the requested amount of data.
    TempFile tmp(256 * 1024 * 2 + 256);
    test_max_buf_size = 256;
    EXPECT_TRUE(run({"writefile", tmp.path()}));
    tmp.verify();
}

TEST(WritefileDeathTest, BadFile)
{
    EXPECT_EXIT(run({"writefile", "this is not a valid path#$#$://\\\\"}),
            ::testing::ExitedWithCode(2), "Error opening ");
}
