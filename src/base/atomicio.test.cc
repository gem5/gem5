/*
 * Copyright (c) 2019 The Regents of the University of California
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
#include <unistd.h>

#include <string>

#include "base/atomicio.hh"

/*
 * This will test reading from a file with a buffer capable of storing the
 * entirity of the file.
 */
TEST(AtomicioTest, AtomicReadBigBuffer)
{
    FILE* file;
    file = tmpfile();

    std::string file_contents = "This is just some test data to ensure that we"
                                " can read correctly from a file.";

    fputs(file_contents.c_str(), file);
    fflush(file);
    rewind(file);

    char s[1000];

    ssize_t size = atomic_read(fileno(file), s, 1000);
    fclose(file);

    EXPECT_EQ(file_contents.size(), size);
    for (unsigned int i = 0; i < size; i++) {
        EXPECT_EQ(file_contents[i], s[i]);
    }
}

/*
 * This will test reading from a file using a buffer smaller than the file
 * contents. The buffer should be filled and the remainder left unread.
 */
TEST(AtomicioTest, AtomicReadSmallBuffer)
{
    FILE* file;
    file = tmpfile();

    std::string file_contents = "This is just some test data to ensure that we"
                                " can read correctly from a file.";

    fputs(file_contents.c_str(), file);
    fflush(file);
    rewind(file);

    char s[10];

    ssize_t size = atomic_read(fileno(file), s, 10);
    fclose(file);

    EXPECT_EQ(10, size);
    for (unsigned int i = 0; i < size; i++) {
        EXPECT_EQ(file_contents[i], s[i]);
    }
}

/*
 * This tests writing a string to a file via the atomic_write function.
 */
TEST(AtomicioTest, AtomicWrite)
{
    FILE* file;
    file = tmpfile();

    std::string file_contents = "This is just some test data to ensure that we"
                                " can write correctly to a file.";

    ssize_t size = atomic_write(fileno(file),
                                file_contents.c_str(),
                                file_contents.size());
    fflush(file);
    rewind(file);

    EXPECT_EQ(file_contents.size(), size);

    int c;
    for (unsigned int i = 0; (c = fgetc(file)) != EOF; i++) {
        EXPECT_EQ(file_contents[i], (unsigned char)c);
    }

    fclose(file);
}
