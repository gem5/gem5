/*
 * Copyright (c) 2019 The Regents of the University of California
 * All rights reserved
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
#include <sys/types.h>
#include <unistd.h>

#include <cstdlib>

#include "base/loader/image_file_data.hh"
#include "base/loader/small_image_file.test.hh"

using namespace gem5;
using namespace loader;

TEST(ImageFileDataTest, SimpleImage)
{
    /*
     * Create a temporary file from our data blob.
     */
    char filename[] = "image-XXXXXX";
    int fd = mkstemp(filename);
    ASSERT_NE(-1, fd);
    ssize_t size = write(fd, image_file, sizeof(image_file));

    /*
     * In this basic test, the image file is simply loaded to the
     * ImageFileData object (i.e., no decompression).
     */
    ImageFileData idf(filename);

    EXPECT_EQ(idf.filename(), filename);
    EXPECT_EQ(size, idf.len());

    for (size_t i = 0; i < sizeof(image_file); i++) {
        EXPECT_EQ(image_file[i], idf.data()[i]);
    }

    /*
     * Close and delete the temporary file.
     */
    close(fd);
    unlink(filename);
}


TEST(ImageFileDataTest, GZipImage)
{
    /*
     * Create temporary files from our data blobs.
     */
    char filename_gz[] = "image-XXXXXX";
    int fd_gz = mkstemp(filename_gz);
    ASSERT_NE(-1, fd_gz);
    ssize_t size_gz = write(fd_gz, image_file_gzipped,
                            sizeof(image_file_gzipped));

    char filename[] = "image-XXXXXX";
    int fd = mkstemp(filename);
    ASSERT_NE(-1, fd);
    ssize_t size = write(fd, image_file, sizeof(image_file));

    /*
     * ImageFileData decompresses a gzipped file. image_file_gzipped is just
     * image_file gzipped. Therefore ifd_gz.len() should equal ifd.len() and
     * ifd.data() should equal ifd_gz.data().
     */
    ImageFileData ifd_gz(filename_gz);
    ImageFileData ifd(filename);

    EXPECT_EQ(ifd.len(), ifd_gz.len());
    EXPECT_EQ(size, ifd.len());
    EXPECT_NE(size_gz, ifd_gz.len());

    for (size_t index = 0; index < ifd.len(); index++) {
        EXPECT_EQ(ifd.data()[index], ifd_gz.data()[index]);
    }

    /*
     * Close and delete the temporary files.
     */
    close(fd);
    unlink(filename);
    close(fd_gz);
    unlink(filename_gz);
}
