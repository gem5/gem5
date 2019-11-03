/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
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
 * Authors: Nathan Binkert
 */

#include "base/atomicio.hh"

#include <cerrno>
#include <cstdio>

ssize_t
atomic_read(int fd, void *s, size_t n)
{
    char *p = reinterpret_cast<char *>(s);
    size_t pos = 0;

    // Keep reading until we've gotten all of the data.
    while (n > pos) {
        ssize_t result = read(fd, p + pos, n - pos);

        // We didn't get any more data, so we should probably punt,
        // otherwise we'd just keep spinning
        if (result == 0)
            break;

        // If there was an error, try again on EINTR/EAGAIN, pass the
        // error up otherwise.
        if (result == -1) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return result;
        }

        pos += result;
    }

    return pos;
}

ssize_t
atomic_write(int fd, const void *s, size_t n)
{
    const char *p = reinterpret_cast<const char *>(s);
    size_t pos = 0;

    // Keep writing until we've written all of the data
    while (n > pos) {
        ssize_t result = write(fd, p + pos, n - pos);

        // We didn't manage to write anything this time, so we should
        // probably punt, otherwise we'd just keep spinning
        if (result == 0)
            break;

        // If there was an error, try again on EINTR/EAGAIN, pass the
        // error up otherwise.
        if (result == -1) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return result;
        }

        pos += result;
    }

    return pos;
}
