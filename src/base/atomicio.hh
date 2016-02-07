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

#ifndef __BASE_ATOMICIO_HH__
#define __BASE_ATOMICIO_HH__

#include <unistd.h>

// These functions keep reading/writing, if possible, until all data
// has been transferred.  Basically, try again when there's no error,
// but there is data left also retry on EINTR.
// This function blocks until it is done.

ssize_t atomic_read(int fd, void *s, size_t n);
ssize_t atomic_write(int fd, const void *s, size_t n);

/**
 * Statically allocate a string and write it to a file descriptor.
 *
 * @warning The return value will from atomic_write will be ignored
 * which means that errors will be ignored. This is normally fine as
 * this macro is intended to be used in fatal signal handlers where
 * error handling might not be feasible.
 */
#define STATIC_MSG(fd, m)                                       \
    do {                                                        \
        static const char msg[] = m;                            \
        atomic_write(fd, msg, sizeof(msg) - 1);                 \
    } while (0)

/**
 * Statically allocate a string and write it to STDERR.
 *
 * @warning The return value will from atomic_write will be ignored
 * which means that errors will be ignored. This is normally fine as
 * this macro is intended to be used in fatal signal handlers where
 * error handling might not be feasible.
 */
#define STATIC_ERR(m) STATIC_MSG(STDERR_FILENO, m)

#endif // __BASE_ATOMICIO_HH__
