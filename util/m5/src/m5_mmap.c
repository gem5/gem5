/*
 * Copyright (c) 2011, 2017 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "m5_mmap.h"

void *m5_mem = NULL;

#ifndef M5OP_ADDR
#define M5OP_ADDR 0
#endif
uint64_t m5op_addr = M5OP_ADDR;

const char *m5_mmap_dev = "/dev/gem5/bridge";
const char *m5_mmap_dev_fallback = "/dev/mem";

void
map_m5_mem()
{
    int fd;

    if (m5_mem) {
        fprintf(stderr, "m5 mem already mapped.\n");
        exit(1);
    }

    if (m5op_addr == 0) {
        fprintf(stdout, "Warn: m5op_addr is set to 0x0\n");
    }

    fd = open(m5_mmap_dev, O_RDWR | O_SYNC);
    if (fd == -1) {
        fprintf(stderr, "Can't open %s: %s\n", m5_mmap_dev, strerror(errno));
        fprintf(stderr, "--> Make sure the gem5_bridge device driver has "
                        "been properly inserted into the kernel. Otherwise, "
                        "sudo access required to perform address-mode ops.\n");

        /* Fallback device */
        fd = open(m5_mmap_dev_fallback, O_RDWR | O_SYNC);
        if (fd == -1) {
            fprintf(stderr, "Can't open %s: %s\n",
                    m5_mmap_dev_fallback, strerror(errno));
            fprintf(stderr, "--> Make sure this utility has sudo access to "
                            "use fallback device.\n");
            exit(1);
        }
    }

    m5_mem = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (!m5_mem) {
        fprintf(stderr, "Can't map %s: %s\n", m5_mmap_dev, strerror(errno));
        exit(1);
    }
}

void
unmap_m5_mem()
{
    if (m5_mem) {
        munmap(m5_mem, 0x10000);
        m5_mem = NULL;
    }
}
