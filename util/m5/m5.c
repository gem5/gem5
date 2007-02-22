/*
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
 *
 * Authors: Nathan Binkert
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "m5op.h"

char *progname;

void
usage()
{
    printf("usage: m5 initparam\n"
           "       m5 sw99param\n"
           "       m5 exit [delay]\n"
           "       m5 resetstats [delay [period]]\n"
           "       m5 dumpstats [delay [period]]\n"
           "       m5 dumpresetstats [delay [period]]\n"
           "       m5 checkpoint [delay [period]]\n"
           "       m5 readfile\n"
           "\n"
           "All times in nanoseconds!\n");
    exit(1);
}

#define COMPARE(X) (strcmp(X, command) == 0)

int
main(int argc, char *argv[])
{
    char *command;
    uint64_t param;
    uint64_t arg1 = 0;
    uint64_t arg2 = 0;

    progname = argv[0];
    if (argc < 2)
        usage();

    command = argv[1];

    if (COMPARE("initparam")) {
        if (argc != 2)
            usage();

        printf("%ld", m5_initparam());
        return 0;
    }

    if (COMPARE("sw99param")) {
        if (argc != 2)
            usage();

        param = m5_initparam();
        // run-time, rampup-time, rampdown-time, warmup-time, connections
        printf("%d %d %d %d %d", (param >> 48) & 0xfff,
               (param >> 36) & 0xfff, (param >> 24) & 0xfff,
               (param >> 12) & 0xfff, (param >> 0) & 0xfff);

        return 0;
    }

    if (COMPARE("exit")) {
        switch (argc) {
          case 3:
            arg1 = strtoul(argv[2], NULL, 0);
          case 2:
            m5_exit(arg1);
            return 0;

          default:
            usage();
        }
    }

    if (COMPARE("resetstats")) {
        switch (argc) {
          case 4:
            arg2 = strtoul(argv[3], NULL, 0);
          case 3:
            arg1 = strtoul(argv[2], NULL, 0);
          case 2:
            m5_reset_stats(arg1, arg2);
            return 0;

          default:
            usage();
        }
    }

    if (COMPARE("dumpstats")) {
        switch (argc) {
          case 4:
            arg2 = strtoul(argv[3], NULL, 0);
          case 3:
            arg1 = strtoul(argv[2], NULL, 0);
          case 2:
            m5_dump_stats(arg1, arg2);
            return 0;

          default:
            usage();
        }
    }

    if (COMPARE("dumpresetstats")) {
        switch (argc) {
          case 4:
            arg2 = strtoul(argv[3], NULL, 0);
          case 3:
            arg1 = strtoul(argv[2], NULL, 0);
          case 2:
            m5_dumpreset_stats(arg1, arg2);
            return 0;

          default:
            usage();
        }
    }

    if (COMPARE("readfile")) {
            char buf[256*1024];
            int offset = 0;
            int len;

            if (argc != 2)
                    usage();

            while ((len = m5_readfile(buf, sizeof(buf), offset)) > 0) {
                    write(STDOUT_FILENO, buf, len);
                    offset += len;
            }

            return 0;
    }

    if (COMPARE("checkpoint")) {
        switch (argc) {
          case 4:
            arg2 = strtoul(argv[3], NULL, 0);
          case 3:
            arg1 = strtoul(argv[2], NULL, 0);
          case 2:
            m5_checkpoint(arg1, arg2);
            return 0;

          default:
            usage();
        }

        return 0;
    }

    if (COMPARE("loadsymbol")) {
        m5_loadsymbol(arg1);
        return 0;
    }
    if (COMPARE("readfile")) {
            char buf[256*1024];
            int offset = 0;
            int len;

            if (argc != 2)
                    usage();

            while ((len = m5_readfile(buf, sizeof(buf), offset)) > 0) {
                    write(STDOUT_FILENO, buf, len);
                    offset += len;
            }

            return 0;
    }

    usage();
}
