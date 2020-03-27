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

#include <err.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <gem5/asm/generic/m5ops.h>
#include <gem5/m5ops.h>
#include "call_type.h"
#include "dispatch_table.h"
#include "m5_mmap.h"

char *progname;
char *command = "unspecified";
void usage();

void
parse_int_args(int argc, char *argv[], uint64_t ints[], int len)
{
    if (argc > len)
        usage();

// On 32 bit platforms we need to use strtoull to do the conversion
#ifdef __LP64__
#define strto64 strtoul
#else
#define strto64 strtoull
#endif
    int i;
    for (i = 0; i < len; ++i)
        ints[i] = (i < argc) ? strto64(argv[i], NULL, 0) : 0;

#undef strto64
}

void
pack_str_into_regs(const char *str, uint64_t regs[], int num_regs)
{
    const size_t RegSize = sizeof(regs[0]);
    const size_t MaxLen = num_regs * RegSize;

    size_t len = strlen(str);

    if (len > MaxLen)
        usage();

    memset(regs, 0, MaxLen);

    while (len) {
        for (int offset = 0; offset < RegSize && len; offset++, len--) {
            int shift = offset * 8;
            *regs |= (uint64_t)(uint8_t)*str++ << shift;
        }
        regs++;
    }
}

int
read_file(DispatchTable *dt, int dest_fid)
{
    uint8_t buf[256*1024];
    int offset = 0;
    int len, ret;

    // Touch all buffer pages to ensure they are mapped in the
    // page table. This is required in the case of X86_FS, where
    // Linux does demand paging.
    memset(buf, 0, sizeof(buf));

    while ((len = (*dt->m5_read_file)(buf, sizeof(buf), offset)) > 0) {
        uint8_t *base = buf;
        offset += len;
        do {
            ret = write(dest_fid, base, len);
            if (ret < 0) {
                perror("Failed to write file");
                exit(2);
            } else if (ret == 0) {
                fprintf(stderr, "Failed to write file: "
                        "Unhandled short write\n");
                exit(2);
            }

            base += ret;
            len -= ret;
        } while (len);
    }

    return offset;
}

void
write_file(DispatchTable *dt, const char *filename, const char *host_filename)
{
    fprintf(stderr, "opening %s\n", filename);
    int src_fid = open(filename, O_RDONLY);

    if (src_fid < 0) {
        fprintf(stderr, "error opening %s\n", filename);
        return;
    }

    char buf[256*1024];
    int offset = 0;
    int len;
    int bytes = 0;

    memset(buf, 0, sizeof(buf));

    while ((len = read(src_fid, buf, sizeof(buf))) > 0) {
        bytes += (*dt->m5_write_file)(buf, len, offset, host_filename);
        offset += len;
    }
    fprintf(stderr, "written %d bytes\n", bytes);

    close(src_fid);
}

void
do_exit(DispatchTable *dt, int argc, char *argv[])
{
    if (argc > 1)
        usage();

    uint64_t ints[1];
    parse_int_args(argc, argv, ints, 1);
    (*dt->m5_exit)(ints[0]);
}

void
do_fail(DispatchTable *dt, int argc, char *argv[])
{
    if (argc < 1 || argc > 2)
        usage();

    uint64_t ints[2] = {0,0};
    parse_int_args(argc, argv, ints, argc);
    (*dt->m5_fail)(ints[1], ints[0]);
}

void
do_reset_stats(DispatchTable *dt, int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    (*dt->m5_reset_stats)(ints[0], ints[1]);
}

void
do_dump_stats(DispatchTable *dt, int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    (*dt->m5_dump_stats)(ints[0], ints[1]);
}

void
do_dump_reset_stats(DispatchTable *dt, int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    (*dt->m5_dump_reset_stats)(ints[0], ints[1]);
}

void
do_read_file(DispatchTable *dt, int argc, char *argv[])
{
    if (argc > 0)
        usage();

    read_file(dt, STDOUT_FILENO);
}

void
do_write_file(DispatchTable *dt, int argc, char *argv[])
{
    if (argc != 1 && argc != 2)
        usage();

    const char *filename = argv[0];
    const char *host_filename = (argc == 2) ? argv[1] : argv[0];

    write_file(dt, filename, host_filename);
}

void
do_checkpoint(DispatchTable *dt, int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    (*dt->m5_checkpoint)(ints[0], ints[1]);
}

void
do_addsymbol(DispatchTable *dt, int argc, char *argv[])
{
    if (argc != 2)
        usage();

    uint64_t addr = strtoul(argv[0], NULL, 0);
    char *symbol = argv[1];
    (*dt->m5_add_symbol)(addr, symbol);
}


void
do_loadsymbol(DispatchTable *dt, int argc, char *argv[])
{
    if (argc > 0)
        usage();

    (*dt->m5_load_symbol)();
}

void
do_initparam(DispatchTable *dt, int argc, char *argv[])
{
    if (argc > 1)
        usage();

    uint64_t key_str[2];
    pack_str_into_regs(argc == 0 ? "" : argv[0], key_str, 2);
    uint64_t val = (*dt->m5_init_param)(key_str[0], key_str[1]);
    printf("%"PRIu64, val);
}

struct MainFunc
{
    char *name;
    void (*func)(DispatchTable *dt, int argc, char *argv[]);
    char *usage;
};

struct MainFunc mainfuncs[] = {
    { "addsymbol",      do_addsymbol,        "<address> <symbol> // Adds a "
                                             "symbol with address \"address\" "
                                             "to gem5's symbol table" },
    { "checkpoint",     do_checkpoint,       "[delay [period]] // After "
                                             "delay (default 0) take a "
                                             "checkpoint, and then optionally "
                                             "every period after" },
    { "dumpresetstats", do_dump_reset_stats, "[delay [period]] // After "
                                             "delay (default 0) dump and "
                                             "reset the stats, and then "
                                             "optionally every period after" },
    { "dumpstats",      do_dump_stats,       "[delay [period]] // After "
                                             "delay (default 0) dump the "
                                             "stats, and then optionally "
                                             "every period after" },
    { "exit",           do_exit,             "[delay] // Exit after delay, "
                                             "or immediately" },
    { "fail",           do_fail,             "<code> [delay] // Exit with "
                                             "failure code code after delay, "
                                             "or immediately" },
    { "initparam",      do_initparam,        "[key] // optional key may be at "
                                             "most 16 characters long" },
    { "loadsymbol",     do_loadsymbol,       "load a preselected symbol file "
                                             "into gem5's symbol table" },
    { "readfile",       do_read_file,        "read a preselected file from "
                                             "the host and write it to "
                                             "stdout" },
    { "resetstats",     do_reset_stats,      "[delay [period]] // After "
                                             "delay (default 0) reset the "
                                             "stats, and then optionally "
                                             "every period after" },
    { "writefile",      do_write_file,       "<filename> [host filename] // "
                                             "Write a file to the host, "
                                             "optionally with a different "
                                             "name" },
};
int numfuncs = sizeof(mainfuncs) / sizeof(mainfuncs[0]);

void
usage()
{
    fprintf(stderr, "Usage: %s [call type] <command> [arguments]\n", progname);
    fprintf(stderr, "\n");
    fprintf(stderr, "Call types:\n");
#   if ENABLE_CT_addr
    fprintf(stderr, "    --addr%s\n", DEFAULT_CT_addr ? " (default)" : "");
    fprintf(stderr, "        Use the address based invocation method.\n");
#   if defined(M5OP_ADDR)
    fprintf(stderr, "        The address is %#"PRIx64".\n",
            (uint64_t)M5OP_ADDR);
#   endif
#   endif
#   if ENABLE_CT_inst
    fprintf(stderr, "    --inst%s\n", DEFAULT_CT_inst ? " (default)" : "");
    fprintf(stderr, "        Use the instruction based invocation method.\n");
#   endif
#   if ENABLE_CT_semi
    fprintf(stderr, "    --semi%s\n", DEFAULT_CT_semi ? " (default)" : "");
    fprintf(stderr, "        Use the semi-hosting based invocation method.\n");
#   endif
    fprintf(stderr, "\n");
    fprintf(stderr, "Commands:\n");
    for (int i = 0; i < numfuncs; ++i)
        fprintf(stderr, "    %s %s\n", mainfuncs[i].name, mainfuncs[i].usage);
    fprintf(stderr, "\n");
    fprintf(stderr, "All times in nanoseconds!\n");

    exit(1);
}

int
main(int argc, char *argv[])
{
    progname = argv[0];

    argv++;
    argc--;

    DispatchTable *dt = NULL;

#   if ENABLE_CT_inst
    if (!dt && inst_call_type_detect(&argc, &argv)) {
        dt = inst_call_type_init();
    }
#   endif
#   if ENABLE_CT_addr
    if (!dt && addr_call_type_detect(&argc, &argv)) {
        dt = addr_call_type_init();
    }
#   endif
#   if ENABLE_CT_semi
    if (!dt && semi_call_type_detect(&argc, &argv)) {
        dt = semi_call_type_init();
    }
#   endif
    if (!dt)
        dt = default_call_type_init();

    if (!argc)
        usage(1);

    command = argv[0];

    argv++;
    argc--;

    int i;
    for (i = 0; i < numfuncs; ++i) {
        if (strcmp(command, mainfuncs[i].name) != 0)
            continue;

        mainfuncs[i].func(dt, argc, argv);
        exit(0);
    }

    usage(1);
}
