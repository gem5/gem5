/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Nathan Binkert
 */

#ifdef linux
#define _GNU_SOURCE
#include <sched.h>
#endif

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

#include "m5op.h"

void *m5_mem = NULL;

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

int
read_file(int dest_fid)
{
    char buf[256*1024];
    int offset = 0;
    int len;

    // Touch all buffer pages to ensure they are mapped in the
    // page table. This is required in the case of X86_FS, where
    // Linux does demand paging.
    memset(buf, 0, sizeof(buf));

    while ((len = m5_readfile(buf, sizeof(buf), offset)) > 0) {
        write(dest_fid, buf, len);
        offset += len;
    }
}

int
write_file(const char *filename)
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
        bytes += m5_writefile(buf, len, offset, filename);
        offset += len;
    }
    fprintf(stderr, "written %d bytes\n", bytes);

    close(src_fid);
}

void
do_exit(int argc, char *argv[])
{
    if (argc > 1)
        usage();

    uint64_t ints[1];
    parse_int_args(argc, argv, ints, 1);
    m5_exit(ints[0]);
}

void
do_fail(int argc, char *argv[])
{
    if (argc < 1 || argc > 2)
        usage();

    uint64_t ints[2] = {0,0};
    parse_int_args(argc, argv, ints, argc);
    m5_fail(ints[1], ints[0]);
}

void
do_reset_stats(int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    m5_reset_stats(ints[0], ints[1]);
}

void
do_dump_stats(int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    m5_dump_stats(ints[0], ints[1]);
}

void
do_dump_reset_stats(int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    m5_dumpreset_stats(ints[0], ints[1]);
}

void
do_read_file(int argc, char *argv[])
{
    if (argc > 0)
        usage();

    read_file(STDOUT_FILENO);
}

void
do_write_file(int argc, char *argv[])
{
    if (argc != 1)
        usage();

    const char *filename = argv[0];

    write_file(filename);
}

void
do_exec_file(int argc, char *argv[])
{
    if (argc > 0)
        usage();

    const char *destname = "/tmp/execfile";

    int fid = open(destname, O_WRONLY, 0777);
    int len = read_file(fid);
    close(fid);
    if (len > 0) {
        execl(destname, "execfile", NULL);
        err(1, "execl failed!");
    }
}

void
do_checkpoint(int argc, char *argv[])
{
    uint64_t ints[2];
    parse_int_args(argc, argv, ints, 2);
    m5_checkpoint(ints[0], ints[1]);
}

void
do_load_symbol(int argc, char *argv[])
{
    if (argc != 2)
        usage();

    uint64_t addr = strtoul(argv[0], NULL, 0);
    char *symbol = argv[1];
    m5_loadsymbol(addr, symbol);
}

void
do_initparam(int argc, char *argv[])
{
    if (argc != 0)
        usage();

    uint64_t val = m5_initparam();
    printf("%"PRIu64, val);
}

void
do_sw99param(int argc, char *argv[])
{
    if (argc != 0)
        usage();

    uint64_t param = m5_initparam();

    // run-time, rampup-time, rampdown-time, warmup-time, connections
    printf("%"PRId64" %"PRId64" %"PRId64" %"PRId64" %"PRId64,
           (param >> 48) & 0xfff,
           (param >> 36) & 0xfff, (param >> 24) & 0xfff,
           (param >> 12) & 0xfff, (param >> 0) & 0xfff);
}

#ifdef linux
void
do_pin(int argc, char *argv[])
{
    if (argc < 2)
        usage();

    cpu_set_t mask;  
    CPU_ZERO(&mask);

    const char *sep = ",";
    char *target = strtok(argv[0], sep);
    while (target) {
        CPU_SET(atoi(target), &mask);
        target = strtok(NULL, sep);
    }            

    if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) < 0)
        err(1, "setaffinity");

    execvp(argv[1], &argv[1]);
    err(1, "execvp failed!");
}
#endif

struct MainFunc
{
    char *name;
    void (*func)(int argc, char *argv[]);
    char *usage;
};

struct MainFunc mainfuncs[] = {
    { "exit",           do_exit,             "[delay]" },
    { "fail",           do_fail,             "<code> [delay]" },
    { "resetstats",     do_reset_stats,      "[delay [period]]" },
    { "dumpstats",      do_dump_stats,       "[delay [period]]" },
    { "dumpresetstats", do_dump_reset_stats, "[delay [period]]" },
    { "readfile",       do_read_file,        "" },
    { "writefile",      do_write_file,       "<filename>" },
    { "execfile",       do_exec_file,        "" },
    { "checkpoint",     do_checkpoint,       "[delay [period]]" },
    { "loadsymbol",     do_load_symbol,      "<address> <symbol>" },
    { "initparam",      do_initparam,        "" },
    { "sw99param",      do_sw99param,        "" },
#ifdef linux
    { "pin",            do_pin,              "<cpu> <program> [args ...]" }
#endif
};
int numfuncs = sizeof(mainfuncs) / sizeof(mainfuncs[0]);

void
usage()
{
    int i;

    for (i = 0; i < numfuncs; ++i) {
        char *header = i ? "" : "usage:";
        fprintf(stderr, "%-6s %s %s %s\n",
                header, progname, mainfuncs[i].name, mainfuncs[i].usage);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "All times in nanoseconds!\n");

    exit(1);
}

static void
map_m5_mem()
{
#ifdef M5OP_ADDR
    int fd;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("Can't open /dev/mem");
        exit(1);
    }

    m5_mem = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, M5OP_ADDR);
    if (!m5_mem) {
        perror("Can't mmap /dev/mem");
        exit(1);
    }
#endif
}

int
main(int argc, char *argv[])
{
    progname = argv[0];
    if (argc < 2)
        usage(1);

    map_m5_mem();

    command = argv[1];

    argv += 2;
    argc -= 2;

    int i;
    for (i = 0; i < numfuncs; ++i) {
        if (strcmp(command, mainfuncs[i].name) != 0)
            continue;

        mainfuncs[i].func(argc, argv);
        exit(0);
    }

    usage(1);
}
