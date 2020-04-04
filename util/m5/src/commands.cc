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
 */

#include <err.h>
#include <fcntl.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "args.hh"
#include "commands.hh"
#include "usage.hh"

static int
read_file(const DispatchTable &dt, int dest_fid)
{
    uint8_t buf[256*1024];
    int offset = 0;
    int len, ret;

    // Touch all buffer pages to ensure they are mapped in the
    // page table. This is required in the case of X86_FS, where
    // Linux does demand paging.
    memset(buf, 0, sizeof(buf));

    while ((len = (*dt.m5_read_file)(buf, sizeof(buf), offset)) > 0) {
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

static void
write_file(const DispatchTable &dt, const char *filename,
           const char *host_filename)
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
        bytes += (*dt.m5_write_file)(buf, len, offset, host_filename);
        offset += len;
    }
    fprintf(stderr, "written %d bytes\n", bytes);

    close(src_fid);
}

static void
do_exit(const DispatchTable &dt, Args &args)
{
    if (args.size() > 1)
        usage();

    uint64_t ints[1];
    if (!parse_int_args(args, ints, 1))
        usage();
    (*dt.m5_exit)(ints[0]);
}

static void
do_fail(const DispatchTable &dt, Args &args)
{
    if (args.size() < 1 || args.size() > 2)
        usage();

    uint64_t ints[2] = { 0, 0 };
    if (!parse_int_args(args, ints, args.size()))
        usage();
    (*dt.m5_fail)(ints[1], ints[0]);
}

static void
do_reset_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ints[2];
    if (!parse_int_args(args, ints, 2))
        usage();
    (*dt.m5_reset_stats)(ints[0], ints[1]);
}

static void
do_dump_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ints[2];
    if (!parse_int_args(args, ints, 2))
        usage();
    (*dt.m5_dump_stats)(ints[0], ints[1]);
}

static void
do_dump_reset_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ints[2];
    if (!parse_int_args(args, ints, 2))
        usage();
    (*dt.m5_dump_reset_stats)(ints[0], ints[1]);
}

static void
do_read_file(const DispatchTable &dt, Args &args)
{
    if (args.size() > 0)
        usage();

    read_file(dt, STDOUT_FILENO);
}

static void
do_write_file(const DispatchTable &dt, Args &args)
{
    if (args.size() != 1 && args.size() != 2)
        usage();

    const std::string &filename = args.pop();
    const std::string &host_filename = args.pop(filename);

    write_file(dt, filename.c_str(), host_filename.c_str());
}

static void
do_checkpoint(const DispatchTable &dt, Args &args)
{
    uint64_t ints[2];
    if (!parse_int_args(args, ints, 2))
        usage();
    (*dt.m5_checkpoint)(ints[0], ints[1]);
}

static void
do_addsymbol(const DispatchTable &dt, Args &args)
{
    if (args.size() != 2)
        usage();

    uint64_t addr = strtoul(args.pop().c_str(), NULL, 0);
    const std::string &symbol = args.pop();
    (*dt.m5_add_symbol)(addr, symbol.c_str());
}


static void
do_loadsymbol(const DispatchTable &dt, Args &args)
{
    if (args.size() > 0)
        usage();

    (*dt.m5_load_symbol)();
}

static void
do_initparam(const DispatchTable &dt, Args &args)
{
    if (args.size() > 1)
        usage();

    uint64_t key_str[2];
    if (!pack_arg_into_regs(args, key_str, 2))
        usage();
    uint64_t val = (*dt.m5_init_param)(key_str[0], key_str[1]);
    std::cout << val;
}

CommandInfo command_table[] = {
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

int num_commands = sizeof(command_table) / sizeof(CommandInfo);
