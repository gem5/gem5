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

#include <cstring>
#include <fstream>
#include <iostream>

#include "args.hh"
#include "commands.hh"
#include "usage.hh"

void
Command::run(const DispatchTable &dt, Args &args)
{
    const int num_args = args.size();
    if (num_args < minArgs || num_args > maxArgs)
        usage();

    func(dt, args);
}


static int
read_file(const DispatchTable &dt, std::ostream &os)
{
    char buf[256 * 1024];

    // Touch all buffer pages to ensure they are mapped in the
    // page table. This is required in the case of X86_FS, where
    // Linux does demand paging.
    memset(buf, 0, sizeof(buf));

    int len;
    int offset = 0;
    while ((len = (*dt.m5_read_file)(buf, sizeof(buf), offset)) > 0) {
        os.write(buf, len);
        os.flush();
        if (!os) {
            std::cerr << "Failed to write file" << std::endl;
            exit(2);
        }
        offset += len;
    }

    return offset;
}

static void
write_file(const DispatchTable &dt, const std::string &filename,
           const std::string &host_filename)
{
    std::cerr << "opening " << filename << std::endl;
    std::ifstream src(filename, std::ios_base::in | std::ios_base::binary);

    if (!src) {
        std::cerr << "error opening " << filename << std::endl;
        return;
    }

    char buf[256 * 1024];
    int offset = 0;

    memset(buf, 0, sizeof(buf));

    while (true) {
        src.seekg(offset);
        src.read(buf, sizeof(buf));
        int len = src.gcount();
        if (!src && !src.eof())
            break;
        char *wbuf = buf;
        while (len) {
            int bytes = (*dt.m5_write_file)(
                    wbuf, len, offset, host_filename.c_str());
            len -= bytes;
            offset += bytes;
            wbuf += bytes;
        }
        if (src.eof())
            break;
    }
    std::cerr << "Wrote " << offset << " bytes." << std::endl;
}

static void
do_exit(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay;
    if (!args.pop(ns_delay, 0))
        usage();

    (*dt.m5_exit)(ns_delay);
}

// For testing purposes.
static void
do_sum(const DispatchTable &dt, Args &args)
{
    uint64_t a, b, c, d, e, f;
    if (!args.pop(a) || !args.pop(b) || !args.pop(c, 0) ||
            !args.pop(d, 0) || !args.pop(e, 0) || !args.pop(f, 0))
        usage();

    unsigned sum = (*dt.m5_sum)(a, b, c, d, e, f);
    std::cout << "Sum is " << sum << "." << std::endl;
}

static void
do_fail(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay, code;
    if (!args.pop(code) || !args.pop(ns_delay, 0))
        usage();

    (*dt.m5_fail)(ns_delay, code);
}

static void
do_reset_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay, ns_period;
    if (!args.pop(ns_delay, 0) || !args.pop(ns_period, 0))
        usage();

    (*dt.m5_reset_stats)(ns_delay, ns_period);
}

static void
do_dump_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay, ns_period;
    if (!args.pop(ns_delay, 0) || !args.pop(ns_period, 0))
        usage();

    (*dt.m5_dump_stats)(ns_delay, ns_period);
}

static void
do_dump_reset_stats(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay, ns_period;
    if (!args.pop(ns_delay, 0) || !args.pop(ns_period, 0))
        usage();

    (*dt.m5_dump_reset_stats)(ns_delay, ns_period);
}

static void
do_read_file(const DispatchTable &dt, Args &args)
{
    if (args.size() > 0)
        usage();

    read_file(dt, std::cout);
}

static void
do_write_file(const DispatchTable &dt, Args &args)
{
    const std::string &filename = args.pop();
    const std::string &host_filename = args.pop(filename);

    write_file(dt, filename, host_filename);
}

static void
do_checkpoint(const DispatchTable &dt, Args &args)
{
    uint64_t ns_delay, ns_period;
    if (!args.pop(ns_delay, 0) || !args.pop(ns_period, 0))
        usage();

    (*dt.m5_checkpoint)(ns_delay, ns_period);
}

static void
do_addsymbol(const DispatchTable &dt, Args &args)
{
    uint64_t addr;
    if (!args.pop(addr))
        usage();
    const std::string &symbol = args.pop();

    (*dt.m5_add_symbol)(addr, symbol.c_str());
}


static void
do_loadsymbol(const DispatchTable &dt, Args &args)
{
    (*dt.m5_load_symbol)();
}

static void
do_initparam(const DispatchTable &dt, Args &args)
{
    uint64_t key_str[2];
    if (!args.pop(key_str, 2))
        usage();
    uint64_t val = (*dt.m5_init_param)(key_str[0], key_str[1]);
    std::cout << val;
}

std::map<std::string, Command> Command::map = {
    { "addsymbol", { 2, 2, do_addsymbol, "<address> <symbol>\n"
        "        Adds a symbol with address \"address\" to gem5's "
            "symbol table" }},
    { "checkpoint", { 0, 2, do_checkpoint, "[delay [period]]\n"
        "        After delay (default 0) take a checkpoint, and then "
            "optionally every period after" }},
    { "dumpresetstats", { 0, 2, do_dump_reset_stats, "[delay [period]]\n"
        "        After delay (default 0) dump and reset the stats, and then "
            "optionally every period after" }},
    { "dumpstats", { 0, 2, do_dump_stats, "[delay [period]]\n"
        "        After delay (default 0) dump the stats, and then optionally "
            "every period after" }},
    { "exit", { 0, 1, do_exit, "[delay]\n"
        "        Exit after delay, or immediately" }},
    { "fail", { 1, 2, do_fail, "<code> [delay]\n"
        "        Exit with failure code code after delay, or immediately" }},
    { "sum", { 2, 6, do_sum, "<a> <b> [c [d [e [f]]]]\n"
        "        Sum a-f (defaults are 0), for testing purposes" }},
    { "initparam", { 1, 1, do_initparam, "[key]\n"
        "        optional key may be at most 16 characters long" }},
    { "loadsymbol", { 0, 0, do_loadsymbol, "\n"
        "        load a preselected symbol file into gem5's symbol table" }},
    { "readfile", { 0, 0, do_read_file, "\n"
        "        read a preselected file from the host and write it to "
            "stdout" }},
    { "resetstats", { 0, 2, do_reset_stats, "[delay [period]]\n"
        "        After delay (default 0) reset the stats, and then "
            "optionally every period after" }},
    { "writefile", { 1, 2, do_write_file, "<filename> [host filename]\n"
        "        Write a file to the host, optionally with a different "
            "name" }},
};
