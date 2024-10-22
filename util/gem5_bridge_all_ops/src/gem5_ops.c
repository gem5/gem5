// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-Only
/*
 * Copyright (c) 2024 The Regents of the University of California
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

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "gem5_bridge.h"
#include "gem5_ops.h"
#include "gem5_poke.h"

/* ========================================================================= *
 * Op Function Forward Declarations
 * ========================================================================= */

static ssize_t read_retval(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp);
static ssize_t read_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp);

static ssize_t set_write_file_dest(struct gem5_op *op, char *buff,
                            size_t len, loff_t *offp);
static ssize_t write_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp);

static ssize_t op_args(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp);


/* ========================================================================= *
 * Op Configurations
 * ========================================================================= */

struct gem5_op op_list[] =
{
    /* Always keep "bridge" as first op, used as backcompat with m5 binary */
    { .name = "bridge",
        /* no opcode */ /* hard-coded mmap */ },

    /* Used to read the return value from gem5 for normal write-arg ops */
    { .name = "retval",
        /* no opcode */ .read  = read_retval },

    /* Special device for setting destination path of writefile op */
    { .name = "writefiledest",
        /* no opcode */ .write = set_write_file_dest },

    /* Opcode [0x00, 0x0f] */
    { .name = "arm",
        .opcode = 0x00, .write = op_args,
        .argc = 1, .argt = {INT_A} },
    { .name = "quiesce",
        .opcode = 0x01, .write = op_args,
        .argc = 0 },
    { .name = "quiescens",
        .opcode = 0x02, .write = op_args,
        .argc = 1, .argt = {INT_A} },
    { .name = "quiescecycle",
        .opcode = 0x03, .write = op_args,
        .argc = 1, .argt = {INT_A} },
    { .name = "quiescetime",
        .opcode = 0x04, .write = op_args,
        .argc = 0 },
    { .name = "rpns",
        .opcode = 0x07, .write = op_args,
        .argc = 0 },
    { .name = "wakecpu",
        .opcode = 0x09, .write = op_args,
        .argc = 1, .argt = {INT_A} },

    /* Opcode [0x20, 0x2f] */
    { .name = "exit",
        .opcode = 0x21, .write = op_args,
        .argc = 1, .argt = {INT_A} },
    { .name = "fail",
        .opcode = 0x22, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "sum",
        .opcode = 0x23, .write = op_args,
        .argc = 6, .argt = {INT_A, INT_A, INT_A, INT_A, INT_A, INT_A} },

    /* Opcode [0x30, 0x3f] */
    { .name = "initparam",
        .opcode = 0x30, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "loadsymbol",
        .opcode = 0x31, .write = op_args,
        .argc = 0 },

    /* Opcode [0x40, 0x4f] */
    { .name = "resetstats",
        .opcode = 0x40, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "dumpstats",
        .opcode = 0x41, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "dumpresetstats",
        .opcode = 0x42, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "checkpoint",
        .opcode = 0x43, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "writefile",
        .opcode = 0x4F, .write = write_file },

    /* Opcode [0x50, 0x5f] */
    { .name = "readfile",
        .opcode = 0x50, .read  = read_file },
    { .name = "debugbreak",
        .opcode = 0x51, .write = op_args,
        .argc = 0 },
    { .name = "switchcpu",
        .opcode = 0x52, .write = op_args,
        .argc = 0 },
    { .name = "addsymbol",
        .opcode = 0x53, .write = op_args,
        .argc = 2, .argt = {INT_A, STR_A} },
    { .name = "panic",
        .opcode = 0x54, .write = op_args,
        .argc = 0 },
    { .name = "workbegin",
        .opcode = 0x5a, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },
    { .name = "workend",
        .opcode = 0x5b, .write = op_args,
        .argc = 2, .argt = {INT_A, INT_A} },

    /* Opcode [0x60, 0x6f] */
    { .name = "disttogglesync",
        .opcode = 0x62, .write = op_args,
        .argc = 0 },

    /* Opcode [0x60, 0x6f] */
    { .name = "workload",
        .opcode = 0x70, .write = op_args,
        .argc = 0 },
};
const int op_count = ARRAY_SIZE(op_list);


/* ========================================================================= *
 * Utility
 * ========================================================================= */

/* Linear search for op struct associated with the given device */
struct gem5_op *gem5_ops_search(const char *dev_name)
{
    int i;
    for (i = 0; i < op_count; ++i) {
        if (!strcmp(dev_name, op_list[i].name)) {
            return &op_list[i];
        }
    }
    return NULL;
}

/* Takes a pointer to a string, extracts the first space-delimited token and
 * has outstr point to it. Advances bufp to the location after the end of this
 * token. */
static int parse_arg_str(char **bufp, char **outstr)
{
    /* Extract token */
    *outstr = NULL;
    do {
        if (!(*bufp) || !(*bufp)[0]) /* End of buffer */
            break;
        *outstr = strsep(bufp, " \t\n");
    } while ((*outstr) && !(*outstr)[0]); /* Skip long chains of whitespace */

    if (!(*outstr) || !(*outstr)[0]) {
        pr_err("%s: missing argument\n", __func__);
        return -EINVAL;
    }

    return 0;
}

/* Calls parse_arg_str to extract a token and then converts it to u64. */
static int parse_arg_int(char **bufp, u64 *outint)
{
    char *argstr;
    if (parse_arg_str(bufp, &argstr) < 0) {
        pr_err("%s: failed to get argument string\n", __func__);
        return -EINVAL;
    }

    /* Convert to int */
    if (kstrtou64(argstr, 10, outint) < 0) {
        pr_err("%s: failed parsing int from \"%s\"\n", __func__, argstr);
        return -EINVAL;
    }

    return 0;
}

#define ARG_PARSE_ERR(argfmt)                                     \
    pr_err("%s: failed parsing args, expected the form \"%s\"\n", \
            __func__, argfmt);



/* ========================================================================= *
 * Op Function Definitions - Specialized
 * ========================================================================= */

static ssize_t read_retval(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp)
{
    ssize_t bytes_read;

    /* Return EOF if after the first read, detected by non-zero offset */
    if (*offp > 0)
        return 0;

    bytes_read = snprintf(buff, len, "%llu\n", gem5_poke_retval);
    if ((size_t)bytes_read >= len) {
        pr_err("%s: buffer size cannot fit retval\n", __func__);
        return -EINVAL;
    }

    *offp += bytes_read;
    return bytes_read;
}

static ssize_t read_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp)
{
    ssize_t bytes_read = 0;

    gem5_poke_opcode = op->opcode;
    bytes_read = POKE3((u64)buff, len, *offp);

    if (bytes_read > 0) /* only push offset on success */
        *offp += bytes_read;
    return bytes_read;
}


static char *write_file_dest = NULL;
static ssize_t set_write_file_dest(struct gem5_op *op, char *buff, size_t len,
                                    loff_t *off)
{
    char *arg;
    int err = parse_arg_str(&buff, &arg);
    if (err) {
        ARG_PARSE_ERR("<host_dest_filepath>");
        return -EINVAL;
    }

    if (write_file_dest)
        kfree(write_file_dest);
    write_file_dest = kstrndup(arg, len, GFP_KERNEL);
    if (!write_file_dest) {
        pr_err("%s: failed parsing arg, expected a host destination path\n",
                __func__);
        return -ENOMEM;
    }

    return len; /* unconditionally consume entire input */
}

static ssize_t write_file(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off)
{
    ssize_t bytes_written = 0;

    if (!write_file_dest) {
        pr_err("%s: need to set destination path by writing to %s\n",
                __func__, "/dev/gem5/writefiledest");
        return -EINVAL;
    }

    gem5_poke_opcode = op->opcode;
    bytes_written = POKE4((u64)buff, len, *off, (u64)write_file_dest);

    if (bytes_written > 0) /* only push offset on success */
        *off += bytes_written;
    return bytes_written;
}


/* ========================================================================= *
 * Op Function Definitions - General
 * ========================================================================= */

static void op_args_error(struct gem5_op *op)
{
    char err_usage_buff[128], *errstr = err_usage_buff;
    int arg_i;

    if (op->argc == 0) {
        errstr += snprintf(errstr, 8, " <none>");
    } else {
        for (arg_i = 0; arg_i < op->argc; ++arg_i) {
            switch (op->argt[arg_i]) {
                case STR_A:
                    errstr += snprintf(errstr, 8, " <str>");
                    break;
                case INT_A:
                    errstr += snprintf(errstr, 8, " <int>");
                    break;
                default:
                    errstr += snprintf(errstr, 8, " <???>");
            }
        }
    }

    pr_err("%s: argument error on arg %d\n", __func__, arg_i);
    pr_err("    expected args of the form:%s\n", err_usage_buff);
}

static ssize_t op_args(struct gem5_op *op, char *buff, size_t len,
                            loff_t *offp)
{
    u64 args[GEM5_OPS_MAXARGS] = { 0 };
    char *argstr;
    int arg_i;
    int err;

    for (arg_i = 0; arg_i < op->argc; ++arg_i) {
        /* Parse args tokens as described by op */
        switch (op->argt[arg_i]) {
            case STR_A:
                err = parse_arg_str(&buff, &argstr);
                args[arg_i] = (u64)argstr; /* store pointer as u64 */
                break;
            case INT_A:
                err = parse_arg_int(&buff, &args[arg_i]);
                break;
            default:
                pr_err("%s: unrecognized arg type\n", __func__);
                return -EINVAL;
        }

        /* Report arg usage errors */
        if (err) {
            op_args_error(op);
            return -EINVAL;
        }
    }

    gem5_poke_opcode = op->opcode;
    (void)POKE6(args[0], args[1], args[2], args[3], args[4], args[5]);
    return len; /* unconditionally consume entire input */
}


/* ========================================================================= *
 * FOOTER
 * ========================================================================= */

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Noah Krim");
MODULE_DESCRIPTION(
    "Supports gem5_bridge by implementing gem5 op ABI and functionality"
);
