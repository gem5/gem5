/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-Only */
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

#ifndef __GEM5_OPS_H__
#define __GEM5_OPS_H__

#include <linux/types.h>

/* Types for op function pointers */
struct gem5_op; /* Forward decl */
typedef ssize_t (*read_f)(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);
typedef ssize_t (*write_f)(struct gem5_op *op, char *buff, size_t len,
                            loff_t *off);

/* Op arg enum */
enum gem5_op_arg
{
    NIL_A=0,
    STR_A,
    INT_A,
};

/* Op configuration struct */
#define GEM5_OPS_MAXARGS 6
struct gem5_op
{
    const char *name;
    u8 opcode;
    read_f read;
    write_f write;
    /* Only for generic ops using op_args() */
    int argc;
    enum gem5_op_arg argt[GEM5_OPS_MAXARGS];
};

/* Exported op configuration list */
extern struct gem5_op op_list[];
extern const int op_count;

/* Utility funcitons */
struct gem5_op *gem5_ops_search(const char *dev_name);

#endif /* __GEM5_OPS_H__ */
