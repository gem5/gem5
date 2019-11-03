/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 *          Nathan Binkert
 *          Ali Saidi
 */

#include <gem5/asm/generic/m5ops.h>

#if defined(M5OP_ADDR) && defined(M5OP_PIC)
/* Use the memory mapped m5op interface */
#define TWO_BYTE_OP(name, number)         \
        .globl name;                      \
        .func name;                       \
name:                                     \
        mov m5_mem@gotpcrel(%rip), %r11;  \
        mov (%r11), %r11;                 \
        mov $number, %rax;                \
        shl $8, %rax;                     \
        mov 0(%r11, %rax, 1), %rax;       \
        ret;                              \
        .endfunc;

#elif defined(M5OP_ADDR) && !defined(M5OP_PIC)
/* Use the memory mapped m5op interface */
#define TWO_BYTE_OP(name, number)         \
        .globl name;                      \
        .func name;                       \
name:                                     \
        mov m5_mem, %r11;                 \
        mov $number, %rax;                \
        shl $8, %rax;                     \
        mov 0(%r11, %rax, 1), %rax;       \
        ret;                              \
        .endfunc;

#else
/* Use the magic instruction based m5op interface. This does not work
 * in virtualized environments.
 */

#define TWO_BYTE_OP(name, number)         \
        .globl name;                      \
        .func name;                       \
name:                                     \
        .byte 0x0F, 0x04;                 \
        .word number;                     \
        ret;                              \
        .endfunc;

#endif

TWO_BYTE_OP(m5_arm, M5OP_ARM)
TWO_BYTE_OP(m5_quiesce, M5OP_QUIESCE)
TWO_BYTE_OP(m5_quiesce_ns, M5OP_QUIESCE_NS)
TWO_BYTE_OP(m5_quiesce_cycle, M5OP_QUIESCE_CYCLE)
TWO_BYTE_OP(m5_quiesce_time, M5OP_QUIESCE_TIME)
TWO_BYTE_OP(m5_rpns, M5OP_RPNS)
TWO_BYTE_OP(m5_wake_cpu, M5OP_WAKE_CPU)
TWO_BYTE_OP(m5_exit, M5OP_EXIT)
TWO_BYTE_OP(m5_fail, M5OP_FAIL)
TWO_BYTE_OP(m5_init_param, M5OP_INIT_PARAM)
TWO_BYTE_OP(m5_load_symbol, M5OP_LOAD_SYMBOL)
TWO_BYTE_OP(m5_reset_stats, M5OP_RESET_STATS)
TWO_BYTE_OP(m5_dump_stats, M5OP_DUMP_STATS)
TWO_BYTE_OP(m5_dump_reset_stats, M5OP_DUMP_RESET_STATS)
TWO_BYTE_OP(m5_checkpoint, M5OP_CHECKPOINT)
TWO_BYTE_OP(m5_read_file, M5OP_READ_FILE)
TWO_BYTE_OP(m5_write_file, M5OP_WRITE_FILE)
TWO_BYTE_OP(m5_debug_break, M5OP_DEBUG_BREAK)
TWO_BYTE_OP(m5_switch_cpu, M5OP_SWITCH_CPU)
TWO_BYTE_OP(m5_add_symbol, M5OP_ADD_SYMBOL)
TWO_BYTE_OP(m5_panic, M5OP_PANIC)
TWO_BYTE_OP(m5_work_begin, M5OP_WORK_BEGIN)
TWO_BYTE_OP(m5_work_end, M5OP_WORK_END)
TWO_BYTE_OP(m5_dist_toggle_sync, M5OP_DIST_TOGGLE_SYNC)
