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
 */

#ifndef __GEM5_M5OP_H__
#define __GEM5_M5OP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void m5_arm(uint64_t address);
void m5_quiesce(void);
void m5_quiesce_ns(uint64_t ns);
void m5_quiesce_cycle(uint64_t cycles);
uint64_t m5_quiesce_time(void);
uint64_t m5_rpns();
void m5_wake_cpu(uint64_t cpuid);

void m5_exit(uint64_t ns_delay);
void m5_fail(uint64_t ns_delay, uint64_t code);
uint64_t m5_init_param(uint64_t key_str1, uint64_t key_str2);
void m5_checkpoint(uint64_t ns_delay, uint64_t ns_period);
void m5_reset_stats(uint64_t ns_delay, uint64_t ns_period);
void m5_dump_stats(uint64_t ns_delay, uint64_t ns_period);
void m5_dump_reset_stats(uint64_t ns_delay, uint64_t ns_period);
uint64_t m5_read_file(void *buffer, uint64_t len, uint64_t offset);
uint64_t m5_write_file(void *buffer, uint64_t len, uint64_t offset,
                       const char *filename);
void m5_debug_break(void);
void m5_switch_cpu(void);
void m5_dist_toggle_sync(void);
void m5_add_symbol(uint64_t addr, const char *symbol);
void m5_load_symbol();
void m5_panic(void);
void m5_work_begin(uint64_t workid, uint64_t threadid);
void m5_work_end(uint64_t workid, uint64_t threadid);

void m5_se_syscall();
void m5_se_page_fault();

#ifdef __cplusplus
}
#endif
#endif // __GEM5_M5OP_H__
