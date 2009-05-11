
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
   This file has been modified by Kevin Moore and Dan Nussbaum of the
   Scalable Systems Research Group at Sun Microsystems Laboratories
   (http://research.sun.com/scalable/) to support the Adaptive
   Transactional Memory Test Platform (ATMTP).

   Please send email to atmtp-interest@sun.com with feedback, questions, or
   to request future announcements about ATMTP.

   ----------------------------------------------------------------------

   File modification date: 2008-02-23

   ----------------------------------------------------------------------
*/

/*
 * $Id$
 *
 * Description:
 *
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#ifdef SPARC
  #define MEMORY_TRANSACTION_TYPE void
#else
  #define MEMORY_TRANSACTION_TYPE void
#endif

int  mh_memorytracer_possible_cache_miss(MEMORY_TRANSACTION_TYPE *mem_trans);
void mh_memorytracer_observe_memory(MEMORY_TRANSACTION_TYPE *mem_trans);

void magic_instruction_callback(void* desc, void * cpu, integer_t val);

void ruby_change_debug_verbosity(char* new_verbosity_str);
void ruby_change_debug_filter(char* new_filter_str);
void ruby_set_debug_output_file (const char * new_filename);
void ruby_set_debug_start_time(char* start_time_str);

void ruby_clear_stats();
void ruby_dump_stats(char* tag);
void ruby_dump_short_stats(char* tag);

void ruby_set_periodic_stats_file(char* filename);
void ruby_set_periodic_stats_interval(int interval);

void ruby_load_caches(char* name);
void ruby_save_caches(char* name);

void ruby_dump_cache(int cpuNumber);
void ruby_dump_cache_data(int cpuNumber, char *tag);

void ruby_set_tracer_output_file (const char * new_filename);
void ruby_xact_visualizer_file (char * new_filename);

void ctrl_exception_start(void* desc, void* cpu, integer_t val);
void ctrl_exception_done(void* desc, void* cpu, integer_t val);

void change_mode_callback(void* desc, void* cpu, integer_t old_mode, integer_t new_mode);
void dtlb_map_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg);
void dtlb_demap_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg);
void dtlb_replace_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg);
void dtlb_overwrite_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg);

integer_t read_reg(void *cpu, const char* reg_name);
void dump_registers(void *cpu);

// Needed so that the ruby module will compile, but functions are
// implemented in Rock.C.
//
void rock_exception_start(void* desc, void* cpu, integer_t val);
void rock_exception_done(void* desc, void* cpu, integer_t val);

#endif //COMMANDS_H
