
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
 * $Id: interface.h 1.33 05/01/19 13:12:32-06:00 mikem@maya.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef INTERFACE_H
#define INTERFACE_H

#include "Global.hh"
#include "mf_api.hh"
#include "Address.hh"

// // Simics includes
// extern "C" {
// #include "simics/api.hh"
// }

typedef void memory_transaction_t;

// simics memory access
integer_t SIMICS_read_physical_memory(int procID, physical_address_t address,
                                      int len );
void SIMICS_read_physical_memory_buffer(int procID, physical_address_t addr,
                                         char* buffer, int len );
void SIMICS_write_physical_memory( int procID, physical_address_t address,
                                   integer_t value, int len );
void SIMICS_write_physical_memory_buffer(int procID, physical_address_t addr,
                                         char* buffer, int len );
bool SIMICS_check_memory_value(int procID, physical_address_t addr,
                               char* buffer, int len);
const char *SIMICS_disassemble_physical( int procID, physical_address_t pa );

// simics VM translation, decoding, etc.
physical_address_t SIMICS_translate_address( int procID, Address address );
physical_address_t SIMICS_translate_data_address( int procID, Address address );
#ifdef SPARC
bool SIMICS_is_ldda(const memory_transaction_t *mem_trans);
#endif

// simics timing
void SIMICS_unstall_proc(int cpuNumber);
void SIMICS_unstall_proc(int cpuNumber, int cycles);
void SIMICS_stall_proc(int cpuNumber, int cycles);
void SIMICS_post_stall_proc(int cpuNumber, int cycles);
void SIMICS_wakeup_ruby();

// simics callbacks
void SIMICS_remove_ruby_callback( void );
void SIMICS_install_timing_model( void );
void SIMICS_remove_timing_model( void );
void SIMICS_install_exception_callback( void );
void SIMICS_remove_exception_callback( void );
#ifdef SPARC
void SIMICS_install_asi_callback( void );
void SIMICS_remove_asi_callback( void );
#endif

// simics PC, IC
integer_t SIMICS_get_insn_count( int cpuNumber );
integer_t SIMICS_get_cycle_count(int cpuNumber);
Address SIMICS_get_program_counter( void *cpu );
Address SIMICS_get_program_counter( int procID );
Address SIMICS_get_npc(int procID);
void SIMICS_set_program_counter( int procID, Address newPC );
void SIMICS_set_next_program_counter( int procID, Address newPC );
void SIMICS_set_pc( int procID, Address newPC );
void SIMICS_set_npc( int procID, Address newNPC );

void SIMICS_post_continue_execution(int procID);
void SIMICS_post_restart_transaction(int procID);

// simics processor number
int SIMICS_number_processors( void );
void * SIMICS_current_processor( void );
int SIMICS_current_processor_number( void );
int SIMICS_get_proc_no( void *cpu );
void* SIMICS_get_proc_ptr( int cpuNumber );

// simics version
void SIMICS_print_version(ostream& out);

// opal
mf_opal_api_t *SIMICS_get_opal_interface( void );

// STC related, should not be used anymore!
void SIMICS_flush_STC(int cpuNumber);
void SIMICS_invalidate_from_STC(const Address& address, int cpuNumber);
void SIMICS_downgrade_from_STC(const Address& address, int cpuNumber);

// KM -- from Nikhil's SN code
uinteger_t SIMICS_read_control_register(int cpuNumber, int registerNumber);
uinteger_t SIMICS_read_window_register(int cpuNumber, int window, int registerNumber);
uinteger_t SIMICS_read_global_register(int cpuNumber, int globals, int registerNumber);
//uint64 SIMICS_read_fp_register_x(int cpuNumber, int registerNumber);

// KM -- new version based on reg names
int SIMICS_get_register_number(int cpuNumber, const char * reg_name);
const char * SIMICS_get_register_name(int cpuNumber, int reg_num);
uinteger_t SIMICS_read_register(int cpuNumber, int registerNumber);
void SIMICS_write_register(int cpuNumber, int registerNumber, uinteger_t value);

void SIMICS_write_control_register(int cpuNumber, int registerNumber, uinteger_t value);
void SIMICS_write_window_register(int cpuNumber, int window, int registerNumber, uinteger_t value);
void SIMICS_write_global_register(int cpuNumber, int globals, int registerNumber, uinteger_t value);
void SIMICS_write_fp_register_x(int cpuNumber, int registerNumber, uint64 value);
void SIMICS_enable_processor(int cpuNumber);
void SIMICS_disable_processor(int cpuNumber);
void SIMICS_post_disable_processor(int cpuNumber);
bool SIMICS_processor_enabled(int cpuNumber);

void ruby_abort_transaction(void *cpu, void *parameter);
void ruby_set_program_counter(void *cpu, void *parameter);
void ruby_set_pc(void *cpu, void *parameter);
void ruby_set_npc(void *cpu, void *parameter);
void ruby_continue_execution(void *cpu, void *parameter);
void ruby_restart_transaction(void *cpu, void *parameter);
void ruby_stall_proc(void *cpu, void *parameter);
void ruby_disable_processor(void *cpu, void *parameter);

#endif //INTERFACE_H

