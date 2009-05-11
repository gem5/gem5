
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
 * $Id: interface.C 1.39 05/01/19 13:12:31-06:00 mikem@maya.cs.wisc.edu $
 *
 */

#include "Global.hh"
#include "System.hh"
#include "OpalInterface.hh"
#include "EventQueue.hh"
#include "mf_api.hh"
#include "interface.hh"
#include "Sequencer.hh"
// #include "TransactionInterfaceManager.hh"

#ifdef CONTIGUOUS_ADDRESSES
#include "ContiguousAddressTranslator.hh"

/* Also used in init.C, commands.C */
ContiguousAddressTranslator * g_p_ca_translator = NULL;

#endif // #ifdef CONTIGUOUS_ADDRESSES

////////////////////////   Local helper functions //////////////////////

// Callback when exception occur
static void core_exception_callback(void *data, void *cpu,
                                    integer_t exc)
{
  // SimicsDriver *simics_intf = dynamic_cast<SimicsDriver*>(g_system_ptr->getDriver());
  // ASSERT( simics_intf );
  // simics_intf->exceptionCallback(cpu, exc);
  assert(0);
}

#ifdef SPARC
// Callback when asi accesses occur
// static exception_type_t core_asi_callback(void * cpu, generic_transaction_t *g)
// {
//   SimicsDriver *simics_intf = dynamic_cast<SimicsDriver*>(g_system_ptr->getDriver());
//   assert( simics_intf );
//   return simics_intf->asiCallback(cpu, g);
// }
#endif

static void runRubyEventQueue(void* obj, void* arg)
{
  Time time = g_eventQueue_ptr->getTime() + 1;
  DEBUG_EXPR(NODE_COMP, HighPrio, time);
  g_eventQueue_ptr->triggerEvents(time);
//   void* obj_ptr = (void*) SIM_proc_no_2_ptr(0);  // Maurice
//   SIM_time_post_cycle(obj_ptr, SIMICS_RUBY_MULTIPLIER, Sim_Sync_Processor, &runRubyEventQueue, NULL);  // Maurice
  assert(0);
}

////////////////////////   Simics API functions //////////////////////

int SIMICS_number_processors()
{
//   return SIM_number_processors();  // Maurice
  assert(0);
  return 0;
}

void SIMICS_wakeup_ruby()
{
//   void* obj_ptr = (void*) SIM_proc_no_2_ptr(0);  // Maurice
//   SIM_time_post_cycle(obj_ptr, SIMICS_RUBY_MULTIPLIER, Sim_Sync_Processor, &runRubyEventQueue, NULL);  // Maurice
  assert(0);
}

// an analogue to wakeup ruby, this function ends the callbacks ruby normally
// recieves from simics. (it removes ruby from simics's event queue). This
// function should only be called when opal is installed. Opal advances ruby's
// event queue independently of simics.
void SIMICS_remove_ruby_callback( void )
{
//   void* obj_ptr = (void*) SIM_proc_no_2_ptr(0);  // Maurice
//   SIM_time_clean( obj_ptr, Sim_Sync_Processor, &runRubyEventQueue, NULL);  // Maurice
  assert(0);
}

// Install ruby as the timing model (analogous code exists in ruby/ruby.c)
void SIMICS_install_timing_model( void )
{
// //   void *phys_mem0 = SIM_get_object("phys_mem0");  // Maurice
//   attr_value_t   val;
// //   val.kind     = Sim_Val_String;  // Maurice
//   val.u.string = "ruby0";
//   set_error_t install_error;
//
//   if(phys_mem0==NULL) {
//     /* Look for "phys_mem" instead */
// //     SIM_clear_exception();  // Maurice
// //     phys_mem0 = SIM_get_object("phys_mem");  // Maurice
//   }
//
//   if(phys_mem0==NULL) {
//     /* Okay, now panic... can't install ruby without a physical memory object */
//     WARN_MSG( "Cannot Install Ruby... no phys_mem0 or phys_mem object found" );
//     WARN_MSG( "Ruby is NOT installed." );
// //     SIM_clear_exception();  // Maurice
//     return;
//   }
//
// //   install_error = SIM_set_attribute(phys_mem0, "timing_model", &val);  // Maurice
//
// //   if (install_error == Sim_Set_Ok) {  // Maurice
//     WARN_MSG( "successful installation of the ruby timing model" );
//   } else {
//     WARN_MSG( "error installing ruby timing model" );
// //     WARN_MSG( SIM_last_error() );  // Maurice
//   }

  assert(0);
}

// Removes ruby as the timing model interface
void SIMICS_remove_timing_model( void )
{
//   void *phys_mem0 = SIM_get_object("phys_mem0");  // Maurice
//   attr_value_t   val;
//   memset( &val, 0, sizeof(attr_value_t) );
// //   val.kind = Sim_Val_Nil;  // Maurice
//
//   if(phys_mem0==NULL) {
//     /* Look for "phys_mem" instead */
// //     SIM_clear_exception();  // Maurice
// //     phys_mem0 = SIM_get_object("phys_mem");  // Maurice
//   }
//
//   if(phys_mem0==NULL) {
//     /* Okay, now panic... can't uninstall ruby without a physical memory object */
//     WARN_MSG( "Cannot Uninstall Ruby... no phys_mem0 or phys_mem object found" );
//     WARN_MSG( "Uninstall NOT performed." );
// //     SIM_clear_exception();  // Maurice
//     return;
//   }
//
// //   SIM_set_attribute(phys_mem0, "timing_model", &val);  // Maurice
  assert(0);
}

// Installs the (SimicsDriver) function to recieve the exeception callback
void SIMICS_install_exception_callback( void )
{
  // install exception callback
  // s_exception_hap_handle =
//     SIM_hap_add_callback("Core_Exception",  // Maurice
     //                    (obj_hap_func_t)core_exception_callback, NULL );
  assert(0);
}

// removes the exception callback
void SIMICS_remove_exception_callback( void )
{
  // uninstall exception callback
//   SIM_hap_delete_callback_id( "Core_Exception",  // Maurice
    //                          s_exception_hap_handle );
  assert(0);
}

#ifdef SPARC
// Installs the (SimicsDriver) function to recieve the asi callback
void SIMICS_install_asi_callback( void )
{
//   for(int i = 0; i < SIM_number_processors(); i++) {  // Maurice
 //   sparc_v9_interface_t *v9_interface = (sparc_v9_interface_t *)
//       SIM_get_interface(SIM_proc_no_2_ptr(i), SPARC_V9_INTERFACE);  // Maurice

    // init asi callbacks, 16bit ASI
  //  for(int j = 0; j < MAX_ADDRESS_SPACE_ID; j++) {
    //  v9_interface->install_user_asi_handler(core_asi_callback, j);
 //   }
  // }
  assert(0);
}

// removes the asi callback
void SIMICS_remove_asi_callback( void )
{
//   for(int i = 0; i < SIM_number_processors(); i++) {  // Maurice
//    sparc_v9_interface_t *v9_interface = (sparc_v9_interface_t *)
//       SIM_get_interface(SIM_proc_no_2_ptr(i), SPARC_V9_INTERFACE);  // Maurice

    // disable asi callback
  //  for(int j = 0; j < MAX_ADDRESS_SPACE_ID; j++) {
  //    v9_interface->remove_user_asi_handler(core_asi_callback, j);
  //  }
 // }
  assert(0);
}
#endif

// Query simics for the presence of the opal object.
// returns its interface if found, NULL otherwise
mf_opal_api_t *SIMICS_get_opal_interface( void )
{
//   void *opal = SIM_get_object("opal0");  // Maurice
  //if (opal != NULL) {
//     mf_opal_api_t *opal_intf  = (mf_opal_api_t *) SIM_get_interface( opal, "mf-opal-api" );  // Maurice
   // if ( opal_intf != NULL ) {
   //   return opal_intf;
//     } else {
//       WARN_MSG("error: OpalInterface: opal does not implement mf-opal-api interface.\n");
//       return NULL;
//     }
//   }
//   SIM_clear_exception();     // Maurice
  assert(0);
  return NULL;
}

void * SIMICS_current_processor(){
//   return SIM_current_processor();  // Maurice
  assert(0);
  return NULL;
}

int SIMICS_current_processor_number()
{
//   return (SIM_get_proc_no((processor_t *) SIM_current_processor()));  // Maurice
  assert(0);
  return 0;
}

integer_t SIMICS_get_insn_count(int cpuNumber)
{
  // NOTE: we already pass in the logical cpuNumber (ie Simics simulated cpu number)
  int num_smt_threads = RubyConfig::numberofSMTThreads();
  integer_t total_insn = 0;
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   total_insn += SIM_step_count((void*) cpu);   // Maurice
  assert(0);
  return total_insn;
}

integer_t SIMICS_get_cycle_count(int cpuNumber)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   integer_t result =  SIM_cycle_count((void*) cpu);   // Maurice
  assert(0);
  return 0;
}

void SIMICS_unstall_proc(int cpuNumber)
{
//   void* proc_ptr = (void *) SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_stall_cycle(proc_ptr, 0);  // Maurice
  assert(0);
}

void SIMICS_unstall_proc(int cpuNumber, int cycles)
{
//   void* proc_ptr = (void *) SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_stall_cycle(proc_ptr, cycles);  // Maurice
  assert(0);
}

void SIMICS_stall_proc(int cpuNumber, int cycles)
{
//   void* proc_ptr = (void*) SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   if (SIM_stalled_until(proc_ptr) != 0){  // Maurice
//     cout << cpuNumber << " Trying to stall. Stall Count currently at " << SIM_stalled_until(proc_ptr) << endl;  // Maurice
//  }
//   SIM_stall_cycle(proc_ptr, cycles);  // Maurice
  assert(0);
}

void SIMICS_post_stall_proc(int cpuNumber, int cycles)
{
//   void* proc_ptr = (void*) SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_stacked_post(proc_ptr, ruby_stall_proc, (void *) cycles);  // Maurice
  assert(0);
}

integer_t SIMICS_read_physical_memory( int procID, physical_address_t address,
                                       int len )
{
// //   SIM_clear_exception();        // Maurice
//   ASSERT( len <= 8 );
// #ifdef CONTIGUOUS_ADDRESSES
//   if(g_p_ca_translator != NULL) {
//     address = g_p_ca_translator->TranslateRubyToSimics( address );
//   }
// #endif // #ifdef CONTIGUOUS_ADDRESSES
// //   integer_t result = SIM_read_phys_memory( SIM_proc_no_2_ptr(procID),  // Maurice
// // //                                           address, len );
// //
// // //   int isexcept = SIM_get_pending_exception();  // Maurice
// //   if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// // //     sim_exception_t except_code = SIM_clear_exception();  // Maurice
// //     WARN_MSG( "SIMICS_read_physical_memory: raised exception." );
// // //     WARN_MSG( SIM_last_error() );  // Maurice
// //     WARN_MSG( Address(address) );
// //     WARN_MSG( procID );
// //     ASSERT(0);
// //   }
// //   return ( result );
  assert(0);
   return 0;
}
//
// /*
//  * Read data into a buffer and assume the buffer is already allocated
//  */
void SIMICS_read_physical_memory_buffer(int procID, physical_address_t addr,
                                         char* buffer, int len ) {
// // //   processor_t* obj = SIM_proc_no_2_ptr(procID);  // Maurice
// //
// //   assert( obj != NULL);
// //   assert( buffer != NULL );
// //
// // #ifdef CONTIGUOUS_ADDRESSES
// //   if(g_p_ca_translator != NULL) {
// //     addr = g_p_ca_translator->TranslateRubyToSimics( addr );
// //   }
// // #endif // #ifdef CONTIGUOUS_ADDRESSES
// //
// //   int buffer_pos = 0;
// //   physical_address_t start = addr;
// //   do {
// //     int size = (len < 8)? len:8;
// // //     integer_t result = SIM_read_phys_memory( obj, start, size );  // Maurice
// // //     int isexcept = SIM_get_pending_exception();  // Maurice
// //     if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// // //       sim_exception_t except_code = SIM_clear_exception();  // Maurice
// //       WARN_MSG( "SIMICS_read_physical_memory_buffer: raised exception." );
// // //       WARN_MSG( SIM_last_error() );  // Maurice
// //       WARN_MSG( addr );
// //       WARN_MSG( procID );
// //       ASSERT( 0 );
// //     }
// //
// // #ifdef SPARC
// //     // assume big endian (i.e. SPARC V9 target)
// //     for(int i = size-1; i >= 0; i--) {
// // #else
// //     // assume little endian (i.e. x86 target)
// //     for(int i = 0; i<size; i++) {
// // #endif
// //       buffer[buffer_pos++] = (char) ((result>>(i<<3))&0xff);
// //     }
// //
// //     len -= size;
// //     start += size;
// //   } while(len != 0);
  assert(0);
}
//
void SIMICS_write_physical_memory( int procID, physical_address_t address,
                                    integer_t value, int len )
 {
// //   ASSERT( len <= 8 );
// //
// // //   SIM_clear_exception();  // Maurice
// //
// // //   processor_t* obj = SIM_proc_no_2_ptr(procID);  // Maurice
// //
// // #ifdef CONTIGUOUS_ADDRESSES
// //   if(g_p_ca_translator != NULL) {
// //     address = g_p_ca_translator->TranslateRubyToSimics( address );
// //   }
// // #endif // #ifdef CONTIGUOUS_ADDRESSES
// //
// // //   int isexcept = SIM_get_pending_exception();  // Maurice
// //   if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// // //     sim_exception_t except_code = SIM_clear_exception();  // Maurice
// //     WARN_MSG( "SIMICS_write_physical_memory 1: raised exception." );
// // //     WARN_MSG( SIM_last_error() );  // Maurice
// //     WARN_MSG( address );
// //   }
// //
// // //   SIM_write_phys_memory(obj, address, value, len );  // Maurice
// //
// // //   isexcept = SIM_get_pending_exception();  // Maurice
// //   if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// // //     sim_exception_t except_code = SIM_clear_exception();  // Maurice
// //     WARN_MSG( "SIMICS_write_physical_memory 2: raised exception." );
// // //     WARN_MSG( SIM_last_error() );  // Maurice
// //     WARN_MSG( address );
// //   }
  assert(0);
}
//
// /*
//  * write data to simics memory from a buffer (assumes the buffer is valid)
//  */
void SIMICS_write_physical_memory_buffer(int procID, physical_address_t addr,
                                         char* buffer, int len ) {
// //   processor_t* obj = SIM_proc_no_2_ptr(procID);  // Maurice
//
//   assert( obj != NULL);
//   assert( buffer != NULL );
//
// #ifdef CONTIGUOUS_ADDRESSES
//   if(g_p_ca_translator != NULL) {
//     addr = g_p_ca_translator->TranslateRubyToSimics( addr );
//   }
// #endif // #ifdef CONTIGUOUS_ADDRESSES
//
//   int buffer_pos = 0;
//   physical_address_t start = addr;
//   do {
//     int size = (len < 8)? len:8;
// //     //integer_t result = SIM_read_phys_memory( obj, start, size );  // Maurice
//     integer_t value = 0;
// #ifdef SPARC
//     // assume big endian (i.e. SPARC V9 target)
//     for(int i = size-1; i >= 0; i--) {
// #else
//     // assume little endian (i.e. x86 target)
//     for(int i = 0; i<size; i++) {
// #endif
//       integer_t mask = buffer[buffer_pos++];
//       value |= ((mask)<<(i<<3));
//     }
//
//
// //     SIM_write_phys_memory( obj, start, value, size);  // Maurice
// //     int isexcept = SIM_get_pending_exception();  // Maurice
//     if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// //       sim_exception_t except_code = SIM_clear_exception();  // Maurice
//       WARN_MSG( "SIMICS_write_physical_memory_buffer: raised exception." );
// //       WARN_MSG( SIM_last_error() );  // Maurice
//       WARN_MSG( addr );
//     }
//
//     len -= size;
//     start += size;
//   } while(len != 0);
  assert(0);
}

bool SIMICS_check_memory_value(int procID, physical_address_t addr,
                               char* buffer, int len) {
  char buf[len];
  SIMICS_read_physical_memory_buffer(procID, addr, buf, len);
  assert(0);
  return (memcmp(buffer, buf, len) == 0)? true:false;
}

physical_address_t SIMICS_translate_address( int procID, Address address ) {
//   SIM_clear_exception();        // Maurice
//   physical_address_t physical_addr = SIM_logical_to_physical(SIM_proc_no_2_ptr(procID), Sim_DI_Instruction, address.getAddress() );  // Maurice
//   int isexcept = SIM_get_pending_exception();  // Maurice
//  if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
//     sim_exception_t except_code = SIM_clear_exception();  // Maurice
//     /*
//     WARN_MSG( "SIMICS_translate_address: raised exception." );
//     WARN_MSG( procID );
//     WARN_MSG( address );
// //     WARN_MSG( SIM_last_error() );  // Maurice
//     */
//     return 0;
//   }
//
// #ifdef CONTIGUOUS_ADDRESSES
//   if(g_p_ca_translator != NULL) {
//     physical_addr = g_p_ca_translator->TranslateSimicsToRuby( physical_addr );
//   }
// #endif // #ifdef CONTIGUOUS_ADDRESSES
//
//   return physical_addr;
  assert(0);
  return 0;
}

physical_address_t SIMICS_translate_data_address( int procID, Address address ) {
//   SIM_clear_exception();        // Maurice
//   physical_address_t physical_addr = SIM_logical_to_physical(SIM_proc_no_2_ptr(procID), Sim_DI_Data, address.getAddress() );  // Maurice
//   int isexcept = SIM_get_pending_exception();  // Maurice
//  if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
//     sim_exception_t except_code = SIM_clear_exception();  // Maurice
    /*
    WARN_MSG( "SIMICS_translate_data_address: raised exception." );
    WARN_MSG( procID );
    WARN_MSG( address );
//     WARN_MSG( SIM_last_error() );  // Maurice
    */
//  }
//  return physical_addr;
  assert(0);
  return 0;
}

#ifdef SPARC
bool SIMICS_is_ldda(const memory_transaction_t *mem_trans) {
//  void *cpu  = mem_trans->s.ini_ptr;
//  int            proc= SIMICS_get_proc_no(cpu);
//  Address        addr = SIMICS_get_program_counter(cpu);
//  physical_address_t phys_addr = SIMICS_translate_address( proc, addr );
//  uint32         instr= SIMICS_read_physical_memory( proc, phys_addr, 4 );
//
//  // determine if this is a "ldda" instruction (non-exclusive atomic)
//  // ldda bit mask:  1100 0001 1111 1000 == 0xc1f80000
//  // ldda match   :  1100 0000 1001 1000 == 0xc0980000
//  if ( (instr & 0xc1f80000) == 0xc0980000 ) {
//    // should exactly be ldda instructions
//    ASSERT(!strncmp(SIMICS_disassemble_physical(proc, phys_addr), "ldda", 4));
//    //cout << "SIMICS_is_ldda END" << endl;
//    return true;
//   }
//   return false;
  assert(0);
  return false;
}
#endif

const char *SIMICS_disassemble_physical( int procID, physical_address_t pa ) {
//#ifdef CONTIGUOUS_ADDRESSES
//  if(g_p_ca_translator != NULL) {
//    pa = g_p_ca_translator->TranslateRubyToSimics( pa );
//  }
//#endif // #ifdef CONTIGUOUS_ADDRESSES
//   return SIM_disassemble( SIM_proc_no_2_ptr(procID), pa , /* physical */ 0)->string;  // Maurice
  assert(0);
  return "There is no spoon";
}

Address SIMICS_get_program_counter(void *cpu) {
  assert(cpu != NULL);
//   return Address(SIM_get_program_counter((processor_t *) cpu));  // Maurice
  assert(0);
  return Address(0);
}

Address SIMICS_get_npc(int procID) {
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//   return Address(SIM_read_register(cpu, SIM_get_register_number(cpu, "npc")));  // Maurice
  assert(0);
  return Address(0);
}

Address SIMICS_get_program_counter(int procID) {
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//  assert(cpu != NULL);

//   Address addr = Address(SIM_get_program_counter(cpu));  // Maurice
  assert(0);
  return Address(0);
}

// /* NOTE: SIM_set_program_counter sets NPC to PC+4 */  // Maurice
void SIMICS_set_program_counter(int procID, Address newPC) {
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//  assert(cpu != NULL);

//   SIM_stacked_post(cpu, ruby_set_program_counter, (void*) newPC.getAddress());  // Maurice
  assert(0);
}

void SIMICS_set_pc(int procID, Address newPC) {
  // IMPORTANT: procID is the SIMICS simulated proc number (takes into account SMT)
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//  assert(cpu != NULL);
//
//   if(OpalInterface::isOpalLoaded() == false){
// //     SIM_set_program_counter(cpu, newPC.getAddress());  // Maurice
//   } else {
//     // explicitly change PC
//     ruby_set_pc( cpu, (void *) newPC.getAddress() );
//   }
// //   int isexcept = SIM_get_pending_exception();  // Maurice
//   if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// //     sim_exception_t except_code = SIM_clear_exception();  // Maurice
//     WARN_MSG( "SIMICS_set_pc: raised exception." );
// //     WARN_MSG( SIM_last_error() );  // Maurice
//     ASSERT(0);
//   }
  assert(0);
}

void SIMICS_set_next_program_counter(int procID, Address newNPC) {
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//  assert(cpu != NULL);

//   SIM_stacked_post(cpu, ruby_set_npc, (void*) newNPC.getAddress());  // Maurice
  assert(0);
}

void SIMICS_set_npc(int procID, Address newNPC) {
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//   assert(cpu != NULL);
//
//   if(OpalInterface::isOpalLoaded() == false){
// //     SIM_write_register(cpu, SIM_get_register_number(cpu, "npc"), newNPC.getAddress());  // Maurice
//   } else {
//     // explicitly change NPC
//     ruby_set_npc( cpu, (void *) newNPC.getAddress() );
//   }
//
// //   int isexcept = SIM_get_pending_exception();  // Maurice
//   if ( !(isexcept == SimExc_No_Exception || isexcept == SimExc_Break) ) {
// //     sim_exception_t except_code = SIM_clear_exception();  // Maurice
//     WARN_MSG( "SIMICS_set_npc: raised exception " );
// //     WARN_MSG( SIM_last_error() );  // Maurice
//     ASSERT(0);
//   }
  assert(0);
}

void SIMICS_post_continue_execution(int procID){
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//   assert(cpu != NULL);
//
//   if(OpalInterface::isOpalLoaded() == false){
// //     SIM_stacked_post(cpu, ruby_continue_execution, (void *) NULL);  // Maurice
//   } else{
//     ruby_continue_execution( cpu, (void *) NULL );
//   }
  assert(0);
}

void SIMICS_post_restart_transaction(int procID){
//   void *cpu = SIM_proc_no_2_ptr(procID);  // Maurice
//   assert(cpu != NULL);
//
//   if(OpalInterface::isOpalLoaded() == false){
// //     SIM_stacked_post(cpu, ruby_restart_transaction, (void *) NULL);  // Maurice
//   } else{
//     ruby_restart_transaction( cpu, (void *) NULL );
//   }
  assert(0);
}

// return -1 when fail
int SIMICS_get_proc_no(void *cpu) {
//   int proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//  return proc_no;
  assert(0);
   return -1;
}

void SIMICS_disable_processor( int cpuNumber ) {
//   if(SIM_cpu_enabled(SIMICS_get_proc_ptr(cpuNumber))) {  // Maurice
//     SIM_disable_processor(SIMICS_get_proc_ptr(cpuNumber));  // Maurice
//   } else {
//     WARN_MSG(cpuNumber);
//     WARN_MSG( "Tried to disable a 'disabled' processor");
//     ASSERT(0);
//   }
  assert(0);
}

void SIMICS_post_disable_processor( int cpuNumber ) {
//   SIM_stacked_post(SIMICS_get_proc_ptr(cpuNumber), ruby_disable_processor, (void*) NULL);  // Maurice
  assert(0);
}

void SIMICS_enable_processor( int cpuNumber ) {
//   if(!SIM_cpu_enabled(SIMICS_get_proc_ptr(cpuNumber))) {  // Maurice
//     SIM_enable_processor(SIMICS_get_proc_ptr(cpuNumber));  // Maurice
//  } else {
//    WARN_MSG(cpuNumber);
//    WARN_MSG( "Tried to enable an 'enabled' processor");
//  }
  assert(0);
}

bool SIMICS_processor_enabled( int cpuNumber ) {
//   return SIM_cpu_enabled(SIMICS_get_proc_ptr(cpuNumber));  // Maurice
  assert(0);
  return false;
}

// return NULL when fail
void* SIMICS_get_proc_ptr(int cpuNumber) {
//   return (void *) SIM_proc_no_2_ptr(cpuNumber);  // Maurice
  assert(0);
  return NULL;
}

void SIMICS_print_version(ostream& out) {
//   const char* version = SIM_version();  // Maurice
//  if (version != NULL) {
//     out << "simics_version: " << SIM_version() << endl;  // Maurice
//  }
  out << "Mwa ha ha this is not Simics!!";
}

// KM -- From Nikhil's SN code
//these functions should be in interface.C ??

uinteger_t SIMICS_read_control_register(int cpuNumber, int registerNumber)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   uinteger_t result = SIM_read_register(cpu, registerNumber);  // Maurice
//  return result;
  assert(0);
  return 0;
}

uinteger_t SIMICS_read_window_register(int cpuNumber, int window, int registerNumber)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   uinteger_t result = SIM_read_register(cpu, registerNumber);  // Maurice
//  return result;
  assert(0);
  return 0;
}

uinteger_t SIMICS_read_global_register(int cpuNumber, int globals, int registerNumber)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   uinteger_t result = SIM_read_register(cpu, registerNumber);  // Maurice
//  return result;
  assert(0);
  return 0;
}

/**
   uint64 SIMICS_read_fp_register_x(int cpuNumber, int registerNumber)
   {
//    processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//    return SIM_read_fp_register_x(cpu, registerNumber);  // Maurice
   }
**/

void SIMICS_write_control_register(int cpuNumber, int registerNumber, uinteger_t value)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_write_register(cpu, registerNumber, value);  // Maurice
  assert(0);
}

void SIMICS_write_window_register(int cpuNumber, int window, int registerNumber, uinteger_t value)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_write_register(cpu, registerNumber, value);  // Maurice
  assert(0);
}

void SIMICS_write_global_register(int cpuNumber, int globals, int registerNumber, uinteger_t value)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_write_register(cpu, registerNumber, value);  // Maurice
  assert(0);
}

/***
    void SIMICS_write_fp_register_x(int cpuNumber, int registerNumber, uint64 value)
    {
//     processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//     SIM_write_fp_register_x(cpu, registerNumber, value);  // Maurice
    }
***/

// KM -- Functions using new APIs (update from Nikhil's original)

int SIMICS_get_register_number(int cpuNumber, const char * reg_name){
//   int result = SIM_get_register_number(SIM_proc_no_2_ptr(cpuNumber), reg_name);  // Maurice
//  return result;
  assert(0);
  return 0;
}

const char * SIMICS_get_register_name(int cpuNumber, int reg_num){
//   const char * result = SIM_get_register_name(SIM_proc_no_2_ptr(cpuNumber), reg_num);  // Maurice
//  return result;
  assert(0);
  return "Then we shall fight in the shade";
}

uinteger_t SIMICS_read_register(int cpuNumber, int registerNumber)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   uinteger_t result = SIM_read_register(cpu, registerNumber);  // Maurice
//  return result;
  assert(0);
  return 0;
}

void SIMICS_write_register(int cpuNumber, int registerNumber, uinteger_t value)
{
//   processor_t* cpu = SIM_proc_no_2_ptr(cpuNumber);  // Maurice
//   SIM_write_register(cpu, registerNumber, value);  // Maurice
  assert(0);
}

// This version is called whenever we are about to jump to the SW handler
void ruby_set_pc(void *cpu, void *parameter){
//   physical_address_t paddr;
//   paddr = (physical_address_t) parameter;
//   // Simics' processor number
// //   int proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//   int smt_proc_no = proc_no / RubyConfig::numberofSMTThreads();
// //   SIM_set_program_counter(cpu, paddr);  // Maurice
//   //cout << "ruby_set_pc setting cpu[ " << proc_no << " ] smt_cpu[ " << smt_proc_no << " ] PC[ " << hex << paddr << " ]" << dec << endl;
// //   physical_address_t newpc = SIM_get_program_counter(cpu);  // Maurice
// //   int pc_reg = SIM_get_register_number(cpu, "pc");  // Maurice
// //   int npc_reg = SIM_get_register_number( cpu, "npc");  // Maurice
// //   uinteger_t pc = SIM_read_register(cpu, pc_reg);  // Maurice
// //   uinteger_t npc = SIM_read_register(cpu, npc_reg);  // Maurice
//   //cout << "NEW PC[ 0x" << hex << newpc << " ]" << " PC REG[ 0x" << pc << " ] NPC REG[ 0x" << npc << " ]" << dec << endl;
//
//   if(XACT_MEMORY){
//     if( !OpalInterface::isOpalLoaded() ){
//       // using SimicsDriver
//       ASSERT( proc_no == smt_proc_no );
//       SimicsDriver *simics_intf = dynamic_cast<SimicsDriver*>(g_system_ptr->getDriver());
//       ASSERT( simics_intf );
//       simics_intf->notifyTrapStart( proc_no, Address(paddr), 0 /*dummy threadID*/, 0 /* Simics uses 1 thread */ );
//     }
//     else{
//       // notify Opal about changing pc to SW handler
//       //cout << "informing Opal via notifyTrapStart proc = " << proc_no << endl;
//       //g_system_ptr->getSequencer(smt_proc_no)->notifyTrapStart( proc_no, Address(paddr) );
//     }
//
//   if (XACT_DEBUG && XACT_DEBUG_LEVEL > 1){
//     cout <<  g_eventQueue_ptr->getTime() << " " << proc_no
//        << " ruby_set_pc PC: " << hex
// //        << SIM_get_program_counter(cpu) <<   // Maurice
// //       " NPC is: " << hex << SIM_read_register(cpu, 33) << " pc_val: " << paddr << dec << endl;   // Maurice
//   }
//   }
  assert(0);
}

// This version is called whenever we are about to return from SW handler
void ruby_set_program_counter(void *cpu, void *parameter){
//  physical_address_t paddr;
//  paddr = (physical_address_t) parameter;
//  // Simics' processor number
////   int proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//  // SMT proc number
//  int smt_proc_no = proc_no / RubyConfig::numberofSMTThreads();
//
////   // SIM_set_program_counter() also sets the NPC to PC+4.   // Maurice
//  // Need to ensure that NPC doesn't change especially for PCs in the branch delay slot
////   uinteger_t npc_val = SIM_read_register(cpu, SIM_get_register_number(cpu, "npc"));  // Maurice
////   SIM_set_program_counter(cpu, paddr);  // Maurice
////   SIM_write_register(cpu, SIM_get_register_number(cpu, "npc"), npc_val);  // Maurice
//
//  //LUKE - notify Opal of PC change (ie end of all register updates and abort complete)
//  //          I moved the register checkpoint restoration to here also, to jointly update the PC and the registers at the same time
//  if(XACT_MEMORY){
//    if( !OpalInterface::isOpalLoaded() ){
//      //using SimicsDriver
//      //we should only be running with 1 thread with Simics
//      ASSERT( proc_no == smt_proc_no );
//      SimicsDriver *simics_intf = dynamic_cast<SimicsDriver*>(g_system_ptr->getDriver());
//      ASSERT( simics_intf );
//      simics_intf->notifyTrapComplete(proc_no, Address( paddr ), 0 /* Simics uses 1 thread */ );
//   }
//   else{
//     //using OpalInterface
//     // g_system_ptr->getSequencer(smt_proc_no)->notifyTrapComplete( proc_no, Address(paddr) );
//   }
//  }
//  if (XACT_DEBUG && XACT_DEBUG_LEVEL > 1){
//    cout <<  g_eventQueue_ptr->getTime() << " " << proc_no
//       << " ruby_set_program_counter PC: " << hex
////        << SIM_get_program_counter(cpu) <<   // Maurice
////       " NPC is: " << hex << SIM_read_register(cpu, 33) << " pc_val: " << paddr << " npc_val: " << npc_val << dec << endl;   // Maurice
//  }
  assert(0);
}

void ruby_set_npc(void *cpu, void *parameter){
//   physical_address_t paddr;
//   paddr = (physical_address_t) parameter;
// //   int proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//   // SMT proc number
//   int smt_proc_no = proc_no / RubyConfig::numberofSMTThreads();
//
// //   SIM_write_register(cpu, SIM_get_register_number(cpu, "npc"), paddr);  // Maurice
//   if (XACT_DEBUG && XACT_DEBUG_LEVEL > 1){
//     cout <<  g_eventQueue_ptr->getTime() << " " << proc_no
//          << " ruby_set_npc val: " << hex << paddr << " PC: " << hex
// //        << SIM_get_program_counter(cpu) <<   // Maurice
// //       " NPC is: " << hex << SIM_read_register(cpu, 33) << dec << endl;   // Maurice
//   }
  assert(0);
}

void ruby_continue_execution(void *cpu, void *parameter){
//   int logical_proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//  int thread = logical_proc_no % RubyConfig::numberofSMTThreads();
//  int proc_no = logical_proc_no / RubyConfig::numberofSMTThreads();
//  g_system_ptr->getTransactionInterfaceManager(proc_no)->continueExecutionCallback(thread);
  assert(0);
}

void ruby_restart_transaction(void *cpu, void *parameter){
//   int logical_proc_no = SIM_get_proc_no((processor_t *) cpu);  // Maurice
//  int thread = logical_proc_no % RubyConfig::numberofSMTThreads();
//  int proc_no = logical_proc_no / RubyConfig::numberofSMTThreads();
//  g_system_ptr->getTransactionInterfaceManager(proc_no)->restartTransactionCallback(thread);
  assert(0);
}

void ruby_stall_proc(void *cpu, void *parameter){
//   int logical_proc_no = SIM_get_proc_no((processor_t*)cpu);  // Maurice
//  int cycles          = (uint64)parameter;

//  SIMICS_stall_proc(logical_proc_no, cycles);
  assert(0);
}

void ruby_disable_processor(void *cpu, void *parameter){
//   int logical_proc_no = SIM_get_proc_no((processor_t*)cpu);  // Maurice
//  SIMICS_disable_processor(logical_proc_no);
  assert(0);
}

