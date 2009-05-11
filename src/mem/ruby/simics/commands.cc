
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
 */

#include "protocol_name.hh"
#include "Global.hh"
#include "System.hh"
#include "CacheRecorder.hh"
//#include "Tracer.hh"
#include "RubyConfig.hh"
#include "interface.hh"
#include "Network.hh"
// #include "TransactionInterfaceManager.hh"
// #include "TransactionVersionManager.hh"
// #include "TransactionIsolationManager.hh"
//#include "XactCommitArbiter.hh" // gem5:Arka for decomissioning of log_tm
#include "Chip.hh"
//#include "XactVisualizer.hh" // gem5:Arka for decomissioning of log_tm

extern "C" {
#include "commands.hh"
}

#ifdef CONTIGUOUS_ADDRESSES
#include "ContiguousAddressTranslator.hh"

/* Declared in interface.C */
extern ContiguousAddressTranslator * g_p_ca_translator;

memory_transaction_t local_memory_transaction_t_shadow;

#endif // #ifdef CONTIGUOUS_ADDRESSES

//////////////////////// extern "C" api ////////////////////////////////

extern "C"
void ruby_dump_cache(int cpuNumber)
{
  assert(0);
  g_system_ptr->getChip(cpuNumber/RubyConfig::numberOfProcsPerChip())->dumpCaches(cout);
}

extern "C"
void ruby_dump_cache_data(int cpuNumber, char* tag)
{
  assert(0);
  if (tag == NULL) {
    // No filename, dump to screen
    g_system_ptr->printConfig(cout);
    g_system_ptr->getChip(cpuNumber/RubyConfig::numberOfProcsPerChip())->dumpCacheData(cout);
  } else {
    // File name, dump to file
    string filename(tag);

    cout << "Dumping stats to output file '" << filename << "'..." << endl;
    ofstream m_outputFile;
    m_outputFile.open(filename.c_str());
    if(m_outputFile == NULL){
      cout << endl << "Error: error opening output file '" << filename << "'" << endl;
      return;
    }
    g_system_ptr->getChip(cpuNumber/RubyConfig::numberOfProcsPerChip())->dumpCacheData(m_outputFile);
  }
}

extern "C"
void ruby_set_periodic_stats_file(char* filename)
{
  assert(0);
  g_system_ptr->getProfiler()->setPeriodicStatsFile(filename);
}

extern "C"
void ruby_set_periodic_stats_interval(int interval)
{
  assert(0);
  g_system_ptr->getProfiler()->setPeriodicStatsInterval(interval);
}

extern "C"
int mh_memorytracer_possible_cache_miss(memory_transaction_t *mem_trans)
{

  assert(0);
  memory_transaction_t *p_mem_trans_shadow = mem_trans;

#ifdef CONTIGUOUS_ADDRESSES
  if(g_p_ca_translator!=NULL) {
    memcpy( &local_memory_transaction_t_shadow, mem_trans, sizeof(memory_transaction_t) );
    p_mem_trans_shadow = &local_memory_transaction_t_shadow;
    uint64 contiguous_address = g_p_ca_translator->TranslateSimicsToRuby( p_mem_trans_shadow->s.physical_address );
    p_mem_trans_shadow->s.physical_address = contiguous_address;
  }
#endif // #ifdef CONTIGUOUS_ADDRESSES


  // Pass this request off to SimicsDriver::makeRequest()
  // SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
  // return simics_interface_ptr->makeRequest(p_mem_trans_shadow);
  return 0;
}

extern "C"
void mh_memorytracer_observe_memory(memory_transaction_t *mem_trans)
{

  assert(0);
  memory_transaction_t *p_mem_trans_shadow = mem_trans;


#ifdef CONTIGUOUS_ADDRESSES
  if(g_p_ca_translator!=NULL) {
    memcpy( &local_memory_transaction_t_shadow, mem_trans, sizeof(memory_transaction_t) );
    p_mem_trans_shadow = &local_memory_transaction_t_shadow;
    uint64 contiguous_address = g_p_ca_translator->TranslateSimicsToRuby( p_mem_trans_shadow->s.physical_address );
    p_mem_trans_shadow->s.physical_address = contiguous_address;

 }
#endif // #ifdef CONTIGUOUS_ADDRESSES


  // Pass this request off to SimicsDriver::makeRequest()
  //SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
  //simics_interface_ptr->observeMemoryAccess(p_mem_trans_shadow);
}


void ruby_set_g3_reg(void *cpu, void *parameter){
  assert(0);
#if 0
  int proc_num = SIM_get_proc_no(cpu);
  sparc_v9_interface_t * m_v9_interface = (sparc_v9_interface_t *) SIM_get_interface(cpu, SPARC_V9_INTERFACE);

  for(int set=0; set < 4; set++) {
    for(int i=0; i <8; i++) {
      int registerNumber = i;
      uinteger_t value = m_v9_interface->read_global_register((void *)cpu, set, registerNumber);
      cout << "ruby_set_g3_reg BEFORE: proc =" << proc_num << " GSET = " << set << " GLOBAL_REG = " << i << " VALUE = " << value << endl;
    }
  }

  uinteger_t value_ptr = (uinteger_t) parameter;
  int g3_regnum = SIM_get_register_number(cpu, "g3");
  SIM_write_register(cpu, g3_regnum, (uinteger_t) value_ptr);

  cout << endl;
  for(int set=0; set < 4; set++) {
    for(int i=0; i <8; i++) {
      int registerNumber = i;
      uinteger_t value = m_v9_interface->read_global_register((void *)cpu, set, registerNumber);
      cout << "ruby_set_g3_reg AFTER: proc =" << proc_num << " GSET = " << set << " GLOBAL_REG = " << i << " VALUE = " << value << endl;
    }
  }
#endif

}

// #define XACT_MGR g_system_ptr->getChip(SIMICS_current_processor_number()/RubyConfig::numberOfProcsPerChip()/RubyConfig::numberofSMTThreads())->getTransactionInterfaceManager( (SIMICS_current_processor_number()/RubyConfig::numberofSMTThreads())%RubyConfig::numberOfProcsPerChip())

extern "C"
void magic_instruction_callback(void* desc, void* cpu, integer_t val)
{
  assert(0);
#if 0
  // Use magic callbacks to start and end transactions w/o opal
  if (val > 0x10000) // older magic call numbers. Need to be right-shifted.
    val = val >> 16;
  int id = -1;
  int proc_num = SIMICS_current_processor_number();
  int sim_proc_num = proc_num / RubyConfig::numberofSMTThreads();
  int thread_num = proc_num % RubyConfig::numberofSMTThreads();
  int ruby_cycle = g_eventQueue_ptr->getTime();

  if(proc_num < 0){
    cout << "ERROR proc_num= " << proc_num << endl;
  }
  assert(proc_num >= 0);
  if(thread_num < 0){
    cout << "ERROR thread_num= " << thread_num << endl;
  }
  assert(thread_num >= 0);
  if( sim_proc_num < 0){
    cout << "ERROR sim_proc_num = " << sim_proc_num << endl;
  }
  assert(sim_proc_num >= 0);

  if (val == 3) {
    g_system_ptr->getProfiler()->startTransaction(sim_proc_num);
  } else if (val == 4) {
    ; // magic breakpoint
  } else if (val == 5) {
    g_system_ptr->getProfiler()->endTransaction(sim_proc_num);
  } else if (val == 6){ // Begin Exposed Action
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Begin exposed action for thread " << thread_num << " of proc " << proc_num << " PC " << SIMICS_get_program_counter(proc_num) << endl;
    XACT_MGR->beginEscapeAction(thread_num);
  } else if (val == 7){ // Begin Exposed Action
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "End exposed action for thread " << thread_num << " of proc " << proc_num << " PC " << SIMICS_get_program_counter(proc_num) << endl;
    XACT_MGR->endEscapeAction(thread_num);
  } else if (val == 8) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Set log Base Address for thread " << thread_num << " of proc " << proc_num << endl;
    XACT_MGR->setLogBase(thread_num);
  } else if (val == 9) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Setting Handler Address for thread " << thread_num << " of proc " << proc_num << endl;
    XACT_MGR->setHandlerAddress(thread_num);
  } else if (val == 10) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Release Isolation for thread " << thread_num << " of proc " << proc_num << endl;
    XACT_MGR->releaseIsolation(thread_num);
  } else if (val == 11) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Restart transaction for thread " << thread_num << " of proc " << proc_num << endl;
    XACT_MGR->restartTransaction(thread_num);
  } else if (val == 12) {
    // NOTE: this is a functional magic call for the Java VM
    //   It is used by mfacet.py to check whether to use TM macros or JVM locking
    return;
  } else if (val == 13) {
    // NOTE: this is a debug magic call for the Java VM
    // Indicates BEGIN XACT
    return;
  } else if (val == 14) {
    // NOTE: this is a debug magic call for the Java VM
    // Indicates COMMIT_XACT
    return;
  } else if (val == 15) {
    cout << "SIMICS SEG FAULT  for thread " << thread_num << " of proc " << proc_num << endl;
    SIM_break_simulation("SIMICS SEG FAULT");
    return;
  } else if (val == 16) {
    // NOTE : this is a debug magic call for the Java VM
    // Indicates LOCKING object
    return;
  } else if (val == 17) {
    // NOTE : this is a debug magic call for the Java VM
    // Indicates UNLOCKING object
    return;
  } else if (val == 18) {
    // NOTE: this is a magic call to enable the xact mem macros in the Java VM
    //  The functionality is implemented in gen-scripts/mfacet.py because it can be independent of Ruby
    return;
  } else if (val == 19){
    cout << "RUBY WATCH: " << endl;
    g_system_ptr->getProfiler()->rubyWatch(SIMICS_current_processor_number());
  } else if (val == 20) {
    //XACT_MGR->setJavaPtrs(thread_num);
  } else if (val == 21){
    // NOTE : this is a debug magic call used to dump the registers for a processor
    //   Functionality is implemented in gen-scripts/mfacet.py because it can be independent of Ruby
    return;
  } else if (val == 23){
    // register compensating action
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " REGISTER COMPENSATING ACTION " << endl;
    XACT_MGR->registerCompensatingAction(thread_num);
  } else if (val == 24){
    // register commit action
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " REGISTER COMMIT ACTION " << endl;
    XACT_MGR->registerCommitAction(thread_num);
  } else if (val == 27){
    // xmalloc
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " XMALLOC " << endl;
    XACT_MGR->xmalloc(thread_num);
  } else if (val == 29){
    // Begin Barrier
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " BEGIN BARRIER " << endl;
    g_system_ptr->getXactVisualizer()->moveToBarrier(proc_num);
  } else if (val == 30){
    // End Barrier
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " END BARRIER " << endl;
    g_system_ptr->getXactVisualizer()->moveToNonXact(proc_num);
  } else if (val == 28) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Continue execution for thread " << thread_num << " of proc " << proc_num << endl;
    XACT_MGR->continueExecution(thread_num);
  } else if (val == 31){
    // Begin Timer
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " BEGIN TIMER " << endl;
    g_system_ptr->getProfiler()->getXactProfiler()->profileBeginTimer(proc_num);
  } else if (val == 32){
    // End Timer
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << proc_num << "," << thread_num << " END TIMER " << endl;
    g_system_ptr->getProfiler()->getXactProfiler()->profileEndTimer(proc_num);
  } else if (val == 40) {
                // register a thread for virtualization
    if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
      SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
      simics_interface_ptr->getHypervisor()->registerThreadWithHypervisor(proc_num);
    }
  } else if (val == 41) {
                // get information about the last summary conflict
                if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
        Address addr = XACT_MGR->getXactIsolationManager()->getSummaryConflictAddress();
        unsigned int conflictAddress = addr.getAddress();
        unsigned int conflictType = XACT_MGR->getXactIsolationManager()->getSummaryConflictType();
            SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "g2"), conflictAddress);
            SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "g3"), conflictType);
                }
  } else if (val == 42) {
                // resolve summary conflict magic callback
                if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
                        SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
                        simics_interface_ptr->getHypervisor()->resolveSummarySignatureConflict(proc_num);
                }
  } else if (val == 50) {
    // set summary signature bit
    int index = SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2"));
    XACT_MGR->writeBitSummaryWriteSetFilter(thread_num, index, 1);
  } else if (val == 51) {
    // unset summary signature bit
    int index = SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2"));
    XACT_MGR->writeBitSummaryWriteSetFilter(thread_num, index, 0);
  } else if (val == 52) {
    // add address in summary signature
    Address addr = Address(SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2")));
    cout << "Add to summary write set filter: " << addr << endl;
    XACT_MGR->addToSummaryWriteSetFilter(thread_num, addr);
  } else if (val == 53) {
    // remove address from summary signature
    Address addr = Address(SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2")));
    XACT_MGR->removeFromSummaryWriteSetFilter(thread_num, addr);
  } else if (val == 54) {
    // translate address to summary signature index
    Address addr = Address(SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2")));
    SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "g3"), XACT_MGR->getIndexSummaryFilter(thread_num, addr));
  } else if (val == 55) {
    XACT_MGR->setIgnoreWatchpointFlag(thread_num, true);
  } else if (val == 56) {
    g_system_ptr->getProfiler()->watchpointsFalsePositiveTrigger();
  } else if (val == 57) {
    g_system_ptr->getProfiler()->watchpointsTrueTrigger();
  } else if (val == 60) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2) {
      cout << "Set restorePC for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    }
    unsigned int pc = SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "g2"));
    XACT_MGR->setRestorePC(thread_num, pc);
  } else if (val == 61) {
    // get log size
    SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "g2"), XACT_MGR->getXactVersionManager()->getLogSize(thread_num));
  } else if (val == 62) {
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2){
      cout << " GET THREAD ID " << thread_num << " of proc " << proc_num << " TID " << XACT_MGR->getTID(thread_num) << endl;
    }
    // get thread id
    SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "g2"), XACT_MGR->getTID(thread_num));
  } else if (val == 100) {
    dump_registers((void*)cpu);
  } else if (val >= 1024 && val < 2048) {
    // begin closed
    id = val - 1024;
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Begin CLOSED transaction for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    XACT_MGR->beginTransaction(thread_num, id, false);
    //}  else if (val >= min_closed_commit && val < XACT_OPEN_MIN_ID) {
  }  else if (val >= 2048 && val < 3072) {
    // commit closed
    id = val - 2048;
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Commit CLOSED transaction for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    XACT_MGR->commitTransaction(thread_num, id, false);
  } else if (val >= 3072 && val < 4096) {
    // begin open
    id = val - 3072;
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "Begin OPEN transaction for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    XACT_MGR->beginTransaction(thread_num, id, true);
  } else if (val >= 4096 && val < 5120) {
    // commit open
    id = val - 4096;
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "COMMIT OPEN transaction for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    XACT_MGR->commitTransaction(thread_num, id, true);
  } else if (val >= 5120 && val < 6144){

    cout << " SYSCALL " << val - 5120 << " of proc " << proc_num << " " << thread_num << " time = " << ruby_cycle << endl;
  } else if (val >= 6144 && val < 7168) {
    // commit open
    id = val - 6144;
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2)
      cout << "ABORT transaction for thread " << thread_num << " of proc " << proc_num << " XID " << id << endl;
    XACT_MGR->abortTransaction(thread_num, id);
   } else if (val == 8000) {
    // transaction level
    if (XACT_DEBUG && XACT_DEBUG_LEVEL > 2) {
      id = val - 8000;
      cout << "Transaction Level for thread " << thread_num << " of proc " << proc_num << " XID " << id << " : "
            << XACT_MGR->getTransactionLevel(thread_num)<< endl;
    }
    SIMICS_write_register(proc_num, SIMICS_get_register_number(proc_num, "i0"),(unsigned int) XACT_MGR->getTransactionLevel(thread_num));
  } else if (val==8001) {
    cout << " " << g_eventQueue_ptr->getTime() << " " << dec << proc_num << " [" << proc_num << "," << thread_num << " ]"
                                        << " TID " << XACT_MGR->getTID(0)
          << " DEBUGMSG " << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i0")) << "  "
          << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i1")) << " "
          << "(0x" << hex << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i1")) << ") "
          << dec << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i2")) << " "
          << "(0x" << hex << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i2")) << ")" << dec
                                        << " break = " << SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i3")) << endl;
                if (SIMICS_read_register(proc_num, SIMICS_get_register_number(proc_num, "i3")) == 1) {
        SIM_break_simulation("DEBUGMSG");
                }
  } else {
    WARN_EXPR(val);
    WARN_EXPR(SIMICS_get_program_counter(proc_num));
    WARN_MSG("Unexpected magic call");
  }
#endif
}

/* -- Handle command to change the debugging verbosity for Ruby */
extern "C"
void ruby_change_debug_verbosity(char* new_verbosity_str)
{
  assert(0);
  g_debug_ptr->setVerbosityString(new_verbosity_str);
}

/* -- Handle command to change the debugging filter for Ruby */
extern "C"
void ruby_change_debug_filter(char* new_filter_str)
{
  assert(0);
  g_debug_ptr->setFilterString(new_filter_str);
}

/* -- Handle command to set the debugging output file for Ruby */
extern "C"
void ruby_set_debug_output_file (const char * new_filename)
{
  assert(0);
  string filename(new_filename);

  filename += "-";
  filename += CURRENT_PROTOCOL;
  // get the date and time to label the debugging file
  const time_t T = time(NULL);
  tm *localTime = localtime(&T);
  char buf[100];
  strftime(buf, 100, ".%b%d.%Y-%H.%M.%S", localTime);

  filename += buf;
  filename += ".debug";

  cout << "Dumping debugging output to file '" << filename << "'...";
  g_debug_ptr->setDebugOutputFile (filename.c_str());
}

extern "C"
void ruby_set_debug_start_time(char* start_time_str)
{
  assert(0);
  int startTime = atoi(start_time_str);
  g_debug_ptr->setDebugTime(startTime);
}

/* -- Clear stats */
extern "C"
void ruby_clear_stats()
{
  assert(0);
  cout << "Clearing stats...";
  fflush(stdout);
  g_system_ptr->clearStats();
  cout << "Done." << endl;
}

/* -- Dump stats */
extern "C"
// File name, dump to file
void ruby_dump_stats(char* filename)
{
  assert(0);
  /*g_debug_ptr->closeDebugOutputFile();*/
  if (filename == NULL) {
    // No output file, dump to screen
    cout << "Dumping stats to standard output..." << endl;
    g_system_ptr->printConfig(cout);
    g_system_ptr->printStats(cout);
  } else {
    cout << "Dumping stats to output file '" << filename << "'..." << endl;
    ofstream m_outputFile;
    m_outputFile.open(filename);
    if(m_outputFile == NULL) {
      cout << "Error: error opening output file '" << filename << "'" << endl;
      return;
    }
    g_system_ptr->printConfig(m_outputFile);
    g_system_ptr->printStats(m_outputFile);
  }
  cout << "Dumping stats completed." << endl;
}

/* -- Dump stats */
extern "C"
// File name, dump to file
void ruby_dump_short_stats(char* filename)
{
  assert(0);
  g_debug_ptr->closeDebugOutputFile();
  if (filename == NULL) {
    // No output file, dump to screen
    //cout << "Dumping short stats to standard output..." << endl;
    //g_system_ptr->printConfig(cout);
    g_system_ptr->getProfiler()->printStats(cout, true);
  } else {
    cout << "Dumping stats to output file '" << filename << "'..." << endl;
    ofstream m_outputFile;
    m_outputFile.open(filename);
    if(m_outputFile == NULL) {
      cout << "Error: error opening output file '" << filename << "'" << endl;
      return;
    }
    g_system_ptr->getProfiler()->printShortStats(m_outputFile);
    cout << "Dumping stats completed." << endl;
  }
}

extern "C"
void ruby_load_caches(char* name)
{
  assert(0);
  if (name == NULL) {
    cout << "Error: ruby_load_caches requires a file name" << endl;
    return;
  }

  cout << "Reading cache contents from '" << name << "'...";
 /*  gem5:Binkert for decomissiong of tracer
  int read = Tracer::playbackTrace(name);
  cout << "done. (" << read << " cache lines read)" << endl;
 */
  cout << "done. (TRACER DISABLED!)" << endl;
  ruby_clear_stats();
}

extern "C"
void ruby_save_caches(char* name)
{
  assert(0);
  if (name == NULL) {
    cout << "Error: ruby_save_caches requires a file name" << endl;
    return;
  }

  cout << "Writing cache contents to '" << name << "'...";
  CacheRecorder recorder;
  g_system_ptr->recordCacheContents(recorder);
  int written = recorder.dumpRecords(name);
  cout << "done. (" << written << " cache lines written)" << endl;
}

extern "C"
void ruby_set_tracer_output_file (const char * new_filename)
{
  assert(0);
  //g_system_ptr->getTracer()->startTrace(string(new_filename));
}

/* -- Handle command to set the xact visualizer file for Ruby */
extern "C"
void ruby_xact_visualizer_file (char * new_filename)
{
  cout << "Dumping xact visualizer output to file '" << new_filename << "'...";
  //  g_system_ptr->getProfiler()->setXactVisualizerFile (new_filename);
}

extern "C"
void ctrl_exception_start(void* desc, void* cpu, integer_t val)
{
#if 0
  int proc_no = SIM_get_proc_no((void*) cpu);
  void* cpu_obj = (void*) cpu;
  uinteger_t trap_level = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tl"));

  if (!XACT_MEMORY) return;
  TransactionInterfaceManager *xact_mgr = XACT_MGR;

  // level {10,14} interrupt
  //
  if (val == 0x4a || val == 0x4e) {
    int rn_tick           = SIM_get_register_number(cpu_obj, "tick");
    uinteger_t tick       = SIM_read_register(cpu_obj, rn_tick);
    int rn_tick_cmpr      = SIM_get_register_number(cpu_obj, "tick_cmpr");
    uinteger_t tick_cmpr  = SIM_read_register(cpu_obj, rn_tick_cmpr);
    int rn_stick          = SIM_get_register_number(cpu_obj, "stick");
    uinteger_t stick      = SIM_read_register(cpu_obj, rn_stick);
    int rn_stick_cmpr     = SIM_get_register_number(cpu_obj, "stick_cmpr");
    uinteger_t stick_cmpr = SIM_read_register(cpu_obj, rn_stick_cmpr);
    int rn_pc             = SIM_get_register_number(cpu_obj, "pc");
    uinteger_t pc = SIM_read_register(cpu_obj, rn_pc);
    int rn_npc            =  SIM_get_register_number(cpu_obj, "npc");
    uinteger_t npc = SIM_read_register(cpu_obj, rn_npc);
    int rn_pstate         = SIM_get_register_number(cpu_obj, "pstate");
    uinteger_t pstate     = SIM_read_register(cpu_obj, rn_pstate);
    int rn_pil            = SIM_get_register_number(cpu_obj, "pil");
    int pil               = SIM_read_register(cpu_obj, rn_pil);
    g_system_ptr->getProfiler()->profileTimerInterrupt(proc_no,
                                                       tick, tick_cmpr,
                                                       stick, stick_cmpr,
                                                       trap_level,
                                                       pc, npc,
                                                       pstate, pil);
  }

  int smt_thread_num = proc_no % RubyConfig::numberofSMTThreads();
  // The simulated processor number
  int sim_proc_no = proc_no / RubyConfig::numberofSMTThreads();

  uinteger_t pc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "pc"));
  uinteger_t npc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "npc"));

  g_system_ptr->getProfiler()->profileExceptionStart(xact_mgr->getTransactionLevel(smt_thread_num) > 0, sim_proc_no, smt_thread_num, val, trap_level, pc, npc);

  if((val >= 0x80 && val <= 0x9f) || (val >= 0xc0 && val <= 0xdf)){
      //xact_mgr->setLoggedException(smt_thread_num);
  }
  // CORNER CASE - You take an exception while stalling for a commit token
  if (XACT_LAZY_VM && !XACT_EAGER_CD){
    if (g_system_ptr->getXactCommitArbiter()->getTokenOwner() == proc_no)
      g_system_ptr->getXactCommitArbiter()->releaseCommitToken(proc_no);
  }
#endif
  assert(0);
}

extern "C"
void ctrl_exception_done(void* desc, void* cpu, integer_t val)
{
  assert(0);
#if 0
  int proc_no = SIM_get_proc_no((void*) cpu);
  void* cpu_obj = (void*) cpu;
  uinteger_t trap_level = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tl"));
  uinteger_t pc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "pc"));
  uinteger_t npc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "npc"));
  uinteger_t tpc = 0;
  uinteger_t tnpc = 0;
  //get the return PC,NPC pair based on the trap level
  ASSERT(1 <= trap_level && trap_level <= 5);
  if(trap_level == 1){
    tpc =  SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tpc1"));
    tnpc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tnpc1"));
  }
  if(trap_level == 2){
    tpc =  SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tpc2"));
    tnpc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tnpc2"));
  }
  if(trap_level == 3){
    tpc =  SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tpc3"));
    tnpc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tnpc3"));
  }
  if(trap_level == 4){
    tpc =  SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tpc4"));
    tnpc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tnpc4"));
  }
  if(trap_level == 5){
    tpc =  SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tpc5"));
    tnpc = SIM_read_register(cpu_obj, SIM_get_register_number(cpu_obj, "tnpc5"));
  }

  if (!XACT_MEMORY) return;
  TransactionInterfaceManager *xact_mgr = XACT_MGR;

  int smt_thread_num = proc_no % RubyConfig::numberofSMTThreads();
  // The simulated processor number
  int sim_proc_no = proc_no / RubyConfig::numberofSMTThreads();

  if (proc_no != SIMICS_current_processor_number()){
    WARN_EXPR(proc_no);
    WARN_EXPR(SIMICS_current_processor_number());
    WARN_MSG("Callback for a different processor");
  }

  g_system_ptr->getProfiler()->profileExceptionDone(xact_mgr->getTransactionLevel(smt_thread_num) > 0, sim_proc_no, smt_thread_num, val, trap_level, pc, npc, tpc, tnpc);

  if((val >= 0x80 && val <= 0x9f) || (val >= 0xc0 && val <= 0xdf)){
    //xact_mgr->clearLoggedException(smt_thread_num);
  }

  if ((val == 0x122) && xact_mgr->shouldTrap(smt_thread_num)){
    // use software handler
    if (xact_mgr->shouldUseHardwareAbort(smt_thread_num)){
        xact_mgr->hardwareAbort(smt_thread_num);
    } else {
      xact_mgr->trapToHandler(smt_thread_num);
    }
  }
#endif
}

extern "C"
void change_mode_callback(void* desc, void* cpu, integer_t old_mode, integer_t new_mode)
{
  assert(0);
#if 0
  if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
     SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
     simics_interface_ptr->getHypervisor()->change_mode_callback(desc, cpu, old_mode, new_mode);
  }
#endif
}

extern "C"
void dtlb_map_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg){
  assert(0);
#if 0
  if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
     SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
     simics_interface_ptr->getHypervisor()->dtlb_map_callback(desc, chmmu, tag_reg, data_reg);
  }
#endif
}

extern "C"
void dtlb_demap_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg){
  assert(0);
#if 0
  if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
     SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
     simics_interface_ptr->getHypervisor()->dtlb_demap_callback(desc, chmmu, tag_reg, data_reg);
  }
#endif
}

extern "C"
void dtlb_replace_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg){
  assert(0);
#if 0
  if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
     SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
     simics_interface_ptr->getHypervisor()->dtlb_replace_callback(desc, chmmu, tag_reg, data_reg);
  }
#endif
}

extern "C"
void dtlb_overwrite_callback(void* desc, void* chmmu, integer_t tag_reg, integer_t data_reg){
  assert(0);
#if 0
  if (XACT_ENABLE_VIRTUALIZATION_LOGTM_SE) {
     SimicsDriver* simics_interface_ptr = static_cast<SimicsDriver*>(g_system_ptr->getDriver());
     simics_interface_ptr->getHypervisor()->dtlb_overwrite_callback(desc, chmmu, tag_reg, data_reg);
  }
#endif
}

extern "C"
void core_control_register_write_callback(void* desc, void* cpu, integer_t register_number, integer_t value) {
  assert(0);
#if 0
  int proc_no = SIM_get_proc_no((void*) cpu);
  void* cpu_obj = (void*) cpu;
#endif
}

integer_t
read_reg(void *cpu, const char* reg_name)
{
  assert(0);
#if 0
  int reg_num = SIM_get_register_number(SIM_current_processor(), reg_name);
  if (SIM_clear_exception()) {
    fprintf(stderr, "read_reg: SIM_get_register_number(%s, %s) failed!\n",
            cpu->name, reg_name);
    assert(0);
  }
  integer_t val = SIM_read_register(cpu, reg_num);
  if (SIM_clear_exception()) {
    fprintf(stderr, "read_reg: SIM_read_register(%s, %d) failed!\n",
            cpu->name, reg_num);
    assert(0);
  }
  return val;
#endif
  return 0;
}

extern "C"
void dump_registers(void *cpu)
{
  assert(0);
#if 0
  const char* reg_names[] = {
    "g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7",
    "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7",
    "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
    "o0", "o1", "o2", "o3", "o4", "o5", "o6", "o7",
    "ccr", "pc", "npc"
  };

  printf("Registers for %s\n", cpu->name);
  printf("------------------\n");

  for (int i = 0; i < (sizeof(reg_names) / sizeof(char*)); i++) {
    const char* reg_name = reg_names[i];
    printf(" %3s: 0x%016llx\n", reg_name, read_reg(cpu, reg_name));
    if (i % 8 == 7) {
      printf("\n");
    }
  }

  int myID = SIMICS_get_proc_no(cpu);
  Address myPC = SIMICS_get_program_counter(myID);
  physical_address_t myPhysPC = SIMICS_translate_address(myID, myPC);
  integer_t myInst = SIMICS_read_physical_memory(myID, myPhysPC, 4);
  const char *myInstStr = SIMICS_disassemble_physical(myID, myPhysPC);
  printf("\n *pc: 0x%llx: %s\n", myInst, myInstStr);

  printf("\n\n");
#endif
}
