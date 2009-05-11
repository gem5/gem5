//
// This file has been modified by Kevin Moore and Dan Nussbaum of the
// Scalable Systems Research Group at Sun Microsystems Laboratories
// (http://research.sun.com/scalable/) to support the Adaptive
// Transactional Memory Test Platform (ATMTP).  For information about
// ATMTP, see the GEMS website: http://www.cs.wisc.edu/gems/.
//
// Please send email to atmtp-interest@sun.com with feedback, questions, or
// to request future announcements about ATMTP.
//
// ----------------------------------------------------------------------
//
// File modification date: 2008-02-23
//
// ----------------------------------------------------------------------
//
// ATMTP is distributed as part of the GEMS software toolset and is
// available for use and modification under the terms of version 2 of the
// GNU General Public License.  The GNU General Public License is contained
// in the file $GEMS/LICENSE.
//
// Multifacet GEMS is free software; you can redistribute it and/or modify
// it under the terms of version 2 of the GNU General Public License as
// published by the Free Software Foundation.
//
// Multifacet GEMS is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with the Multifacet GEMS; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
//
// ----------------------------------------------------------------------
//

// see rubyconfig.defaults for some explanations

PARAM( g_RANDOM_SEED );

// Maximum number of cycles a request is can be outstanding before the
// Sequencer of StoreBuffer declares we're in deadlock/livelock
PARAM( g_DEADLOCK_THRESHOLD );
PARAM_BOOL(  RANDOMIZATION );
PARAM_BOOL(  g_SYNTHETIC_DRIVER );
PARAM_BOOL(  g_DETERMINISTIC_DRIVER );

// FOR MOESI_CMP_token
PARAM_BOOL(  g_FILTERING_ENABLED );
PARAM_BOOL(  g_DISTRIBUTED_PERSISTENT_ENABLED );
PARAM_BOOL(  g_DYNAMIC_TIMEOUT_ENABLED );
PARAM( g_RETRY_THRESHOLD );
PARAM( g_FIXED_TIMEOUT_LATENCY );

PARAM(  g_trace_warmup_length );
PARAM_DOUBLE( g_bash_bandwidth_adaptive_threshold );

PARAM( g_tester_length );
PARAM( g_synthetic_locks );
PARAM( g_deterministic_addrs );
// Specified Generator: See SpecifiedGeneratorType in external.sm for valid values
PARAM_STRING( g_SpecifiedGenerator );
PARAM( g_callback_counter );
PARAM( g_NUM_COMPLETIONS_BEFORE_PASS );

PARAM( g_NUM_SMT_THREADS );

PARAM( g_think_time );
PARAM( g_hold_time );
PARAM( g_wait_time );

// For debugging purposes, one can enable a trace of all the protocol
// state machine changes. Unfortunately, the code to generate the
// trace is protocol specific. To enable the code for some of the
// standard protocols,
//   1. change "PROTOCOL_DEBUG_TRACE = true"
//   2. enable debug in Makefile
//   3. use the "--start 1" command line parameter or
//      "g_debug_ptr->setDebugTime(1)" to beging the following to set the
//      debug begin time
//
// this use to be ruby/common/Global.h

PARAM_BOOL(  PROTOCOL_DEBUG_TRACE );
// a string for filtering debugging output (for all g_debug vars see Debug.h)
PARAM_STRING( DEBUG_FILTER_STRING );
// filters debugging messages based on priority (low, med, high)
PARAM_STRING( DEBUG_VERBOSITY_STRING );
// filters debugging messages based on a ruby time
PARAM_ULONG( DEBUG_START_TIME );
// sends debugging messages to a output filename
PARAM_STRING( DEBUG_OUTPUT_FILENAME );

// defines relative (integer) clock multipliers between ruby, opal, and simics
PARAM( SIMICS_RUBY_MULTIPLIER );
PARAM( OPAL_RUBY_MULTIPLIER );

PARAM_BOOL( TRANSACTION_TRACE_ENABLED );
PARAM_BOOL( USER_MODE_DATA_ONLY );
PARAM_BOOL( PROFILE_HOT_LINES );

// PROFILE_ALL_INSTRUCTIONS is used if you want Ruby to profile all instructions executed
// The following need to be true for this to work correctly:
// 1. Disable istc and dstc for this simulation run
// 2. Add the following line to the object "sim" in the checkpoint you run from:
// 	instruction_profile_line_size: 4
// This is used to have simics report back all instruction requests

// For more details on how to find out how to interpret the output physical instruction
// address, please read the document in the simics-howto directory
PARAM_BOOL( PROFILE_ALL_INSTRUCTIONS );

// Set the following variable to true if you want a complete trace of
// PCs (physical address of program counters, with executing processor IDs)
// to be printed to stdout. Make sure to direct the simics output to a file.
// Otherwise, the run will take a really long time!
// A long run may write a file that can exceed the OS limit on file length
PARAM_BOOL( PRINT_INSTRUCTION_TRACE );
PARAM( g_DEBUG_CYCLE );

// Don't allow any datablocks to enter the STC
PARAM_BOOL( BLOCK_STC );

// Make the entire memory system perfect
PARAM_BOOL( PERFECT_MEMORY_SYSTEM );
PARAM( PERFECT_MEMORY_SYSTEM_LATENCY );

PARAM_BOOL( DATA_BLOCK ); // Define NO_DATA_BLOCK to make the DataBlock take zero space

PARAM_BOOL( REMOVE_SINGLE_CYCLE_DCACHE_FAST_PATH );

// *********************************************
// CACHE & MEMORY PARAMETERS
// *********************************************

PARAM_BOOL( g_SIMICS );

PARAM( L1_CACHE_ASSOC );
PARAM( L1_CACHE_NUM_SETS_BITS );
PARAM( L2_CACHE_ASSOC );
PARAM( L2_CACHE_NUM_SETS_BITS );

PARAM_ULONG( g_MEMORY_SIZE_BYTES );
PARAM( g_DATA_BLOCK_BYTES );
// The following page size parameter is used by the stride prefetcher
PARAM( g_PAGE_SIZE_BYTES );
PARAM_STRING( g_REPLACEMENT_POLICY );

PARAM( g_NUM_PROCESSORS );
PARAM( g_NUM_L2_BANKS );
PARAM( g_NUM_MEMORIES );
PARAM( g_PROCS_PER_CHIP );

// The following group of parameters are calculated.  They must
// _always_ be left at zero.
PARAM( g_NUM_CHIPS );
PARAM( g_NUM_CHIP_BITS );
PARAM( g_MEMORY_SIZE_BITS );
PARAM( g_DATA_BLOCK_BITS );
PARAM( g_PAGE_SIZE_BITS );
PARAM( g_NUM_PROCESSORS_BITS );
PARAM( g_PROCS_PER_CHIP_BITS );
PARAM( g_NUM_L2_BANKS_BITS );
PARAM( g_NUM_L2_BANKS_PER_CHIP_BITS );
PARAM( g_NUM_L2_BANKS_PER_CHIP );
PARAM( g_NUM_MEMORIES_BITS );
PARAM( g_NUM_MEMORIES_PER_CHIP );
PARAM( g_MEMORY_MODULE_BITS );
PARAM_ULONG( g_MEMORY_MODULE_BLOCKS );

// determines the mapping between L2 banks and sets within L2 banks
PARAM_BOOL( MAP_L2BANKS_TO_LOWEST_BITS );

// TIMING PARAMETERS
PARAM( DIRECTORY_CACHE_LATENCY );

PARAM( NULL_LATENCY );
PARAM( ISSUE_LATENCY );
PARAM( CACHE_RESPONSE_LATENCY );
PARAM( L2_RESPONSE_LATENCY );
PARAM( L2_TAG_LATENCY );
PARAM( L1_RESPONSE_LATENCY );
PARAM( MEMORY_RESPONSE_LATENCY_MINUS_2 );
PARAM( DIRECTORY_LATENCY );
PARAM( NETWORK_LINK_LATENCY );
PARAM( COPY_HEAD_LATENCY );
PARAM( ON_CHIP_LINK_LATENCY );
PARAM( RECYCLE_LATENCY );
PARAM( L2_RECYCLE_LATENCY );
PARAM( TIMER_LATENCY );
PARAM( TBE_RESPONSE_LATENCY );
PARAM_BOOL(  PERIODIC_TIMER_WAKEUPS );

// constants used by TM protocols
PARAM_BOOL( PROFILE_EXCEPTIONS );
PARAM_BOOL( PROFILE_XACT );
PARAM_BOOL( PROFILE_NONXACT );
PARAM_BOOL( XACT_DEBUG );
PARAM ( XACT_DEBUG_LEVEL );
PARAM_BOOL( XACT_MEMORY );
PARAM_BOOL( XACT_ENABLE_TOURMALINE );
PARAM( XACT_NUM_CURRENT );
PARAM( XACT_LAST_UPDATE );
PARAM_BOOL( XACT_ISOLATION_CHECK );
PARAM_BOOL( PERFECT_FILTER );
PARAM_STRING( READ_WRITE_FILTER );
PARAM_BOOL( PERFECT_VIRTUAL_FILTER );
PARAM_STRING( VIRTUAL_READ_WRITE_FILTER );
PARAM_BOOL( PERFECT_SUMMARY_FILTER );
PARAM_STRING( SUMMARY_READ_WRITE_FILTER );
PARAM_BOOL( XACT_EAGER_CD );
PARAM_BOOL( XACT_LAZY_VM );
PARAM_STRING( XACT_CONFLICT_RES );
PARAM_BOOL( XACT_VISUALIZER );
PARAM( XACT_COMMIT_TOKEN_LATENCY ) ;
PARAM_BOOL( XACT_NO_BACKOFF );
PARAM ( XACT_LOG_BUFFER_SIZE );
PARAM ( XACT_STORE_PREDICTOR_HISTORY);
PARAM ( XACT_STORE_PREDICTOR_ENTRIES);
PARAM ( XACT_STORE_PREDICTOR_THRESHOLD);
PARAM ( XACT_FIRST_ACCESS_COST );
PARAM ( XACT_FIRST_PAGE_ACCESS_COST );
PARAM_BOOL( ENABLE_MAGIC_WAITING );
PARAM_BOOL( ENABLE_WATCHPOINT );
PARAM_BOOL( XACT_ENABLE_VIRTUALIZATION_LOGTM_SE );

// ATMTP
PARAM_BOOL( ATMTP_ENABLED );
PARAM_BOOL( ATMTP_ABORT_ON_NON_XACT_INST );
PARAM_BOOL( ATMTP_ALLOW_SAVE_RESTORE_IN_XACT );
PARAM( ATMTP_XACT_MAX_STORES );
PARAM( ATMTP_DEBUG_LEVEL );

// constants used by CMP protocols
PARAM( L1_REQUEST_LATENCY );
PARAM( L2_REQUEST_LATENCY );
PARAM_BOOL( SINGLE_ACCESS_L2_BANKS ); // hack to simulate multi-cycle L2 bank accesses

// Ruby cycles between when a sequencer issues a miss it arrives at
// the L1 cache controller
PARAM( SEQUENCER_TO_CONTROLLER_LATENCY );

// Number of transitions each controller state machines can complete per cycle
PARAM( L1CACHE_TRANSITIONS_PER_RUBY_CYCLE );
PARAM( L2CACHE_TRANSITIONS_PER_RUBY_CYCLE );
PARAM( DIRECTORY_TRANSITIONS_PER_RUBY_CYCLE );

// Maximum number of requests (including prefetches) outstanding from
// the sequencer (Note: this also include items buffered in the store
// buffer)
PARAM( g_SEQUENCER_OUTSTANDING_REQUESTS );

// Number of TBEs available for demand misses, prefetches, and replacements
PARAM( NUMBER_OF_TBES );
PARAM( NUMBER_OF_L1_TBES );
PARAM( NUMBER_OF_L2_TBES );

// NOTE: Finite buffering allows us to simulate a wormhole routed network
// with idealized flow control.  All message buffers within the network (i.e.
// the switch's input and output buffers) are set to the size specified below
// by the PROTOCOL_BUFFER_SIZE
PARAM_BOOL( FINITE_BUFFERING );
PARAM( FINITE_BUFFER_SIZE ); // Zero is unbounded buffers
// Number of requests buffered between the sequencer and the L1 conroller
// This can be more accurately simulated in Opal, therefore it's set to an
// infinite number
// Only effects the simualtion when FINITE_BUFFERING is enabled
PARAM( PROCESSOR_BUFFER_SIZE );
// The PROTOCOL_BUFFER_SIZE limits the size of all other buffers connecting to
// Controllers.  Controlls the number of request issued by the L2 HW Prefetcher
PARAM( PROTOCOL_BUFFER_SIZE );

// Enable the TSO (Total Store Order) memory model
PARAM_BOOL( TSO ); // Note: This also disables the "write" STCs

// NETWORK PARAMETERS

// Network Topology: See TopologyType in external.sm for valid values
PARAM_STRING( g_NETWORK_TOPOLOGY );

// Cache Design specifies file prefix for topology
PARAM_STRING( g_CACHE_DESIGN );

PARAM( g_endpoint_bandwidth );
PARAM_BOOL( g_adaptive_routing );
PARAM( NUMBER_OF_VIRTUAL_NETWORKS );
PARAM( FAN_OUT_DEGREE );
PARAM_BOOL( g_PRINT_TOPOLOGY );

// transactional memory
PARAM( XACT_LENGTH );
PARAM( XACT_SIZE );
PARAM( ABORT_RETRY_TIME );

// Princeton Network (Garnet)
PARAM_BOOL( g_GARNET_NETWORK );
PARAM_BOOL( g_DETAIL_NETWORK );
PARAM_BOOL( g_NETWORK_TESTING );
PARAM( g_FLIT_SIZE );
PARAM( g_NUM_PIPE_STAGES );
PARAM( g_VCS_PER_CLASS );
PARAM( g_BUFFER_SIZE );

// MemoryControl:
PARAM( MEM_BUS_CYCLE_MULTIPLIER );
PARAM( BANKS_PER_RANK );
PARAM( RANKS_PER_DIMM );
PARAM( DIMMS_PER_CHANNEL );
PARAM( BANK_BIT_0 );
PARAM( RANK_BIT_0 );
PARAM( DIMM_BIT_0 );
PARAM( BANK_QUEUE_SIZE );
PARAM( BANK_BUSY_TIME );
PARAM( RANK_RANK_DELAY );
PARAM( READ_WRITE_DELAY );
PARAM( BASIC_BUS_BUSY_TIME );
PARAM( MEM_CTL_LATENCY );
PARAM( REFRESH_PERIOD );
PARAM( TFAW );
PARAM( MEM_RANDOM_ARBITRATE );
PARAM( MEM_FIXED_DELAY );

