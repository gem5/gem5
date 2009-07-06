
// FOR MOESI_CMP_token
//PARAM_BOOL(  FilteringEnabled, false, false );
//PARAM_BOOL(  DistributedPersistentEnabled, true, false );
//PARAM_BOOL(  DynamicTimeoutEnabled, true, false );
//PARAM( RetryThreshold, 1, false );
//PARAM( FixedTimeoutLatency, 300, false );

//PARAM( TraceWarmupLength, 1000000, false );

//PARAM( callback_counter, 0, false );
//PARAM( NUM_COMPLETIONS_BEFORE_PASS, 0, false );

//PARAM( tester_length, 0, false );
//PARAM( synthetic_locks, 2048, false );
//PARAM( think_time, 5, false );
//PARAM( wait_time, 5, false );
//PARAM( hold_time, 5, false );
//PARAM( deterministic_addrs, 1, false );
//PARAM_STRING( SpecifiedGenerator, "DetermInvGenerator", false );

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
// this use to be ruby/common/Global.hh

//PARAM_BOOL(  ProtocolDebugTrace, true, false );
// a string for filtering debugging output (for all g_debug vars see Debug.hh)
//PARAM_STRING( DEBUG_FILTER_STRING, "", false );
// filters debugging messages based on priority (low, med, high)
//PARAM_STRING( DEBUG_VERBOSITY_STRING, "", false );
// filters debugging messages based on a ruby time
//PARAM_ULONG( DEBUG_START_TIME, 0, false );
// sends debugging messages to a output filename
//PARAM_STRING( DEBUG_OUTPUT_FILENAME, "", false );

//PARAM_BOOL( ProfileHotLines, false, false );

// PROFILE_ALL_INSTRUCTIONS is used if you want Ruby to profile all instructions executed
// The following need to be true for this to work correctly:
// 1. Disable istc and dstc for this simulation run
// 2. Add the following line to the object "sim" in the checkpoint you run from:
//      instruction_profile_line_size: 4
// This is used to have simics report back all instruction requests

// For more details on how to find out how to interpret the output physical instruction
// address, please read the document in the simics-howto directory
//PARAM_BOOL( ProfileAllInstructions, false, false );

// Set the following variable to true if you want a complete trace of
// PCs (physical address of program counters, with executing processor IDs)
// to be printed to stdout. Make sure to direct the simics output to a file.
// Otherwise, the run will take a really long time!
// A long run may write a file that can exceed the OS limit on file length
//PARAM_BOOL( PRINT_INSTRUCTION_TRACE, false, false );
//PARAM( DEBUG_CYCLE, 0, false );

// Make the entire memory system perfect
//PARAM_BOOL( PERFECT_MEMORY_SYSTEM, false, false );
//PARAM( PERFECT_MEMORY_SYSTEM_LATENCY, 0, false );

// *********************************************
// SYSTEM PARAMETERS
// *********************************************

//PARAM( NumberOfChips, 1, false );
//PARAM( NumberOfCores, 2, false );
//PARAM_ARRAY( NumberOfCoresPerChip, int, m_NumberOfChips, 2, false);

// *********************************************
// CACHE PARAMETERS
// *********************************************

//PARAM( NumberOfCaches, m_NumberOfCores, false );
//PARAM( NumberOfCacheLevels, 1, false );
/* this returns the number of discrete CacheMemories per level (i.e. a split L1 counts for 2) */
//PARAM_ARRAY( NumberOfCachesPerLevel, int, m_NumberOfCacheLevels, m_NumberOfCores, false );   // this is the number of discrete caches if the level is private
                                                                                             // or the number of banks if the level is shared
//PARAM( CacheIDFromParams, 1, true );                                                         // returns a unique CacheID from the parameters (level, num, split_type)
//PARAM_ARRAY( CacheLatency, int, m_NumberOfCaches, 1, false );                                // returns the latency for cache, indexed by CacheID
//PARAM_ARRAY( CacheSplitType, string, m_NumberOfCaches, "unified", false );                   // returns "data", "instruction", or "unified", indexed by CacheID
//PARAM_ARRAY( CacheType, string, m_NumberOfCaches, "SetAssociative", false );                 // returns the type of a cache, indexed by CacheID
//PARAM_ARRAY( CacheAssoc, int, m_NumberOfCaches, 4, false );                                  // returns the cache associativity, indexed by CacheID
//PARAM_ARRAY( NumberOfCacheSets, int, m_NumberOfCaches, 256, false );                         // returns the number of cache sets, indexed by CacheID
//PARAM_ARRAY( NumberOfCacheSetBits, int, m_NumberOfCaches, log_int(256), false );             // returns the number of cache set bits, indexed by CacheID
//PARAM_ARRAY( CacheReplacementPolicy, string, m_NumberOfCaches, "PSEUDO_LRU", false );        // other option is "LRU"

//PARAM( DataBlockBytes, 64, false );
//PARAM( DataBlockBits, log_int(m_DataBlockBytes), false);

// ********************************************
// MEMORY PARAMETERS
// ********************************************

//PARAM_ARRAY( NumberOfControllersPerType, int, m_NumberOfCacheLevels+2, m_NumberOfCores, false);
//PARAM_ARRAY2D( NumberOfControllersPerTypePerChip, int, m_NumberOfCacheLevels+2, m_NumberOfChips, m_NumberOfCores, false);

// ********************************************
// DMA CONTROLLER PARAMETERS
// ********************************************

//PARAM( NumberOfDMA, 1, false );
//PARAM_ARRAY( NumberOfDMAPerChip, int, m_NumberOfChips, 1, false);
//PARAM_ARRAY( ChipNumFromDMAVersion, int, m_NumberOfDMA, 0, false );

//PARAM_ULONG( MemorySizeBytes, 4294967296, false );
//PARAM_ULONG( MemorySizeBits, 32, false);

//PARAM( NUM_PROCESSORS, 0, false );
//PARAM( NUM_L2_BANKS, 0, false );
//PARAM( NUM_MEMORIES, 0, false );
//PARAM( ProcsPerChip, 1, false );

// The following group of parameters are calculated.  They must
// _always_ be left at zero.
//PARAM( NUM_CHIPS, 0, false );
//PARAM( NUM_CHIP_BITS, 0, false );
//PARAM( MEMORY_SIZE_BITS, 0, false );
//PARAM( DATA_BLOCK_BITS, 0, false );
//PARAM( PAGE_SIZE_BITS, 0, false );
//PARAM( NUM_PROCESSORS_BITS, 0, false );
//PARAM( PROCS_PER_CHIP_BITS, 0, false );
//PARAM( NUM_L2_BANKS_BITS, 0, false );
//PARAM( NUM_L2_BANKS_PER_CHIP_BITS, 0, false );
//PARAM( NUM_L2_BANKS_PER_CHIP, 0, false );
//PARAM( NUM_MEMORIES_BITS, 0, false );
//PARAM( NUM_MEMORIES_PER_CHIP, 0, false );
//PARAM( MEMORY_MODULE_BITS, 0, false );
//PARAM_ULONG( MEMORY_MODULE_BLOCKS, 0, false );

// TIMING PARAMETERS
//PARAM( DIRECTORY_CACHE_LATENCY, 6, false );

//PARAM( NULL_LATENCY, 1, false );
//PARAM( ISSUE_LATENCY, 2, false );
//PARAM( CACHE_RESPONSE_LATENCY, 12, false );
//PARAM( L2_RESPONSE_LATENCY, 6, false );
//PARAM( L2_TAG_LATENCY, 6, false );
//PARAM( L1_RESPONSE_LATENCY, 3, false );

//PARAM( MEMORY_RESPONSE_LATENCY_MINUS_2, 158, false );
//PARAM( DirectoryLatency, 6, false );

//PARAM( NetworkLinkLatency, 1, false );
//PARAM( COPY_HEAD_LATENCY, 4, false );
//PARAM( OnChipLinkLatency, 1, false );
//PARAM( RecycleLatency, 10, false );
//PARAM( L2_RECYCLE_LATENCY, 5, false );
//PARAM( TIMER_LATENCY, 10000, false );
//PARAM( TBE_RESPONSE_LATENCY, 1, false );
//PARAM_BOOL(  PERIODIC_TIMER_WAKEUPS, true, false );

// constants used by CMP protocols
//PARAM( L1_REQUEST_LATENCY, 2, false );
//PARAM( L2_REQUEST_LATENCY, 4, false );
//PARAM_BOOL( SINGLE_ACCESS_L2_BANKS, true, false ); // hack to simulate multi-cycle L2 bank accesses

// Ruby cycles between when a sequencer issues a miss it arrives at
// the L1 cache controller
//PARAM( SequencerToControllerLatency, 4, false );

// Number of transitions each controller state machines can complete per cycle
//PARAM( L1CacheTransitionsPerCycle, 32, false );
//PARAM( L2CACHE_TRANSITIONS_PER_RUBY_CYCLE, 32, false );
//PARAM( DirectoryTransitionsPerCycle, 32, false );
//PARAM( DMATransitionsPerCycle, 1, false );

// Number of TBEs available for demand misses, prefetches, and replacements
//PARAM( NumberOfTBEs, 128, false );
//PARAM( NumberOfL1TBEs, 32, false );
//PARAM( NumberOfL2TBEs, 32, false );

// NOTE: Finite buffering allows us to simulate a wormhole routed network
// with idealized flow control.  All message buffers within the network (i.e.
// the switch's input and output buffers) are set to the size specified below
// by the PROTOCOL_BUFFER_SIZE
//PARAM_BOOL( FiniteBuffering, false, false );
//PARAM( FiniteBufferSize, 3, false ); // Zero is unbounded buffers
// Number of requests buffered between the sequencer and the L1 conroller
// This can be more accurately simulated in Opal, therefore it's set to an
// infinite number
// Only effects the simualtion when FINITE_BUFFERING is enabled
//PARAM( ProcessorBufferSize, 10, false );
// The PROTOCOL_BUFFER_SIZE limits the size of all other buffers connecting to
// Controllers.  Controlls the number of request issued by the L2 HW Prefetcher
//PARAM( ProtocolBufferSize, 32, false );

// NETWORK PARAMETERS

// Network Topology: See TopologyType in external.sm for valid values
//PARAM_STRING( NetworkTopology, "PT_TO_PT", false );

// Cache Design specifies file prefix for topology
//PARAM_STRING( CacheDesign, "NUCA", false );

//PARAM( EndpointBandwidth, 10000, false );
//PARAM_BOOL( AdaptiveRouting, true, false );
//PARAM( NumberOfVirtualNetworks, 6, false );
//PARAM( FanOutDegree, 4, false );
//PARAM_BOOL( PrintTopology, true, false );

// Princeton Network (Garnet)
//PARAM_BOOL( UsingGarnetNetwork, true, false );
//PARAM_BOOL( UsingDetailNetwork, false, false );
//PARAM_BOOL( UsingNetworkTesting, false, false );
//PARAM( FlitSize, 16, false );
//PARAM( NumberOfPipeStages, 4, false );
//PARAM( VCSPerClass, 4, false );
//PARAM( BufferSize, 4, false );

// MemoryControl:
//PARAM( MEM_BUS_CYCLE_MULTIPLIER, 10, false );
//PARAM( BANKS_PER_RANK, 8, false );
//PARAM( RANKS_PER_DIMM, 2, false );
//PARAM( DIMMS_PER_CHANNEL, 2, false );
//PARAM( BANK_BIT_0, 8, false );
//PARAM( RANK_BIT_0, 11, false );
//PARAM( DIMM_BIT_0, 12, false );
//PARAM( BANK_QUEUE_SIZE, 12, false );
//PARAM( BankBusyTime, 11, false );
//PARAM( RANK_RANK_DELAY, 1, false );
//PARAM( READ_WRITE_DELAY, 2, false );
//PARAM( BASIC_BUS_BUSY_TIME, 2, false );
//PARAM( MEM_CTL_LATENCY, 12, false );
//PARAM( REFRESH_PERIOD, 1560, false );
//PARAM( TFAW, 0, false );
//PARAM( MEM_RANDOM_ARBITRATE, 0, false );
//PARAM( MEM_FIXED_DELAY, 0, false );

