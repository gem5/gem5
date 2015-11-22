#ifndef DIP_H
#define DIP_H

#ifndef DEBUG_PRINT_ON
#define DEBUG_PRINT_ON 0
#endif

#if DEBUG_PRINT_ON
#define DEBUG_PRINT(X) cout<<X<<dec<<endl;
#else
#define DEBUG_PRINT(X)
#endif

#define PRINT_EVERY 10000

#include "utils.hh"

#define  uint32 UINT32
#define  uint64 UINT64

/*************************************************************************************
 **
 **
 **
 *************************************************************************************/

// SDM Types
typedef enum
{
    DIP_FOLLOWERS       = 0,
    DIP_LRU_SDM         = 1,
    DIP_BIP_SDM         = 2,
} DIP_SDM_Type_t;

// SDM Descriptor
typedef struct
{
    uint32       SDM_Type;   // What type of SDM
    uint32       SDM_Owner;  // Who manages the SDM
} DIP_SDM_t;

// Insertion Policy
typedef enum
{
    INSERTION_POLICY_NONE                    = 0,
    INSERTION_POLICY_THREAD_UNAWARE          = 1,
    INSERTION_POLICY_THREAD_AWARE            = 2,
    INSERTION_POLICY_BIMODAL                 = 3,
} DIP_t;

class DIP
{
    uint32      myID;
    string      cachename;

    uint32      NumberSetsInCache;

    // Policy Selector Counter and Max Value
    uint32      *PSEL_cntr;
    uint64      *SDMmissCountPerThread;
    uint32      *policy_selecton_threshold;
    uint32      *thread_active;
    uint32      PSELmax;

    bool        distinguishThreads;
    bool        printStatsPostWarmup;
    bool        adapt;

    // Which policy should I use?
    uint32      InsertionPolicy;

    // Statistics on hits and misses
    uint64      totalDemandMisses;
    uint64      totalDemandHits;

    uint64      totalPrefetchMisses;
    uint64      totalPrefetchHits;

    uint64      totalVulnerableInsertions; // insertions at RRIP tail (i.e. "LRU position")

    uint64      *totalPerThreadMisses;
    uint64      *totalPerThreadHits;

    uint64      *totalPerThreadVulnerableInsertions;

    // Per Set Cache Misses by Thread
    uint64      **perSetMisses;
    uint64      **perSetHits;

    /*************************************************************/
    /*    SET DUELING SPECIFIC PARAMS                            */
    /*************************************************************/
    DIP_SDM_t   *SDM_Info;

    uint32      NumThreadsPerCache;  // number threads per cache
    uint32      SDM_Size;            // Size of an SDM

    uint32      BIP_epsilon;         // BIP epsilon
    uint32      BIP_mask;            // Mask to define static BIP

  public:

    //  This is the interface for creating cache SDMs. The args are:
    //   a)  cache name (debugging purpose)
    //   b)  bank # (debugging purpose)
    //   c)  threads sharing this cache
    //   d)  total sets in the cache
    //   e)  insertion policy to use (as defined by DIP_t above)
    //   f)  Max PSEL value (if 10-bit PSEL, use argument of 2^10)
    //   g)  # of sets per SDM

    DIP(
        string _cachename,
        uint32 _myID,
        uint32 _tpc,
        uint32 _totalSets=2048,
        uint32 _usePolicy=INSERTION_POLICY_THREAD_AWARE,
        uint32 _maxPSEL=1024,
        uint32 _sizeOfSDM=32 );

    // Initialize Stats
    void Init();

    // Initialize BIP Mask
    void InitBIPmask( uint32 _mask ) { BIP_mask = _mask; };

    // Destructor deletes dynamic structures
    ~DIP();

    // Input:  CPU or Thread ID and set number
    // Output: none
    // Function: update PSEL counter based on misses in competing fill policy
    void RecordMiss(
        uint32 cpuid,
        uint32 setnum,
        bool   is_demand=true,
        bool   warmup_mode=false);

    // Input:  CPU or Thread ID and set number
    // Output: none
    // Function: update PSEL counter based on misses in competing fill policy
    void RecordHit(
        uint32 cpuid,
        uint32 setnum,
        bool   is_demand=true,
        bool   warmup_mode=false);

    // Input:  CPU or Thread ID and set number
    // Output: true if LRU should be updated, false if VF
    // Function: Should the current cpuid do a NF or VF in this set?
    bool DoNormalFill(
        uint32 cpuid,
        uint32 setnum );

    bool DoNormalFill(
        uint32 cpuid,
        uint32 setnum,
        uint64 cycle)
    {
        return DoNormalFill( cpuid, setnum );
    }

    // Set Miss Histograms
    void SetMissHistogram();

  private:

    inline uint32 NumSets() { return NumberSetsInCache; }
    inline uint32 NumDemandRefs() { return (totalDemandHits+totalDemandMisses); }
    inline uint32 NumPrefetchRefs() { return (totalPrefetchHits+totalPrefetchMisses); }
    inline uint64 NumRefs() { return (NumDemandRefs() + NumPrefetchRefs()); }


    // Create SDMs for threads sharing the cache
    void CreateSDM( uint32 _sizeOfSDM );

    // Randomly create SDM entries
    void RandomlyChooseSDMs() ;

    // Choose SDMs Based on bits 0-5 and 6-10 of the set index
    void SetIndexBitBasedSDM() ;

    // Input:  A threshold value describing the percent of misses normally filled
    // Output: True if fill should be normal, false if fill is vulnerable
    bool BimodalSuggestion( uint32 throttle );

    // Input:  CPU or Thread ID
    // Output: Fill Policy for current CPU
    // Function: Depending on whether hysteresis is enabled or not, we determine
    // the current fill policy
    uint32 GetCurrentInsertionPolicy( uint32 cpuid );

    // Input:  CPU or Thread ID and set number
    // Output: none
    // Function: update PSEL counter based on misses in competing fill policy
    void SetDuel_RecordMiss(
        uint32 cpuid,
        uint32 setnum,
        bool   warmup_mode=false);

    // Input:  CPU or Thread ID and set number
    // Output: true if LRU should be updated, false if VF
    // Function: Should the current cpuid do a NF or VF in this set?
    bool SetDuel_DoNormalFill(
        uint32 cpuid,
        uint32 setnum );

  public:

    // Print Throttle Traces
    void PrintThrottles( bool warmup_mode );

    // Print Stats
    void PrintStats( bool warmup_mode );

    // Return the PSEL value for the current processor
    uint32 GetPSEL( uint32 cpuid )
    {
        // determine PSEL id
        uint32 PSELid = cpuid % NumThreadsPerCache;

        // return PSEL value
        return PSEL_cntr[ PSELid ];
    }

};

#endif