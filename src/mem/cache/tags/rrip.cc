/*
 * Copyright (c) 2015 Min Cai
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Authors: Min Cai
 */

/**
 * @file
 * Definitions of a RRIP tag store.
 */

#include "debug/CacheRepl.hh"
#include "mem/cache/tags/rrip.hh"
#include "mem/cache/base.hh"

RRIP::RRIP(const Params *p)
    : BaseSetAssoc(p)
{
  rrip = new DIP( "LLC", 0, 4, getNumSets(),
                    USE_INSERTION_POLICY, ((1<<USE_PSELwidth)-1), USE_SDMsize );
}

CacheBlk*
RRIP::accessBlock(ThreadID threadId, Addr pc, Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(threadId, pc, addr, is_secure, lat, master_id);

    if (blk != NULL) {
        rrip->RecordHit( threadId, blk->set );
        UpdateRRIP( threadId, blk->set, blk->way, true );
    }

    return blk;
}

CacheBlk*
RRIP::findVictim(ThreadID threadId, Addr pc, Addr addr)
{
    CacheBlk *blk = BaseSetAssoc::findVictim(threadId, pc, addr);
    int set = extractSet(addr);

    // if all blocks are valid, pick a replacement at random
    if (blk && blk->isValid()) {
        blk = findBlockBySetAndWay(set, Get_RRIP_Victim( set ));
    }

    return blk;
}

void
RRIP::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);

    ThreadID threadId = pkt->req->hasThreadId() ? pkt->req->threadId() : 0;

    int set = extractSet(pkt->getAddr());

    rrip->RecordMiss( threadId, set );
    UpdateRRIP( threadId, set, blk->way, false );
}

void
RRIP::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);
}

RRIP*
RRIPParams::create()
{
    return new RRIP(this);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//////// HELPER FUNCTIONS FOR REPLACEMENT UPDATE AND VICTIM SELECTION //////////
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This function finds the LRU victim in the cache set by returning the       //
// cache block at the bottom of the LRU stack. Top of LRU stack is '0'        //
// while bottom of LRU stack is 'assoc-1'                                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
INT32 RRIP::Get_RRIP_Victim( UINT32 setIndex )
{
    INT32   vicWay   = 0;
    bool    vicFound = false;

    do
    {
        // Search for victim whose RPV is furthest in future
        for(UINT32 way=0; way<assoc; way++)
        {
            CacheBlk *blk = findBlockBySetAndWay(setIndex, way);
            if( blk->RPV == maxRPV )
            {
                vicWay   = way;
                vicFound = true;
                break;
            }
        }

        // If victim is not found, then move all RPVs into future and
        // then repeat the search again

        if( !vicFound )
        {
            for(UINT32 way=0; way<assoc; way++)
            {
                CacheBlk *blk = findBlockBySetAndWay(setIndex, way);
                blk->RPV++;

                if( blk->RPV > maxRPV )
                {
                    cout<<"Why did this happen?"<<endl;
                    exit(0);
                }
            }
        }

    } while( !vicFound );

    // return vic way
    return vicWay;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                                                                          //
// This function implements the RRIP update routine. The arguments to                       //
// the function are the physical way and set index.                                                    //
//                                                                                                                          //
////////////////////////////////////////////////////////////////////////////////
void RRIP::UpdateRRIP( UINT32 tid, UINT32 setIndex, INT32 updateWayID, bool cacheHit )
{
    CacheBlk *blk = findBlockBySetAndWay(setIndex, updateWayID);

    // Set the RPV of line to near-immediate
    if( cacheHit )
    {
        blk->RPV = 0;
    }
    else
    {
        bool updateLRUstateOnMiss = rrip->DoNormalFill( tid, setIndex );

        if( updateLRUstateOnMiss )
        {
            blk->RPV = (maxRPV == 1) ? 0 : (maxRPV-1);
        }
    }
}

////////////////////////

// This is the interface for creating cache SDMs. The args are:
//   a)  cache name (debugging purpose)
//   b)  bank # (debugging purpose)
//   c)  threads sharing this cache
//   d)  total sets in the cache
//   e)  insertion policy to use (as defined by DIP_t)
//   f)  Max PSEL value (if 10-bit PSEL, use argument of 2^10)
//   g)  # of sets per SDM

DIP::DIP (
    string _cachename,
    uint32 _myID,
    uint32 _tpc,
    uint32 _totalSets,
    uint32 _usePolicy,
    uint32 _maxPSEL,
    uint32 _sizeOfSDM )
{

    DEBUG_PRINT("Initializing DIP-"<<_cachename<<"-"<<_myID
                <<" TPC: "<<_tpc<<" Policy: "<<_usePolicy);

    cachename            = _cachename;  // Initialize the cache name
    myID                 = _myID;       // Initialize My ID
    NumThreadsPerCache   = _tpc;        // Initialize # threads per cache
    NumberSetsInCache    = _totalSets;  // Initialize total set count
    InsertionPolicy      = _usePolicy;  // Initialize Insertion Policy
    PSELmax              = _maxPSEL;    // Initialize Max PSEL Value
    SDM_Size             = _sizeOfSDM;  // Initialize the SDM size

    BIP_epsilon          = 5;           // Minimum VSBF thresholds
    distinguishThreads   = true;        // distinguish threads
    printStatsPostWarmup = true;        // Should I Print Stats Post Warm-Up
    adapt                = true;        // adapt the insertion policy
    BIP_mask             = ~0;          // static BIP all threads do BIP

    // Initialize the Insertion Policy and stats
    Init();

    switch( InsertionPolicy )
    {
      case INSERTION_POLICY_THREAD_UNAWARE:
        DEBUG_PRINT("Do not distinguish threads");
        InsertionPolicy = INSERTION_POLICY_THREAD_AWARE;
        distinguishThreads = false;
        break;
    }

    if( InsertionPolicy == INSERTION_POLICY_THREAD_AWARE )
    {
        DEBUG_PRINT("Creating SDMs: "<<(_sizeOfSDM)<<" PSELmax: "<<PSELmax);

        CreateSDM( _sizeOfSDM );
    }
    else if (InsertionPolicy == INSERTION_POLICY_BIMODAL )
    {
        DEBUG_PRINT("Doing a bimodal insertion policy with mask: "<<hex<<BIP_mask);
    }
}

// Clear Stats
void DIP::Init()
{
    // Initialize Cache Miss Stats
    perSetMisses   = (uint64 **) calloc( NumSets(), sizeof(uint64*) );
    perSetHits     = (uint64 **) calloc( NumSets(), sizeof(uint64*) );
    for(uint32 i=0; i<NumSets(); i++)
    {
        perSetMisses[i] = (uint64 *) calloc( NumThreadsPerCache, sizeof(uint64) );
        perSetHits[i]   = (uint64 *) calloc( NumThreadsPerCache, sizeof(uint64) );
    }

    // Per Thread Stats
    totalPerThreadMisses = (uint64 *) calloc( NumThreadsPerCache, sizeof(uint64) );
    totalPerThreadHits   = (uint64 *) calloc( NumThreadsPerCache, sizeof(uint64) );
    totalPerThreadVulnerableInsertions   = (uint64 *) calloc( NumThreadsPerCache, sizeof(uint64) );

    // total misses
    totalDemandMisses = 0;
    totalPrefetchMisses = 0;

    // total misses
    totalVulnerableInsertions = 0;

    // total hits
    totalDemandHits   = 0;
    totalPrefetchHits = 0;

    // Per Thread Miss Stats in SDMs
    SDMmissCountPerThread = (uint64*) calloc( NumThreadsPerCache, sizeof(uint64) );

    // Initialize the per thread selection counters
    thread_active = (uint32*) calloc( NumThreadsPerCache, sizeof(uint32) );
    PSEL_cntr      = (uint32*) calloc( NumThreadsPerCache, sizeof(uint32) );
    policy_selecton_threshold  = (uint32*) calloc( NumThreadsPerCache, sizeof(uint32) );

    for(uint32 i=0; i<NumThreadsPerCache; i++)
    {
        PSEL_cntr[i]  = 0;
        policy_selecton_threshold[i] = (uint32) ((float) PSELmax * 0.5);
    }
}


// Destructor deletes leaderset info and psel counters
DIP::~DIP()
{
    SetMissHistogram();
    free( SDM_Info );
    free( PSEL_cntr );
}

void
DIP::CreateSDM( uint32 _sizeOfSDM )
{
    SDM_Info = (DIP_SDM_t*) calloc( NumSets(), sizeof(DIP_SDM_t) );

    RandomlyChooseSDMs();

//     SetIndexBitBasedSDM();
}

// Choose Leader Sets Based on bits 0-5 and 6-10 of the set index
void DIP::SetIndexBitBasedSDM()
{
#define MASK 31
#define TOP5(x) ((x>>6)&MASK)
#define BOT5(x) (x&MASK)
#define BIT5(x) ((x>>5)&1)

    for( uint32 si=0; si<NumSets(); si++)
    {
        // Dedicate Per Thread SDMs
        // Can determine if it is my dedicated set or not
        for( uint32 c=0; c<NumThreadsPerCache; c++ )
        {
            int index = si - c - 1;

            if( BIT5(index) && index>=0 && (TOP5(index) == BOT5(index)) )
            {
                // Check to make sure this set isn't already assigned
                if( SDM_Info[si].SDM_Type != DIP_FOLLOWERS )
                {
                    cout<<"This set: "<<si<<" already assigned to: "
                        <<SDM_Info[si].SDM_Owner<<endl;
                    exit(0);
                }

                SDM_Info[si].SDM_Type = DIP_LRU_SDM;
                SDM_Info[si].SDM_Owner = c;

                DEBUG_PRINT("Assigning: "<<si<<" as NF for core: "<<c);
            }

            index = si + c + 1;
            if( !BIT5(index) && index<=2047 && (TOP5(index) == BOT5(index)) )
            {
                // Check to make sure this set isn't already assigned
                if( SDM_Info[si].SDM_Type != DIP_FOLLOWERS )
                {
                    cout<<"This set: "<<si<<" already assigned to: "
                        <<SDM_Info[si].SDM_Owner<<endl;
                    exit(0);
                }


                SDM_Info[si].SDM_Type = DIP_BIP_SDM;
                SDM_Info[si].SDM_Owner = c;

                DEBUG_PRINT("Assigning: "<<si<<" as BF for core: "<<c);
            }
        }
    }
}

///////////////////////////////////////
// My Pseudo Random Number Generator //
///////////////////////////////////////

unsigned long dip_rand_seed = 1;
unsigned long dip_max_rand  = 1048576;

inline int dip_rand(void) {
    dip_rand_seed = dip_rand_seed * 1103515245 + 12345;
    return((unsigned)(dip_rand_seed/65536) % dip_max_rand);
}

inline void dip_srand(unsigned seed) {
    dip_rand_seed = seed;
}

///////////////////////////////////////
///////////////////////////////////////

// Randomly assign sets to SDMs
void DIP::RandomlyChooseSDMs()
{
    uint32      tot_SDM_Size;        // total SDM size of cache
    uint32      tot_SDM_Count;       // Number of SDMs per thread

    tot_SDM_Size  = SDM_Size * NumThreadsPerCache;
    tot_SDM_Count = 2;

    if( NumSets() < (tot_SDM_Size * tot_SDM_Count ) ) {
        cout<<"Size of SDMs is Greater Than Total Sets"<<endl;
        assert(0);
    }

    // When using multiple cache banks, seeding is to ensure that all
    // banks use the same sampled sets
    dip_rand_seed = 1;

    for(uint32 p=0; p<tot_SDM_Count; p++)
    {
        // Assign per-thread SDMs
        uint32 tid = 0;
        uint32 ownerSets = SDM_Size;

        for(uint32 ldr=0; ldr<tot_SDM_Size; ldr++)
        {
            uint32 rand_set_indx;
            do
            {
//                 rand_set_indx = rand() % NumSets();
                rand_set_indx = dip_rand() % NumSets();

            }
            while( SDM_Info[rand_set_indx].SDM_Type != DIP_FOLLOWERS );

            // Set the Leader Set Type (NF or BF)
            SDM_Info[rand_set_indx].SDM_Type = p+1;

            // Set the leader set owner
            SDM_Info[rand_set_indx].SDM_Owner = tid;

            DEBUG_PRINT("Assigning: "<<rand_set_indx<<" as Type: "<<(p+1)<<" for core: "<<tid);

            --ownerSets;

            // If owner sets has reached zero, move to next tid
            if( ownerSets == 0 )
            {
                tid++;
                ownerSets = SDM_Size;
            }
        }
    }
}

// Input:  A threshold value describing the percent of misses normally filled
// Output: True if insertion should be normal, false if insertion is vulnerable
bool DIP::BimodalSuggestion( uint32 throttle )
{
    // If random number less than throttle, need to do NF
    if( (uint32) (rand() % 100) <= throttle ) return true;

    // Do VF
    return false;
}

// Input:  CPU or Thread ID and set number
// Output: none
// Function: update PSEL counter based on misses in competing insertion policy
void DIP::RecordMiss(
    uint32 cpuid,
    uint32 setnum,
    bool   is_demand_req,
    bool   warmup_mode )
{

    cpuid = cpuid % NumThreadsPerCache;

    if( setnum >= NumSets() )
    {
        cout<<cachename<<": Requested Set is: "<<setnum<<" > "<<NumSets()<<endl;
        exit(0);
    }

    // total misses increment
    if( is_demand_req )
        totalDemandMisses++;
    else
        totalPrefetchMisses++;

    perSetMisses[setnum][cpuid]++;
    totalPerThreadMisses[cpuid]++;

    if( (warmup_mode == false) && (printStatsPostWarmup) )
    {
        printStatsPostWarmup = false;
        SetMissHistogram();
    }

    if( (NumRefs() % PRINT_EVERY) == 0)
    {
        PrintThrottles( warmup_mode );
    }

    // If we are not distinguishing threads, then set the cpuid = 0
    if( distinguishThreads == false ) cpuid = 0;

//     if( !thread_active[cpuid] )
//     {
//         DEBUG_PRINT(cachename<<": "<<myID<<" Registering Thread: "<<cpuid);
//
//         // register the thread as having activity in the cache
//         thread_active[ cpuid ] = 1;
//
//         // reset the PSEL value
//         PSEL_cntr[cpuid] = 0;
//     }

    if( InsertionPolicy == INSERTION_POLICY_THREAD_AWARE )
    {
        SetDuel_RecordMiss( cpuid, setnum, warmup_mode );
    }
}

// Input:  CPU or Thread ID and set number
// Output: none
// Function: update PSEL counter based on misses in competing insertion policy
void DIP::RecordHit(
    uint32 cpuid,
    uint32 setnum,
    bool   is_demand_req,
    bool   warmup_mode)
{
    cpuid = cpuid % NumThreadsPerCache;

    if( (warmup_mode == false) && (printStatsPostWarmup) )
    {
        printStatsPostWarmup = false;
        SetMissHistogram();
    }

    // total hits increment
    if( is_demand_req )
        totalDemandHits++;
    else
        totalPrefetchHits++;

    perSetHits[setnum][cpuid]++;
    totalPerThreadHits[cpuid]++;

    if( (NumRefs() % PRINT_EVERY) == 0 )
    {
        PrintThrottles( warmup_mode );
    }

    // If we are not distinguishing threads, then set the cpuid = 0
    if( distinguishThreads == false ) cpuid = 0;

//     if( !thread_active[cpuid] )
//     {
//         DEBUG_PRINT(cachename<<": "<<myID<<" Registering Thread: "<<cpuid);
//
//         // register the thread as having activity in the cache
//         thread_active[ cpuid ] = 1;
//     }
}

// Input:  CPU or Thread ID and set number
// Output: true if LRU should be updated, false if VF
// Function: Should the current cpuid do a NF or VF in this set?
bool DIP::DoNormalFill(
    uint32 cpuid,
    uint32 setnum )
{
    bool  updateLRU = true;

    cpuid = cpuid % NumThreadsPerCache;

    // If we are not distinguishing threads, then set the cpuid = 0
    if( distinguishThreads == false ) cpuid = 0;

    if( !thread_active[cpuid] )
    {
        DEBUG_PRINT(cachename<<": "<<myID<<" Registering Thread: "<<cpuid);
        // register the thread as having activity in the cache
        thread_active[ cpuid ] = 1;

//         PSEL_cntr[cpuid] = pselMAX/2;
    }

    if( InsertionPolicy == INSERTION_POLICY_THREAD_AWARE )
    {
        updateLRU = SetDuel_DoNormalFill( cpuid, setnum );

        if( updateLRU == false )
        {
            totalVulnerableInsertions++;
            totalPerThreadVulnerableInsertions[cpuid]++;
        }
    }
    else if( InsertionPolicy == INSERTION_POLICY_BIMODAL )
    {
        updateLRU = BimodalSuggestion( BIP_epsilon );

        if( ((1<<cpuid) & BIP_mask)==0 )
        {
            updateLRU = true;
        }
    }
    else if( InsertionPolicy != INSERTION_POLICY_NONE )
    {
        cout <<"Unimplemented Insertion Policy -- What is this policy supposed to do?"<<endl;
        assert(0);
    }

    return updateLRU;
}

// Input:  CPU or Thread ID and set number
// Output: none
// Function: update PSEL counter based on misses in competing insertion policy
void DIP::SetDuel_RecordMiss(
    uint32 cpuid,
    uint32 setnum,
    bool   warmup_mode)
{
    uint32 owner   = SDM_Info[setnum].SDM_Owner;
    uint32 settype = SDM_Info[setnum].SDM_Type;

    // if it is an SDM that does NF policy increment PSEL
    if( settype == DIP_LRU_SDM )
    {
        if( PSEL_cntr[owner] < PSELmax ) PSEL_cntr[owner]++;

        // Track number of misses to the dedicated sets
        SDMmissCountPerThread[owner]++;
    }
    // if it is an SDM that does BF policy decrement PSEL
    else if( settype == DIP_BIP_SDM )
    {
        if( PSEL_cntr[owner] > 0 ) PSEL_cntr[owner]--;

        // Track number of misses to the dedicated sets
        SDMmissCountPerThread[owner]++;
    }
}

// Input:  CPU or Thread ID and set number
// Output: true if LRU should be updated, false if VF
// Function: Should the current cpuid do a NF or VF in this set?
bool DIP::SetDuel_DoNormalFill(
    uint32 cpuid,
    uint32 setnum )
{
    bool updateLRU = true;

    // Is it an SDM that does NF policy and is dedicated to this cpu?
    if( setnum != (uint32) -1
        && SDM_Info[setnum].SDM_Type == DIP_LRU_SDM
        && SDM_Info[setnum].SDM_Owner == cpuid )
    {
        updateLRU = true;
    }
    // Is it an SDM that does BF policy and is dedicated to this cpu?
    else if( setnum != (uint32) -1
             && SDM_Info[setnum].SDM_Type == DIP_BIP_SDM
             && SDM_Info[setnum].SDM_Owner == cpuid )
    {

        uint32 useThreshold = BIP_epsilon;

        updateLRU = BimodalSuggestion( useThreshold );
    }
    else // it is a follower set
    {
        uint32 InsertionPolicy = GetCurrentInsertionPolicy( cpuid );

        updateLRU = (InsertionPolicy == DIP_LRU_SDM) ? true : BimodalSuggestion( BIP_epsilon );
    }

    return updateLRU;
}

// Input:  CPU or Thread ID
// Output: Insertion Policy for current CPU
// Function: We determine the current insertion policy

uint32 DIP::GetCurrentInsertionPolicy( uint32 cpuid )
{
    // If we are not adapting at all, then just return true
    if( !adapt ) return DIP_LRU_SDM;

    if( PSEL_cntr[cpuid] > policy_selecton_threshold[cpuid] )
    {
        return DIP_BIP_SDM;
    }
    else if( PSEL_cntr[cpuid] <= (policy_selecton_threshold[cpuid]) )
    {
        return DIP_LRU_SDM;
    }

    return DIP_LRU_SDM;
}

// Print Dynamic Throttles
void DIP::PrintThrottles(bool warmup_mode)
{
    if( DEBUG_PRINT_ON )
    {
        PrintStats( warmup_mode );
    }
}

// Print Dynamic Throttles
void DIP::PrintStats( bool warmup_mode )
{
    uint32 missrate = 0;
    uint64 totrefs = totalDemandMisses + totalDemandHits;

    if( totrefs ) missrate = (totalDemandMisses * 100) / (totalDemandMisses+totalDemandHits);

    cout<<cachename<<": "<<myID
        <<" WarmUp: "<<(uint32)warmup_mode
        <<" Miss Rate: "<<missrate
        <<" Total Fills: "<<totalDemandMisses
        <<" Vulnerable Fills: "<<totalVulnerableInsertions
        <<" Total Demand References: "<<NumDemandRefs()
        <<" Total Prefetch Fills: "<<totalPrefetchMisses
        <<" Total Prefetch References: "<<NumPrefetchRefs()
        <<endl;

    for(uint32 tid=0; tid<NumThreadsPerCache; tid++)
    {

        if( thread_active[tid] )
        {

            cout<<"\tCPU ID: "<<tid;

            if( InsertionPolicy == INSERTION_POLICY_THREAD_AWARE )
            {
                cout<<" Selector: "<<PSEL_cntr[tid]
                    <<" Throttle: "<<(GetCurrentInsertionPolicy(tid)==DIP_BIP_SDM ? (BIP_epsilon):(100));

            }

            uint64 threadRefs = totalPerThreadMisses[tid]+totalPerThreadHits[tid];
            double MR = threadRefs ? ((double)totalPerThreadMisses[tid]/threadRefs*100) : 0;

            cout<<"\tMisses: "<<totalPerThreadMisses[tid]<<"/"<<threadRefs<<" Miss Rate: "<<MR<<endl;

        }

    }
}

// Set Miss Histograms
void DIP::SetMissHistogram()
{
    if( DEBUG_PRINT_ON )
    {
        cout<<cachename<<": "<<myID<<" Cache Statistics: "<<totalDemandMisses<<"/"<<NumRefs()
            <<" Miss Rate: "<<((double)totalDemandMisses/NumDemandRefs()*100)<<endl;

        // Per Thread Stats
        for(uint32 tid=0; tid<NumThreadsPerCache; tid++)
        {
            if( thread_active[tid] )
            {
                uint64 threadRefs = totalPerThreadMisses[tid]+totalPerThreadHits[tid];
                cout<<"\ttid: "<<tid<<" "<<totalPerThreadMisses[tid]<<"/"<<threadRefs
                    <<" Miss Rate: "<<((double)totalPerThreadMisses[tid]/threadRefs*100)<<endl;
            }
        }

        cout<<endl<<"Per Set Statistics"<<endl;
        for(uint32 setnum=0; setnum<NumSets(); setnum++)
        {
            cout<<"\tSet:\t"<<setnum<<"\t";

            if( InsertionPolicy == INSERTION_POLICY_THREAD_AWARE )
            {
                cout<<"owner:\t"<<SDM_Info[setnum].SDM_Owner<<"\t"
                    <<"type:\t"<<SDM_Info[setnum].SDM_Type<<"\t";
            }

            uint64 setMisses = 0;
            for(uint32 tid=0; tid<NumThreadsPerCache; tid++)
            {
                if( thread_active[tid] )
                {
                    uint64 tidMisses = perSetMisses[setnum][tid];
                    uint64 tidHits   = perSetHits[setnum][tid];
                    uint64 tidRefs   = tidMisses + tidHits;

                    cout<<"tid: "<<tid<<" Misses: "<<tidMisses<<"/"<<tidRefs<<"\t";
                    setMisses += tidMisses;
                }
            }

            cout<<"Total Misses:\t"<<setMisses<<endl;
        }
        cout<<endl<<endl;
    }
    else
    {
        PrintStats( false );
    }
}

