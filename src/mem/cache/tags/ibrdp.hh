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
 * Declaration of an IbRDP tag store.
 */

#ifndef __MEM_CACHE_TAGS_IBRDP_HH__
#define __MEM_CACHE_TAGS_IBRDP_HH__

#include "mem/cache/tags/base_set_assoc.hh"
#include "params/IbRDP.hh"

// Works by finding position of MSB set.
static inline int CRC_FloorLog2(uint32_t n)
{
    int p = 0;

    if (n == 0) return -1;

    if (n & 0xffff0000) { p += 16; n >>= 16; }
    if (n & 0x0000ff00) { p +=  8; n >>=  8; }
    if (n & 0x000000f0) { p +=  4; n >>=  4; }
    if (n & 0x0000000c) { p +=  2; n >>=  2; }
    if (n & 0x00000002) { p +=  1; }

    return p;
}

// Works by finding position of MSB set.
// @returns -1 if n == 0.
static inline int CRC_CeilLog2(uint32_t n)
{
    return CRC_FloorLog2(n - 1) + 1;
}

// MAX and SAFE values for the IbRDPredictor confidence counters
#define MAX_CONFIDENCE 3
#define SAFE_CONFIDENCE 0

// Max value and quantization granularity for the prediction
#define MAX_VALUE_PREDICTION 15
#define QUANTUM_PREDICTION 8192

// Max value and quantization granulariry for the timestamp
#define MAX_VALUE_TIMESTAMP 7
#define QUANTUM_TIMESTAMP 16384

// Sampling Period and max reuse distance that the sampler must be able to hold
#define SAMPLER_PERIOD 4096
#define SAMPLER_MAX_RD ( ( MAX_VALUE_PREDICTION + 1 ) * QUANTUM_PREDICTION )

// Number of bits that we keep for the PC and the address
// We enforce these numbers of bits by calling TransformAddress and
// TransformPC from the entry-points of our code so that the only values we
// use throughout our code are limited to these numbers of bits
#define BITS_PC 20
#define BITS_ADDR 26

// Sets and associativity of the IbRDPredictor storage
#define IBRDP_SETS 16
#define IBRDP_WAYS 16

//---------------------------------------------------------------------------///
//---------------------------------------------------------------------------///
//---            INSTRUCTION BASED REUSE DISTANCE PREDICTOR               ---///
//---------------------------------------------------------------------------///
//---------------------------------------------------------------------------///
typedef struct
{
    uint32_t valid;               // Valid: 1 bit
    uint32_t tag;                 // Tag: 20 bits PC - 4 bits for set indexing
    uint32_t prediction;          // Prediction: 4 bits (limited by MAX_VALUE_PREDICTION)
    uint32_t confidence;          // Confidence: 2 bits (limited by MAX_CONFIDENCE)
    uint32_t StackPosition;       // StackPosition: 4 bits (log2(IBRDP_WAYS))
} IBRDP_Entry;                  // Total = 27 bits

class IBRDPredictor
{
  private:
    IBRDP_Entry **predictor;    // Predictor Storage
    uint32_t numsets;             // Number of sets, constant and  == IBRDP_SETS
    uint32_t assoc;               // Associativity, constant and == IBRDP_WAYS
    uint32_t set_mask;            // mask for keeping the set indexing bits of pc
                                //    always == numsets - 1;
    uint32_t set_shift;           // # of bits that we shift pc, to get tag
                                //    always == log2(numsets)

  public:
    IBRDPredictor( uint32_t _sets, uint32_t _ways );
    // Lookup returns a reuse distance prediction for the given pc
    uint32_t Lookup( uint32_t pc );
    // Update, changes the confidence counters for the given pc,
    // and if they are zero and observation does not match the prediction
    // it changes the prediction
    void Update( uint32_t pc, uint32_t observation );

  private:
    // Helper function:
    // FindEntry finds the entry which corresponds to the pc
    // GetEntry returns the LRU entry in the set and re-initializes it
    uint32_t FindEntry( uint32_t pc );
    uint32_t GetEntry( uint32_t pc );
};

//---------------------------------------------------------------------------///
//---------------------------------------------------------------------------///
//---                     REUSE DISTANCE SAMPLER                          ---///
//---------------------------------------------------------------------------///
//---------------------------------------------------------------------------///

typedef struct
{
    uint32_t valid;           // Valid: 1 bit
    uint32_t pc;              // PC of the sampled access: 20 bits
    uint32_t address;         // Address of the sampled access: 26 bits
    uint32_t FifoPosition;    // Position in the FIFO Queue: log2(sampler_size) bits
                            //     = 5 bits
} RDSamplerEntry;           // Total: 52 bits

class RDSampler
{
  private:
    RDSamplerEntry *sampler;    // Sampler Storage
    IBRDPredictor *predictor;   // Reference to the IbRDPredictor

    uint32_t size;                // Sampler size == SAMPLER_MAX_RD / SAMPLER_PERIOD
    uint32_t period;              // Sampling Period == SAMPLER_PERIOD
    uint32_t sampling_counter;    // Counts from period-1 to zero.
                                // We take a new sample when it reaches zero

  public:
    RDSampler( uint32_t _period, uint32_t _max_rd, IBRDPredictor *_predictor );
    // Update is the main function of the sampler
    // 1) It performs an associative search on the sampler for the given
    //    address and if there is a hit it update the predictor
    // 2) It takes a new sample every period accesses and it enqueues it
    //    in the sampler. If the dequeued entry was still valid, we update
    //    the predictor using the entries pc and the MAX_VALUE_PREDICTION
    void Update( uint32_t address, uint32_t pc );
};

class IbRDP : public BaseSetAssoc
{
  public:
    /** Convenience typedef. */
    typedef IbRDPParams Params;

    /**
     * Construct and initialize this tag store.
     */
    IbRDP(const Params *p);

    /**
     * Destructor
     */
    ~IbRDP() {}

    CacheBlk* accessBlock(Addr pc, Addr addr, bool is_secure, Cycles &lat,
                         int context_src);
    CacheBlk* findVictim(Addr pc, Addr addr);
    void insertBlock(PacketPtr pkt, BlkType *blk);
    void invalidate(CacheBlk *blk);

  private:
    void UpdateOnEveryAccess( uint32_t address, uint32_t PC );

    int Get_IBRDP_Victim( uint32_t setIndex, Addr PC, Addr paddr );

    void UpdateIBRDP( uint32_t setIndex, int updateWayID, Addr PC, bool cacheHit );

    // 1) We use the 17 bit accessesCounter instead of the 'timer' variable
    //    because we wish to count the accesses caused only by loads and stores
    // 2) We break the accessesCounter into a lower and a higher part, just
    //    to make our lives easier: Since only the 3 higher order bits of
    //    the accessesCounter are used directly by our policy, we keep them
    //    separated by the lower 14 bits. One could very well merge the two
    //    parts in one variable and just write some extra code to isolate
    //    the three higher order bits.
    uint32_t accessesCounterLow;  // Lower 14 bits of acccessesCounter
    uint32_t accessesCounterHigh; // Higher 3 bits of accessesCounter
    IBRDPredictor *predictor;   // Reference to the IbRDPredictor
    RDSampler *rdsampler;       // Reference to the RDSampler
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// ---------------                HELPER FUNCTIONS               ---------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// returns bits 21:2 of pc
// We ignore bits 0 and 1, even though the x86 architecture instructions are
// not alligned on word boundaries, because we believe that the possibility of
// two different memory instrunction starting in the same memory word is too
// low to worth one or two extra bits.
static inline uint32_t TransformPC( Addr pc )
{
    Addr new_pc;
    new_pc = pc >> 2;
    new_pc = new_pc & ( ( 1 << BITS_PC ) - 1 );
    return (uint32_t)new_pc;
}

// returns bits 25:0 of address (bits 31:6 of the real address,
// the argument address has already been stripped of the byte offset bits)
static inline uint32_t TransformAddress( Addr address )
{
    Addr new_address;
    new_address = address & ( ( 1 << BITS_ADDR ) - 1 );
    return (uint32_t)new_address;
}

//
// The rest of the helper functions are self-explanatory, I think
//

static inline uint32_t QuantizeTimestamp( uint32_t timestamp )
{
    return (timestamp / QUANTUM_TIMESTAMP) & MAX_VALUE_TIMESTAMP;
}

static inline uint32_t UnQuantizeTimestamp( uint32_t timestamp )
{
    return timestamp * QUANTUM_TIMESTAMP;
}

static inline uint32_t QuantizePrediction( uint32_t prediction )
{
    prediction = prediction / QUANTUM_PREDICTION;

    if (prediction < MAX_VALUE_PREDICTION)
        return prediction;
    else
        return MAX_VALUE_PREDICTION;
}

static inline uint32_t UnQuantizePrediction( uint32_t prediction )
{
    return prediction * QUANTUM_PREDICTION;
}

#endif // __MEM_CACHE_TAGS_IBRDP_HH__
