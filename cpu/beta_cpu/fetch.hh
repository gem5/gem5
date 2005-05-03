// Todo: add in statistics, only get the MachInst and let decode actually
// decode, think about SMT fetch,
// fix up branch prediction stuff into one thing,
// Figure out where to advance time buffer.  Add a way to get a
// stage's current status.

#ifndef __CPU_BETA_CPU_SIMPLE_FETCH_HH__
#define __CPU_BETA_CPU_SIMPLE_FETCH_HH__

//Will want to include: time buffer, structs, MemInterface, Event,
//whatever class bzero uses, MemReqPtr

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/pc_event.hh"
#include "mem/mem_interface.hh"
#include "sim/eventq.hh"

/**
 * SimpleFetch class to fetch a single instruction each cycle.  SimpleFetch
 * will stall if there's an Icache miss, but otherwise assumes a one cycle
 * Icache hit.
 */

template <class Impl>
class SimpleFetch
{
  public:
    /** Typedefs from Impl. */
    typedef typename Impl::ISA ISA;
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename CPUPol::BPredUnit BPredUnit;
    typedef typename CPUPol::FetchStruct FetchStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;

    /** Typedefs from ISA. */
    typedef typename ISA::MachInst MachInst;

  public:
    enum Status {
        Running,
        Idle,
        Squashing,
        Blocked,
        IcacheMissStall,
        IcacheMissComplete
    };

    // May eventually need statuses on a per thread basis.
    Status _status;

    bool stalled;

  public:
    /** SimpleFetch constructor. */
    SimpleFetch(Params &params);

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);

    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    void tick();

    void fetch();

    void processCacheCompletion();

    // Figure out PC vs next PC and how it should be updated
    void squash(const Addr &new_PC);

  private:
    inline void doSquash(const Addr &new_PC);

    void squashFromDecode(const Addr &new_PC, const InstSeqNum &seq_num);

    /**
     * Looks up in the branch predictor to see if the next PC should be
     * either next PC+=MachInst or a branch target.
     * @params next_PC Next PC variable passed in by reference.  It is
     * expected to be set to the current PC; it will be updated with what
     * the next PC will be.
     * @return Whether or not a branch was predicted as taken.
     */
    bool lookupAndUpdateNextPC(DynInstPtr &inst, Addr &next_PC);

    /**
     * Fetches the cache line that contains fetch_PC.  Returns any
     * fault that happened.  Puts the data into the class variable
     * cacheData.
     * @params fetch_PC The PC address that is being fetched from.
     * @return Any fault that occured.
     */
    Fault fetchCacheLine(Addr fetch_PC);

    // Align an address (typically a PC) to the start of an I-cache block.
    // We fold in the PISA 64- to 32-bit conversion here as well.
    Addr icacheBlockAlignPC(Addr addr)
    {
        addr = ISA::realPCToFetchPC(addr);
        return (addr & ~(cacheBlkMask));
    }

  public:
    class CacheCompletionEvent : public Event
    {
      private:
        SimpleFetch *fetch;

      public:
        CacheCompletionEvent(SimpleFetch *_fetch);

        virtual void process();
        virtual const char *description();
    };

//    CacheCompletionEvent cacheCompletionEvent;

  private:
    /** Pointer to the FullCPU. */
    FullCPU *cpu;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get decode's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromDecode;

    /** Wire to get rename's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromRename;

    /** Wire to get iew's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Internal fetch instruction queue. */
    TimeBuffer<FetchStruct> *fetchQueue;

    //Might be annoying how this name is different than the queue.
    /** Wire used to write any information heading to decode. */
    typename TimeBuffer<FetchStruct>::wire toDecode;

    /** Icache interface. */
    MemInterface *icacheInterface;

    /** BPredUnit. */
    BPredUnit branchPred;

    /** Memory request used to access cache. */
    MemReqPtr memReq;

    /** Decode to fetch delay, in ticks. */
    unsigned decodeToFetchDelay;

    /** Rename to fetch delay, in ticks. */
    unsigned renameToFetchDelay;

    /** IEW to fetch delay, in ticks. */
    unsigned iewToFetchDelay;

    /** Commit to fetch delay, in ticks. */
    unsigned commitToFetchDelay;

    /** The width of fetch in instructions. */
    unsigned fetchWidth;

    /** Cache block size. */
    int cacheBlkSize;

    /** Mask to get a cache block's address. */
    Addr cacheBlkMask;

    /** The cache line being fetched. */
    uint8_t *cacheData;

    /** Size of instructions. */
    int instSize;

    /** Icache stall statistics. */
    Counter lastIcacheStall;

    Stats::Scalar<> icacheStallCycles;
    Stats::Scalar<> fetchedInsts;
    Stats::Scalar<> predictedBranches;
    Stats::Scalar<> fetchCycles;
    Stats::Scalar<> fetchSquashCycles;
    Stats::Scalar<> fetchBlockedCycles;
    Stats::Scalar<> fetchedCacheLines;

    Stats::Distribution<> fetch_nisn_dist;
};

#endif //__CPU_BETA_CPU_SIMPLE_FETCH_HH__
