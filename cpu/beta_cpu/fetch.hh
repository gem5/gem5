// Todo: add in statistics, only get the MachInst and let decode actually
// decode, think about SMT fetch,
// fix up branch prediction stuff into one thing,
// Figure out where to advance time buffer.  Add a way to get a
// stage's current status.

#ifndef __SIMPLE_FETCH_HH__
#define __SIMPLE_FETCH_HH__

//Will want to include: time buffer, structs, MemInterface, Event,
//whatever class bzero uses, MemReqPtr

#include "base/timebuf.hh"
#include "sim/eventq.hh"
#include "cpu/pc_event.hh"
#include "cpu/beta_cpu/comm.hh"
#include "mem/mem_interface.hh"

using namespace std;

/**
 * SimpleFetch class to fetch a single instruction each cycle.  SimpleFetch
 * will stall if there's an Icache miss, but otherwise assumes a one cycle
 * Icache hit.  This will be replaced with a more fleshed out class in the
 * future.
 */

template <class Impl>
class SimpleFetch
{
  public:
    /** Typedefs from Impl. */
    typedef typename Impl::ISA ISA;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename Impl::FetchStruct FetchStruct;
    typedef typename Impl::TimeStruct TimeStruct;

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

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);

    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    void tick();

    void fetch();

    void processCacheCompletion();

//  private:
    // Figure out PC vs next PC and how it should be updated
    void squash(Addr newPC);

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

    CacheCompletionEvent cacheCompletionEvent;

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

    // Will probably have this sit in the FullCPU and just pass a pointr in.
    // Simplifies the constructors of all stages.
    /** Internal fetch instruction queue. */
    TimeBuffer<FetchStruct> *fetchQueue;

    //Might be annoying how this name is different than the queue.
    /** Wire used to write any information heading to decode. */
    typename TimeBuffer<FetchStruct>::wire toDecode;

    /** Icache interface. */
    MemInterface *icacheInterface;

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
    int blkSize;

    /** Mask to get a cache block's address. */
    Addr cacheBlockMask;

    /** The instruction being fetched. */
    MachInst inst;

    /** Size of instructions. */
    int instSize;

    /** Icache stall statistics. */
//     Stats::Scalar<> icacheStallCycles;
//     Counter lastIcacheStall;
};

#endif //__SIMPLE_FETCH_HH__
