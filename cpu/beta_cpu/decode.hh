// Todo:
// Add a couple of the branch fields to DynInst.  Figure out where DynInst
// should try to compute the target of a PC-relative branch.  Try to avoid
// having so many returns within the code.
// Fix up squashing too, as it's too
// dependent upon the iew stage continually telling it to squash.

#ifndef __SIMPLE_DECODE_HH__
#define __SIMPLE_DECODE_HH__

#include <queue>

//Will want to include: time buffer, structs,
#include "base/timebuf.hh"
#include "cpu/beta_cpu/comm.hh"

using namespace std;

template<class Impl>
class SimpleDecode
{
  private:
    // Typedefs from the Impl.
    typedef typename Impl::ISA ISA;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename Impl::FetchStruct FetchStruct;
    typedef typename Impl::DecodeStruct DecodeStruct;
    typedef typename Impl::TimeStruct TimeStruct;

    // Typedefs from the ISA.
    typedef typename ISA::Addr Addr;

  public:
    // The only time decode will become blocked is if dispatch becomes
    // blocked, which means IQ or ROB is probably full.
    enum Status {
        Running,
        Idle,
        Squashing,
        Blocked,
        Unblocking
    };

  private:
    // May eventually need statuses on a per thread basis.
    Status _status;

  public:
    SimpleDecode(Params &params);

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);

    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    void tick();

    void decode();

    // Might want to make squash a friend function.
    void squash();

  private:
    void block();

    inline void unblock();

    void squash(DynInst *inst);

    // Interfaces to objects outside of decode.
    /** CPU interface. */
    FullCPU *cpu;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get rename's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromRename;

    /** Wire to get iew's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write information heading to previous stages. */
    // Might not be the best name as not only fetch will read it.
    typename TimeBuffer<TimeStruct>::wire toFetch;

    /** Decode instruction queue. */
    TimeBuffer<DecodeStruct> *decodeQueue;

    /** Wire used to write any information heading to rename. */
    typename TimeBuffer<DecodeStruct>::wire toRename;

    /** Fetch instruction queue interface. */
    TimeBuffer<FetchStruct> *fetchQueue;

    /** Wire to get fetch's output from fetch queue. */
    typename TimeBuffer<FetchStruct>::wire fromFetch;

    /** Skid buffer between fetch and decode. */
    queue<FetchStruct> skidBuffer;

  private:
    //Consider making these unsigned to avoid any confusion.
    /** Rename to decode delay, in ticks. */
    unsigned renameToDecodeDelay;

    /** IEW to decode delay, in ticks. */
    unsigned iewToDecodeDelay;

    /** Commit to decode delay, in ticks. */
    unsigned commitToDecodeDelay;

    /** Fetch to decode delay, in ticks. */
    unsigned fetchToDecodeDelay;

    /** The width of decode, in instructions. */
    unsigned decodeWidth;
};

#endif // __SIMPLE_DECODE_HH__
