
#ifndef __MEM_DEP_UNIT_HH__
#define __MEM_DEP_UNIT_HH__

#include <set>
#include <map>

#include "cpu/inst_seq.hh"
#include "base/statistics.hh"

/**
 * Memory dependency unit class.  This holds the memory dependence predictor.
 * As memory operations are issued to the IQ, they are also issued to this
 * unit, which then looks up the prediction as to what they are dependent
 * upon.  This unit must be checked prior to a memory operation being able
 * to issue.  Although this is templated, it's somewhat hard to make a generic
 * memory dependence unit.  This one is mostly for store sets; it will be
 * quite limited in what other memory dependence predictions it can also
 * utilize.  Thus this class should be most likely be rewritten for other
 * dependence prediction schemes.
 */
template <class MemDepPred, class Impl>
class MemDepUnit {
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;

  public:
    MemDepUnit(Params &params);

    void regStats();

    void insert(DynInstPtr &inst);

    void insertNonSpec(DynInstPtr &inst);

    void regsReady(DynInstPtr &inst);

    void nonSpecInstReady(DynInstPtr &inst);

    void issue(DynInstPtr &inst);

    void wakeDependents(DynInstPtr &inst);

    void squash(const InstSeqNum &squashed_num);

    void violation(DynInstPtr &store_inst, DynInstPtr &violating_load);

    // Will want to make this operation relatively fast.  Right now it
    // kind of sucks.
    DynInstPtr &top();

    void pop();

    inline bool empty()
    { return readyInsts.empty(); }

  private:
    typedef typename std::set<InstSeqNum>::iterator sn_it_t;
    typedef typename std::map<InstSeqNum, DynInstPtr>::iterator dyn_it_t;

    // Forward declarations so that the following two typedefs work.
    class Dependency;
    class ltDependency;

    typedef typename std::set<Dependency, ltDependency>::iterator dep_it_t;
    typedef typename std::map<InstSeqNum, vector<dep_it_t> >::iterator
    sd_it_t;

    struct Dependency {
        Dependency(const InstSeqNum &_seqNum)
            : seqNum(_seqNum), regsReady(0), memDepReady(0)
        { }

        Dependency(const InstSeqNum &_seqNum, bool _regsReady,
                   bool _memDepReady)
            : seqNum(_seqNum), regsReady(_regsReady),
              memDepReady(_memDepReady)
        { }

        InstSeqNum seqNum;
        mutable bool regsReady;
        mutable bool memDepReady;
        mutable sd_it_t storeDep;
    };

    struct ltDependency {
        bool operator() (const Dependency &lhs, const Dependency &rhs)
        {
            return lhs.seqNum < rhs.seqNum;
        }
    };


  private:
    inline void moveToReady(dep_it_t &woken_inst);

  private:
    /** List of instructions that have passed through rename, yet are still
     *  waiting on either a memory dependence to resolve or source registers to
     *  become available before they can issue.
     */
    std::set<Dependency, ltDependency> waitingInsts;

    /** List of instructions that have all their predicted memory dependences
     *  resolved and their source registers ready.
     */
    std::set<InstSeqNum> readyInsts;

    // Change this to hold a vector of iterators, which will point to the
    // entry of the waiting instructions.
    /** List of stores' sequence numbers, each of which has a vector of
     *  iterators.  The iterators point to the appropriate node within
     *  waitingInsts that has the depenendent instruction.
     */
    std::map<InstSeqNum, vector<dep_it_t> > storeDependents;

    // For now will implement this as a map...hash table might not be too
    // bad, or could move to something that mimics the current dependency
    // graph.
    std::map<InstSeqNum, DynInstPtr> memInsts;

    // Iterator pointer to the top instruction which has is ready.
    // Is set by the top() call.
    dyn_it_t topInst;

    /** The memory dependence predictor.  It is accessed upon new
     *  instructions being added to the IQ, and responds by telling
     *  this unit what instruction the newly added instruction is dependent
     *  upon.
     */
    MemDepPred depPred;

    Stats::Scalar<> insertedLoads;
    Stats::Scalar<> insertedStores;
    Stats::Scalar<> conflictingLoads;
    Stats::Scalar<> conflictingStores;
};

#endif
