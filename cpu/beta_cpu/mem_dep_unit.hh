
#ifndef __MEM_DEP_UNIT_HH__
#define __MEM_DEP_UNIT_HH__

#include <set>
#include <map>

#include "cpu/inst_seq.hh"

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
    typedef typename std::set<InstSeqNum>::iterator sn_it_t;
    typedef typename std::map<InstSeqNum, vector<InstSeqNum> >::iterator
    dep_it_t;

  public:
    MemDepUnit(Params &params);

    void insert(DynInstPtr &inst);

    bool readyToIssue(DynInstPtr &inst);

    void issue(DynInstPtr &inst);

    void wakeDependents(DynInstPtr &inst);

    void squash(const InstSeqNum &squashed_num);

    void violation(DynInstPtr &store_inst, DynInstPtr &violating_load);

  private:
    /** List of instructions that have passed through rename, yet are still
     *  waiting on a memory dependence to resolve before they can issue.
     */
    std::set<InstSeqNum> renamedInsts;

    /** List of instructions that have all their predicted memory dependences
     *  resolved.  They are ready in terms of being free of memory
     *  dependences; however they may still have to wait on source registers.
     */
    std::set<InstSeqNum> readyInsts;

    std::map<InstSeqNum, vector<InstSeqNum> > dependencies;

    /** The memory dependence predictor.  It is accessed upon new
     *  instructions being added to the IQ, and responds by telling
     *  this unit what instruction the newly added instruction is dependent
     *  upon.
     */
    MemDepPred depPred;

};

#endif
