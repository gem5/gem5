#ifndef __CPU_EA_LIST_HH__
#define __CPU_EA_LIST_HH__

#include <list>
#include <utility>

#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"

/**
 * Simple class to hold onto a list of pairs, each pair having a memory
 * instruction's sequence number and effective addr.  This list can be used
 * for memory disambiguation.  However, if I ever want to forward results, I
 * may have to use a list that holds DynInstPtrs.  Hence this may change in
 * the future.
 */
class EAList {
  private:
    typedef std::pair<InstSeqNum, Addr> instEA;
    typedef std::list<instEA>::iterator eaListIt;
    typedef std::list<instEA>::const_iterator constEAListIt;

    std::list<instEA> eaList;

  public:
    EAList() { }
    ~EAList() { }

    void addAddr(const InstSeqNum &new_sn, const Addr &new_ea);

    void clearAddr(const InstSeqNum &sn_to_clear, const Addr &ea_to_clear);

    /** Checks if any instructions older than check_sn have a conflicting
     *  address with check_ea.  Note that this function does not handle the
     *  sequence number rolling over.
     */
    bool checkConflict(const InstSeqNum &check_sn, const Addr &check_ea) const;

    void clear();

    void commit(const InstSeqNum &commit_sn);
};

#endif // __CPU_EA_LIST_HH__
