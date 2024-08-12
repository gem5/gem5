
#ifndef __ARCH_GENERIC_NEW_TLB_HH__
#define __ARCH_GENERIC_NEW_TLB_HH__

#include "base/cache/associative_cache.hh"
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

#include "arch/generic/new_tlb_entry.hh"

namespace gem5
{


// notttttt really sure what goes in each of these
class NewTLB : public Translator // inherits from Translator


// every internal function is called from AssociativeCache functions
// instead of AssociativeCache::, would do this->_cache.

// public vs. private vs. protected
/*
- private = something that only this class needs
- protected = something that the future ISA's would need bc they inherit from them
- public = something that it doesn't matter if its protected or not

start with all the variables that are necessary
then move down to all the ones that the ISA's need
private is the actual last resort

*/


{
  public:
    Associate
  protected:

    /*
    size_t size;

    typedef std::list<TlbEntry *> EntryList;
    EntryList freeList;         // free entries

    // not sure if we still need this 
    uint64_t lruSeq;

    // this is riscv
    Walker *walker; 

    // this is x86
    friend class Walker;
    uint32_t configAddress;


    struct TlbStats : public statistics::Group
    {
        TlbStats(statistics::Group *parent);

        statistics::Scalar readHits;
        statistics::Scalar readMisses;
        statistics::Scalar readAccesses;
        statistics::Scalar writeHits;
        statistics::Scalar writeMisses;
        statistics::Scalar writeAccesses;

        statistics::Formula hits;
        statistics::Formula misses;
        statistics::Formula accesses;
    } stats;
    */ 


  private:
    /*
    uint64_t nextSeq() { return ++lruSeq; }

    void evictLRU();

    void remove(size_t idx);

    Fault translate(const RequestPtr &req, ThreadContext *tc,
                    BaseMMU::Translation *translation, BaseMMU::Mode mode,
                    bool &delayed);
    */

  public:


    typedef NewTLBParams Params;

    NewTLB(const Params &params) // this is identical to the params in x86
      : AssociativeCache(name(), p.size, p.assoc,
        p.replacement_policy, p.indexing_policy),
      SimObject(p) // this is the constructor?
    {

    }

    Walker * TLB::getWalker() {
      return walker;
    }

    // Associate Cache specific - think about how to make these more compatible 
    // with the whole translator class stuff
    TLBEntry * lookup(Addr vpn, uint16_t asid, BaseMMU::Mode mode, bool hidden)
        // AssociativeCache::findEntry(const Addr addr)

    TLBEntry * insert(Addr vpn, const TlbEntry &entry);
        // AssociativeCache::insertEntry(const Addr addr, Entry *entry)

    TLB::remove(size_t idx);
        // AssociativeCache::findEntry(const Addr addr)
        // AssociativeCache::invalidate(Entry *entry)
   
    void flushAll(---);
        // AssociateCache::clear()
   
    void evictLRU(---); // private in x86
        // AssociativeCachefindVictim(const Addr addr)

    

    /*
    Fault TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
    Fault TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
    Fault TLB::translateTiming(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
    virtual translate 
    void takeOverFrom(---)
    */


};
//so basically, we have to use 

} // namespace gem5

#endif
