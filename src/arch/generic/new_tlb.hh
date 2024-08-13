
#ifndef __ARCH_GENERIC_NEW_TLB_HH__
#define __ARCH_GENERIC_NEW_TLB_HH__

#include "base/cache/associative_cache.hh"
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"
#include "base/cache/cache_entry.hh"
#include "arch/generic/new_tlb_entry.hh"

/* General Notes:
  Public vs. Private vs. protected
- Private = only this class can access
- Protected = this class and derived class can access (ex: ISA's that derive from this)
- Public = any class can access

Virtual Function:
- a pure virtual function means that its not implemented in the base class

*/

namespace gem5
{

class NewTLB : public Translator
/*
Types of Functions That Need to Be Added:
- Constructor (including the Translator Paramenters)
- Cache Management (called AC functions this->_cache)
- Translation
  - Transaltion
  - Translation Helping Functions (Permissions, Miss Servicing, Etc.)
  - Different Types of Translation
- Serialization
- Statistics

*/

{
  public:
  protected:
  private:
  public:


};
// This is the NewTLB Entry

// [0] necessary imports
// using the struct TLB Entry to build this
class TLBEntry : public CacheEntry, public Serializable
{

  public:
  // Variables found across the pagetable.hh files across ISA's
  // [1] Variables
    Addr vaddr;  // The virtual address corresponding to this entry
    Addr paddr; // The physical address corresponding to this entry
    bool uncacheable;  // Whether the page is cacheable or not.
    bool writable;         // Read permission is always available, assuming it isn't blocked by other mechanisms.
    unsigned logBytes;  // The size of the page this represents, in address bits.
    uint64_t lruSeq;         // A sequence number to keep track of LRU.

    /* example of some x86 specific variabeles*/
    // bool user;         // Whether this page is accesible without being in supervisor mode, lets the caches handle the writeback policy.
    // bool pwt;
    // bool global;        // Whether or not to kick this page out on a write to CR3.
    // bool patBit;         // A bit used to form an index into the PAT table.
    // bool noExec;         // Whether or not memory on this page can be executed.


  // [2] Constructors
    // change this current implementation to include AC initialization
    TlbEntry(Addr asn, Addr _vaddr, Addr _paddr,
            bool uncacheable, bool read_only);
    TlbEntry();

  // [3] Internal Functions
    void updateVaddr(Addr new_vaddr)
    {
        vaddr = new_vaddr;
    }
    Addr pageStart()
    {
        return paddr;
    }
    int size()     // Return the page size in bytes
    {
        return (1 << logBytes);
    }


// [4] Extended from Serializable
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;


// [5] Extra Functions Extended from Cache Entry

};


} // namespace gem5

#endif
