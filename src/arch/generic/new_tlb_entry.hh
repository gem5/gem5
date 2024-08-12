// [0] necessary imports

#include "base/cache/cache_entry.hh"

// Notes:
/*
- does the tlb entry need a cc file
*/

// to be honest, most things needed to be public or protected

// using the struct TLB Entry to build this 

// public vs. private vs. protected
/*
- private = something that only this class needs
- protected = something that the future ISA's would need bc they inherit from them
- public = something that it doesn't matter if its protected or not

start with all the variables that are necessary
then move down to all the ones that the ISA's need
private is the actual last resort

*/
 
// honestly need to make everything in here public
class TLBEntry : public CacheEntry, public Serializable
{ 

  public:
  // these are pretty common variables
  // [1] these are the variables for the cache entry
    Addr vaddr;  // The virtual address corresponding to this entry
    Addr paddr; // The physical address corresponding to this entry
    bool uncacheable;  // Whether the page is cacheable or not.
    bool writable;         // Read permission is always available, assuming it isn't blocked by other mechanisms.
    unsigned logBytes;  // The size of the page this represents, in address bits.
    uint64_t lruSeq;         // A sequence number to keep track of LRU.

    /* a good amount of these are x86 specific*/
    // bool user;         // Whether this page is accesible without being in supervisor mode, lets the caches handle the writeback policy.
    // bool pwt;
    // bool global;        // Whether or not to kick this page out on a write to CR3.
    // bool patBit;         // A bit used to form an index into the PAT table.
    // bool noExec;         // Whether or not memory on this page can be executed.



  // [2] these are the constructors
    // have to get this constructor to match the variables above
    // at some point i will have to initailize the Cache Entry, no idea
    // have to get Cache Entry instea 
    TlbEntry(Addr asn, Addr _vaddr, Addr _paddr, 
            bool uncacheable, bool read_only); 
    TlbEntry();






// [3] these are the internal functions
    // these are internal functions -> have to include this because
    // i am not sure if there will be a .cc file for Cache etry
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





// [4] these are the extension from serializable
    // this is the part that calls on serializable 
    // come back to this! - not entirely sure how to navigate from this
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;





// [5] these are additional functions from cache entry
    // look at the functions from cache entry and then take them

};
