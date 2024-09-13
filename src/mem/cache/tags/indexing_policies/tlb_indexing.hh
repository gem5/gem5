#ifndef __MEM__CACHE_INDEXING_POLICIES_TLB_INDEXING_HH__
#define __MEM__CACHE_INDEXING_POLICIES_TLB_INDEXING_HH__

#include <vector>

#include "mem/cache/tags/indexing_policies/base.hh" // parent class
#include "params/TLBIndexing.hh" // need to make this?

namespace gem5 {

class ReplaceableEntry;
/**
 * Description of how this works
 */

class TLBIndexing : public BaseIndexingPolicy
{
  protected:
    /* fill this out */

  public:
  /* Construct and initialize this policy*/
    typedef TLBIndexingParams Params;

    TLBIndexing(const Params &p);

   /* Destructor */
   ~TLBIndexing() {};

   // All of the shifting values
   // Used in extractSet and buildKey
   const int idShift;
   // const unsigned setMask already exists

   // Getting the vpn from virtual address
   // Used in getVPNfromVA
   const int pageShift;
   const unsigned vpnMask;

   /* Pure Virtual from BaseIndexingPolicy */

   /* @params: an address that contains the vpn and the pcid (tag) */
   /* @return: a list of entries in a set */
   std::vector<ReplaceableEntry*> getPossibleEntries(const Addr addr) const override;

   /* useless function - never used */
   Addr regenerateAddr(const Addr tag, const ReplaceableEntry* entry) const override;

   /* Override from BaseIndexingPolicy */

   /* @params: tag 
    * @return: tag */
   Addr extractTag(const Addr addr) override const;

   /* New Functions for TLB Indexing */

   /* @params: [vpn][id] 
    * @return: [vpn] */
   auto TLBSetAssociative::extractSet(const Addr addr) const;

   /* @params: [vpn][page offset] */
   /* @return: [vpn] */
   Addr getVPNfromVA(const Addr addr) const;

   /* @params: vpn, id */
   /* @return: [vpn][id][0's] */ 
   /* key and tag are synonomous */
   Addr buildKey(Addr vpn, auto id) const;



}; // end of the indexing policy

} // namespace gem5

#endif //__MEM_CACHE_INDEXING_POLICIES_TLB_INDEXING_HH__
