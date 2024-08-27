**LOOKUP FUNCTION**
-

**Inputs**: 
- `Addr vpn`
- `auto id`
- `BaseMMU::Mode mode`
- `bool updateLRU`


`Addr vpn`: the virtual page number that is *already extracted from the virtual address*. First component of the tag of TLBEntry. `Addr` is an alias for `uint64_t`. This value will be  47 or 27 bits. 

`auto id`: process context id, address space id, etc. Second component of the tag of TLBEntry. `auto` is currently used because it is TBD on whether `id` will be `uint64_t` or `uint16_t`.


`BaseMMU::Mode mode`: reason for lookup. Indicates whether lookup function call will be followed by a read or write. This helps in determining TLB misses/hits

`bool updateLRU`: determines whether replacement information will be updated. `mode` and observation of TLB write/read hits/misses only done if `updateLRU` is enabled.

**Return**
- `TLBEntry *entry`

**Structure of Function**
- key is determined by performing bitwise or between `vpn` and `id`
- key is the paramater for calling `AssociativeCache` function `findEntry(Addr addr)` 
- if `updateLRU` is enabled, the replacement information is updated, and checks are done to record TLB hits/misses
-entry is returned

**New Functions Created / Old Functions Overrided**
 - in `gem5-ATRemodeling/src/mem/cache/tags/indexing_policies/tlb_set_associative.cc` : 
 `extractTag` just returns the input. `extractTag` is called in AC's `getTag` and `findEntry`
 - in  `gem5-ATRemodeling/src/mem/cache/tags/indexing_policies/tlb_set_associative.cc`:
