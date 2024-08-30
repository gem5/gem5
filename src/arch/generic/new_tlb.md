**NEW TLB Description**
--

**Functions**
--

**TLB Functions**
- `lookup`
- `evictLRU`
- `remove`
- `flushAll`
- `demapPage`
- `insert`

**Translation Functions**
- `translate`
- `translateAtomic`
- `translateFunctional`
- `translateTiming`

**Stats**
- calls the stats class of the gem5 one

**Serialization Functions**
- `serialize`
- `unserialize`


**Notes**
- most TLB functions will be using the `AssociativeCache` functions.
- they will use the `lru_rp` replacement policy and the `tlb_set_associative` indexing policy
- the indexing policy is designed to take the virtual address (and hence the virtual page number as input) and to adapt while being fully associative

**Status**
- `insert` left virtual
- most other functions were implemented, using the parameter `id` 


**EvictLRU FUNCTION**
--
-

**Inputs**:
- `Addr addr`: if the current TLB is not fully associative, then this address identifies which set to look for the LRU entry. Otherwise, if the TLB is fully associative, then the `addr` input is irrelevant, and the whole TLB is returned

**Return**
- none

**Structure of Function**
- AC function `findVictim` is called on the address
- `invalidate` is called on the entry returned from `findVictim`

**FlushAll FUNCTION**
-

**Inputs**:
- none

**Return**
- none

**Structure of Function**
- AC cache function called `clear`


**LOOKUP FUNCTION**
--


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



**REMOVE FUNCTION**
-

**Inputs**:
- `Addr vpn`
- `auto id`

`Addr vpn`: the virtual page number that is *already extracted from the virtual address*. First component of the tag of TLBEntry. `Addr` is an alias for `uint64_t`. This value will be  47 or 27 bits.

`auto id`: process context id, address space id, etc. Second component of the tag of TLBEntry. `auto` is currently used because it is TBD on whether `id` will be `uint64_t` or `uint16_t`.

**Return**
- `void`

**Structure of Function**
- key is determined by performing bitwise or between `vpn` and `id`
- key is the paramater for calling `AssociativeCache` function `findEntry(Addr addr)`
- `invalidate` is called on the entry returned by `findEntry`


**New Functions Created / Old Functions Overrided**
 - in `gem5-ATRemodeling/src/mem/cache/tags/indexing_policies/tlb_set_associative.cc` :
 `extractTag` just returns the input. `extractTag` is called in AC's `getTag` and `findEntry`
**Cache Entry Functions **
- `invalidate` calls a function called in `cache_entry.cc`

**Indexing**
--

**TLB SET ASSOCIATIVE INDEX**
--
**`getVPNfromVA`**
- new function
- shifts the address from the page bytes
- isolate a vpn mask

**`extractTag`**
- override from `BaseIndexingPolicy`
- doesn't do anything, just returns the input
- this is because `extractTag` is called in a lot of Associative Cache
- 

**`extractSet`**
- left as normal
- set index is defined?? 

**`getPossibleEntries`**
- pure virtual function, implemented in this class
- has two parts
- first part is if the TLB is fully associative
- second part is the implementation found in set associative
- this uses extractSet


**New TLB Entry**
--
**TLBEntry**
--
**Summary**
- includes information about the virtual and physical address translation

**Inheritance**
---
**Correlation with Cache Entry**
- inherits from `CacheEntry`
- each entry has a `valid` and a `tag` variable
- has functions like `invalidate`, `setTag`, and things like that

**Correlation with Associative Cache**
- a few functions in `Associative Cache` call on the entry functions
- ex: `invalidate`, `insert`, and more

**Correlation with Replaceable Entry**
- inherits from `ReplaceableEntry`
- `ReplaceableEntry` only has the members and functions for `set` and `way`
    - `getSet`, `getWay`, etc.

**Details about TLBEntry**
--
**Specific Members**

**Functions**
