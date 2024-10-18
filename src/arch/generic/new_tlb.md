**NEW TLB Description**
---

What Makes Up the TLB:
- `TLB` class inherits from `Translator`: `src/arch/generic/new_tlb.hh` and `src/arch/generic/new_tlb.cc`
    - `Translator` has a protected `AssociativeCache` member
- `TLBEntry` inherits from `CacheEntry` which inherits from `ReplaceableEntry`: `
- `TLB_Indexing` is the indexing policy that determines how the tag, sets, and entries are managed within the `AssociativeCache`: `src/mem/cache/tags/indexing_policy/tlb_indexing.hh` and `src/mem/cache/tags/indexing_policy/tlb_indexing.cc`

**[1] TLB Class**
--

**Summary of Functions - in new_tlb.cc**

| TLB Functions | Translation Functions | Serialization | Stats |
| --------------|-----------------------|---------------|-------|
| `lookup`      | `translate`           | `serialize`
|`evictLRU`     | `translateAtomic`     | `unserialize`
| `remove`      | `translateFunctional` |
|`flushAll`     |  `translateTiming`    |
|`demapPage`    |
|`insert`       |

**Function Types in new_tlb.cc**

| Implemented in new_tlb.cc | Pure Virtual Functions | Overrides for Associative Cache
| --------------|-----------------------|----------------|
| `lookup`      | `translate`           | `insertEntry`
|`evictLRU`     | `translateAtomic`     | `invalidate`
| `remove`      | `translateFunctional` |
|`flushAll`     |  `translateTiming`    |
| `Constructor` | `demapPage`           |
|               | `insert`             |
|               | `serialize`          |
|               | `unserialize`        |


**Outline**
- Most TLB functions will be using the `AssociativeCache` functions
    - `AssociativeCache` functions are a combinations
- Indexing Policy: newly developed `tlb_indexing`
    - Manages the Associative Cache dat structure
- Replacement Policy: prexisting `lru_rp`
    - Manages the replacement (and eviction) policy for the Associative Cache data structure

** [1a] Implemented Functions**
--

**EvictLRU Function**

Inputs
- `Addr vpn`: takes the virtual page number

Return
- none

Structure of Function
- AC function `findVictim` is called on the address
- `invalidate` is called on the entry returned from `findVictim`

**FlushAll Function**

Inputs: none

Return: none

Structure of Function: AC cache function called `clear`


**Lookup Function**

Inputs:
- `Addr vpn`
- `auto id`
- `BaseMMU::Mode mode`
- `bool updateLRU`


`Addr vpn`: the virtual page number that is *already extracted from the virtual address*. First component of the tag of TLBEntry. `Addr` is an alias for `uint64_t`. This value will be  47 or 27 bits.

`auto id`: process context id, address space id, etc. Second component of the tag of TLBEntry. `auto` is currently used because it is TBD on whether `id` will be `uint64_t` or `uint16_t`.


`BaseMMU::Mode mode`: reason for lookup. Indicates whether lookup function call will be followed by a read or write. This helps in determining TLB misses/hits

`bool updateLRU`: determines whether replacement information will be updated. `mode` and observation of TLB write/read hits/misses only done if `updateLRU` is enabled.

Return: `TLBEntry *entry`

Structure of Function:
- key is determined by performing bitwise or between `vpn` and `id`
- key is the paramater for calling `AssociativeCache` function `findEntry(Addr addr)`
- if `updateLRU` is enabled, the replacement information is updated, and checks are done to record TLB hits/misses
-entry is returned


**Remove Function**

Inputs:
- `Addr vpn`
- `auto id`

`Addr vpn`: the virtual page number that is *already extracted from the virtual address*. First component of the tag of TLBEntry. `Addr` is an alias for `uint64_t`. This value will be  47 or 27 bits.

`auto id`: process context id, address space id, etc. Second component of the tag of TLBEntry. `auto` is currently used because it is TBD on whether `id` will be `uint64_t` or `uint16_t`.

Return: none

Structure of Function:
- `buildKey` called on `vpn` and `id`
- key is the paramater for calling `AssociativeCache` function `findEntry(Addr addr)`
- `invalidate` is called on the entry returned by `findEntry`

**[1b] Pure Virtual Functions**
--
Notes:
- these functions are pure virtual, which means that there is no implementation in the parent class TLB
- each architecture that inherits from this implementation will have to have its implementations

`insert`
- differences in what to do in a Full System situation

`translate`
- looks through the TLB
- if tag is not found in TLB, then page table walk

`translateAtomic`
- passes the translate function using different parameters

`translateFunctional`
- does different processes depending on whether simulation is in FullSystem mode or not

`translateTiming`
- passes the translate function using different parameters

`demapPage`
- varies for x86 and riscv

`serialize`
- relies on the concept of a free list which is removed when using the AssociativeCache

`unserialize`
- relies on the concept of a free list which is removed when using the AssociativeCache


**[1c] Overrided AC Functions**
--
`insertEntry`
- modified to insert something into the sets array

`invalidate`
- modified to set an entry to `NULL` in the sets array


**[2] New TLB Entry**
--
**Summary**
- includes information about the virtual and physical address translation

**Inheritance Structure**
- inherits from `CacheEntry` (which inherits from `ReplaceableEntry`)
- each entry has a `valid` and a `tag` variable
- has functions like `invalidate`, `setTag`, and things like that
- `ReplaceableEntry` has functions for managing set and wat

**Members**
- virtual address
- physical address
- tag
- logBytes (pageBytes)
- uncacheable




**[3a] Indexing Policy: TLB Indexing**
--

Notes:
- inherits from BaseIndexingPolicy
- functions to interact with the `sets` 2-D vector that stores all of the entries in the `AssociativeCache`
- series of functions and variables that manages all of the address shifts in order to retrieve tag, set, etc.
- the "tag" for each entry is the concatenation of the vpn and the id
- the "set" for each entry is the vpn

**`getPossibleEntries`**
- pure virtual function, implemented in this class
- has two parts
- first part is if the TLB is fully associative
- second part is the implementation found in set associative
- this uses extractSet

**`extractTag`**
- override from `BaseIndexingPolicy`
- doesn't do anything, just returns the input
- this is because `extractTag` is called in a lot of Associative Cache

**`extractSet`**
- left as normal
- set index is defined??

**`getVPNfromVA`**
- new function
- shifts the address from the page bytes
- isolate a vpn mask

**`buildKey`**
- takes in vpn and id
- concatenates the two
- this is the tag for an entry

**[3b] Replacement Policy: Least Recently Used**
--
Notes:
- this is a prexisting replacement policy
- updates the replacementData of the entry
- `replacementData` is a shared pointer that belongs to `ReplaceableEntry`

[4] How to Proceed Using This Implementation:
---
- create a new TLB class for each architecture that inherits from this TLB Class
- create a new TLB entry class for each architecture that inherits from this TLB Entry class

[5] Ideas that Still Need to Be Brainstormed:
--
- how to deal with `serialize` and `unserialize` because the `freeList` is removed from implementation
    - `serialize` and `unserialize` are to help dealing with checkpointing
- figuring out how to make the translate function more legible
- seeing if any permision check functions can be generalized
