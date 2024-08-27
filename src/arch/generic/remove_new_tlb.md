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
-
