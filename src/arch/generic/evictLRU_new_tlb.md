**EvictLRU FUNCTION**
-

**Inputs**:
- `Addr addr`: if the current TLB is not fully associative, then this address identifies which set to look for the LRU entry. Otherwise, if the TLB is fully associative, then the `addr` input is irrelevant, and the whole TLB is returned

**Return**
- none

**Structure of Function**
- AC function `findVictim` is called on the address
- `invalidate` is called on the entry returned from `findVictim`
