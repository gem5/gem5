#ifndef __CPU_BETA_CPU_RAS_HH__
#define __CPU_BETA_CPU_RAS_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"

class ReturnAddrStack
{
  public:
    ReturnAddrStack(unsigned numEntries);

    Addr top()
    { return addrStack[tos]; }

    unsigned topIdx()
    { return tos; }

    void push(const Addr &return_addr);

    void pop();

    void restore(unsigned top_entry_idx, const Addr &restored_target);

  private:
    inline void incrTos()
    { if (++tos == numEntries) tos = 0; }

    inline void decrTos()
    { tos = (tos == 0 ? numEntries - 1 : tos - 1); }

    Addr *addrStack;

    unsigned numEntries;

    unsigned usedEntries;

    unsigned tos;
};

#endif // __CPU_BETA_CPU_RAS_HH__
