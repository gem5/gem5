#ifndef __RAS_HH__
#define __RAS_HH__

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
    { tos = (tos + 1) % numEntries; }

    inline void decrTos()
    { tos = (tos == 0 ? numEntries - 1 : tos - 1); }

    Addr *addrStack;

    unsigned numEntries;

    unsigned usedEntries;

    unsigned tos;
};

#endif // __RAS_HH__
