#include "cpu/beta_cpu/ras.hh"

ReturnAddrStack::ReturnAddrStack(unsigned _numEntries)
    : numEntries(_numEntries), usedEntries(0),
      tos(0)
{
    addrStack = new Addr[numEntries];

    for (int i = 0; i < numEntries; ++i)
        addrStack[i] = 0;
}

void
ReturnAddrStack::push(const Addr &return_addr)
{
    incrTos();

    addrStack[tos] = return_addr;

    if (usedEntries != numEntries) {
        ++usedEntries;
    }
}

void
ReturnAddrStack::pop()
{
    // Not sure it's possible to really track usedEntries properly.
//    assert(usedEntries > 0);

    if (usedEntries > 0) {
        --usedEntries;
    }

    decrTos();
}

void
ReturnAddrStack::restore(unsigned top_entry_idx,
                         const Addr &restored_target)
{
    tos = top_entry_idx;

    addrStack[tos] = restored_target;
}
