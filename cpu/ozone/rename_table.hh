#ifndef __CPU_OZONE_RENAME_TABLE_HH__
#define __CPU_OZONE_RENAME_TABLE_HH__

#include "arch/isa_traits.hh"

/** Rename table that holds the rename of each architectural register to
 *  producing DynInst. Needs to support copying from one table to another.
 */

template <class Impl>
class RenameTable {
  public:
    typedef typename Impl::DynInstPtr DynInstPtr;

    RenameTable();

    void copyFrom(const RenameTable<Impl> &table_to_copy);

    DynInstPtr &operator [] (int index)
    { return table[index]; }

    DynInstPtr table[TheISA::TotalNumRegs];
};

#endif // __CPU_OZONE_RENAME_TABLE_HH__
