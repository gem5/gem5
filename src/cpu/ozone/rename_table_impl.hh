
#include <cstdlib>  // Not really sure what to include to get NULL
#include "cpu/ozone/rename_table.hh"

template <class Impl>
RenameTable<Impl>::RenameTable()
{
    // Actually should set these to dummy dyn insts that have the initial value
    // and force their values to be initialized.  This keeps everything the
    // same.
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        table[i] = NULL;
    }
}

template <class Impl>
void
RenameTable<Impl>::copyFrom(const RenameTable<Impl> &table_to_copy)
{
    for (int i = 0; i < TheISA::TotalNumRegs; ++i) {
        table[i] = table_to_copy.table[i];
    }
}
