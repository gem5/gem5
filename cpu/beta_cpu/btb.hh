#ifndef __CPU_BETA_CPU_BTB_HH__
#define __CPU_BETA_CPU_BTB_HH__

// For Addr type.
#include "arch/alpha/isa_traits.hh"

class DefaultBTB
{
  private:
    struct BTBEntry
    {
        BTBEntry()
            : tag(0), target(0), valid(false)
        {
        }

        Addr tag;
        Addr target;
        bool valid;
    };

  public:
    DefaultBTB(unsigned numEntries, unsigned tagBits,
               unsigned instShiftAmt);

    Addr lookup(const Addr &inst_PC);

    bool valid(const Addr &inst_PC);

    void update(const Addr &inst_PC, const Addr &target_PC);

  private:
    inline unsigned getIndex(const Addr &inst_PC);

    inline Addr getTag(const Addr &inst_PC);

    BTBEntry *btb;

    unsigned numEntries;

    unsigned idxMask;

    unsigned tagBits;

    unsigned tagMask;

    unsigned instShiftAmt;

    unsigned tagShiftAmt;
};

#endif // __CPU_BETA_CPU_BTB_HH__
