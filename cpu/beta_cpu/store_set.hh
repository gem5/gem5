#ifndef __STORE_SET_HH__
#define __STORE_SET_HH__

#include <vector>

#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"

class StoreSet
{
  public:
    typedef unsigned SSID;

  public:
    StoreSet(int SSIT_size, int LFST_size);

    void violation(Addr load_PC, Addr store_PC);

    void insertLoad(Addr load_PC, InstSeqNum load_seq_num);

    void insertStore(Addr store_PC, InstSeqNum store_seq_num);

    InstSeqNum checkInst(Addr PC);

    void issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store);

    void squash(InstSeqNum squashed_num);

    void clear();

  private:
    inline int calcIndex(Addr PC)
    { return (PC >> offset_bits) & index_mask; }

    inline SSID calcSSID(Addr PC)
    { return ((PC ^ (PC >> 10)) % LFST_size); }

    SSID *SSIT;

    std::vector<bool> validSSIT;

    InstSeqNum *LFST;

    std::vector<bool> validLFST;

    int *SSCounters;

    int SSIT_size;

    int LFST_size;

    int index_mask;

    // HACK: Hardcoded for now.
    int offset_bits;
};

#endif // __STORE_SET_HH__
