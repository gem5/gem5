
#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ooo_cpu/ea_list.hh"

void
EAList::addAddr(const InstSeqNum &new_sn, const Addr &new_ea)
{
    instEA newEA(new_sn, new_ea);

    eaList.push_back(newEA);
}

void
EAList::clearAddr(const InstSeqNum &sn_to_clear, const Addr &ea_to_clear)
{
    eaListIt list_it = eaList.begin();

    while (list_it != eaList.end() && (*list_it).first != sn_to_clear) {
        assert((*list_it).second == ea_to_clear);
    }
}

bool
EAList::checkConflict(const InstSeqNum &check_sn, const Addr &check_ea) const
{
    const constEAListIt list_it = eaList.begin();

    while (list_it != eaList.end() && (*list_it).first < check_sn) {
        if ((*list_it).second == check_ea) {
            return true;
        }
    }

    return false;
}

void
EAList::clear()
{
    eaList.clear();
}

void
EAList::commit(const InstSeqNum &commit_sn)
{
    while (!eaList.empty() && eaList.front().first <= commit_sn) {
        eaList.pop_front();
    }
}
