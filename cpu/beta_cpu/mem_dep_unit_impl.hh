
#include <map>

#include "cpu/beta_cpu/mem_dep_unit.hh"

// Hack: dependence predictor sizes are hardcoded.
template <class MemDepPred, class Impl>
MemDepUnit<MemDepPred, Impl>::MemDepUnit(Params &params)
    : depPred(4028, 128)
{
    DPRINTF(MemDepUnit, "MemDepUnit: Creating MemDepUnit object.\n");
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::insert(DynInstPtr &inst)
{
    InstSeqNum inst_seq_num = inst->seqNum;


    InstSeqNum producing_store = depPred.checkInst(inst->readPC());

    if (producing_store == 0 ||
        dependencies.find(producing_store) == dependencies.end()) {
        readyInsts.insert(inst_seq_num);
    } else {
        // If it's not already ready, then add it to the renamed
        // list and the dependencies.
        renamedInsts.insert(inst_seq_num);

        dependencies[producing_store].push_back(inst_seq_num);
    }

    if (inst->isStore()) {
        depPred.insertStore(inst->readPC(), inst_seq_num);

        // Make sure this store isn't already in this list.
        assert(dependencies.find(inst_seq_num) == dependencies.end());

        // Put a dependency entry in at the store's sequence number.
        // Uh, not sure how this works...I want to create an entry but
        // I don't have anything to put into the value yet.
        dependencies[inst_seq_num];
    } else if (!inst->isLoad()) {
        panic("MemDepUnit: Unknown type! (most likely a barrier).");
    }
}

template <class MemDepPred, class Impl>
bool
MemDepUnit<MemDepPred, Impl>::readyToIssue(DynInstPtr &inst)
{
    InstSeqNum inst_seq_num = inst->seqNum;

    if (readyInsts.find(inst_seq_num) == readyInsts.end()) {
        return false;
    } else {
        return true;
    }
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::issue(DynInstPtr &inst)
{
    assert(readyInsts.find(inst->seqNum) != readyInsts.end());

    // Remove the instruction from the ready list.
    readyInsts.erase(inst->seqNum);
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::wakeDependents(DynInstPtr &inst)
{
    // Wake any dependencies.
    dep_it_t dep_it = dependencies.find(inst);

    // If there's no entry, then return.  Really there should only be
    // no entry if the instruction is a load.
    if (dep_it == dependencies.end()) {
        return;
    }

    assert(inst->isStore());

    for(int i = 0; i < (*dep_it).second.size(); ++i ) {
        InstSeqNum woken_inst = (*dep_it).second[i];

        // Should we have reached instructions that are actually squashed,
        // there will be no more useful instructions in this dependency
        // list.  Break out early.
        if (renamedInsts.find(woken_inst) == renamedInsts.end()) {
            DPRINTF(MemDepUnit, "MemDepUnit: Dependents on inst PC %#x "
                    "are squashed, starting at SN %i.  Breaking early.\n",
                    inst->readPC(), woken_inst);
            break;
        }

        // Remove it from the renamed instructions.
        renamedInsts.erase(woken_inst);

        // Add it to the ready list.
        readyInsts.insert(woken_inst);
    }

    dependencies.erase(dep_it);
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::squash(const InstSeqNum &squashed_num)
{

    if (!renamedInsts.empty()) {
        sn_it_t renamed_it = renamedInsts.end();

        --renamed_it;

        // Remove entries from the renamed list as long as we haven't reached
        // the end and the entries continue to be younger than the squashed.
        while (!renamedInsts.empty() &&
               (*renamed_it) > squashed_num)
        {
            renamedInsts.erase(renamed_it--);
        }
    }

    if (!readyInsts.empty()) {
        sn_it_t ready_it = readyInsts.end();

        --ready_it;

        // Same for the ready list.
        while (!readyInsts.empty() &&
               (*ready_it) > squashed_num)
        {
            readyInsts.erase(ready_it--);
        }
    }

    if (!dependencies.empty()) {
        dep_it_t dep_it = dependencies.end();

        --dep_it;

        // Same for the dependencies list.
        while (!dependencies.empty() &&
               (*dep_it).first > squashed_num)
        {
            dependencies.erase(dep_it--);
        }
    }

    // Tell the dependency predictor to squash as well.
    depPred.squash(squashed_num);
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::violation(DynInstPtr &store_inst,
                                        DynInstPtr &violating_load)
{
    // Tell the memory dependence unit of the violation.
    depPred.violation(violating_load->readPC(), store_inst->readPC());
}
