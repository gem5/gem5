
#include <map>

#include "cpu/beta_cpu/mem_dep_unit.hh"

template <class MemDepPred, class Impl>
MemDepUnit<MemDepPred, Impl>::MemDepUnit(Params &params)
    : depPred(params.SSITSize, params.LFSTSize)
{
    DPRINTF(MemDepUnit, "MemDepUnit: Creating MemDepUnit object.\n");
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::regStats()
{
    insertedLoads
        .name(name() + ".memDep.insertedLoads")
        .desc("Number of loads inserted to the mem dependence unit.");

    insertedStores
        .name(name() + ".memDep.insertedStores")
        .desc("Number of stores inserted to the mem dependence unit.");

    conflictingLoads
        .name(name() + ".memDep.conflictingLoads")
        .desc("Number of conflicting loads.");

    conflictingStores
        .name(name() + ".memDep.conflictingStores")
        .desc("Number of conflicting stores.");
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::insert(DynInstPtr &inst)
{
    InstSeqNum inst_seq_num = inst->seqNum;

    Dependency unresolved_dependencies(inst_seq_num);

    InstSeqNum producing_store = depPred.checkInst(inst->readPC());

    if (producing_store == 0 ||
        storeDependents.find(producing_store) == storeDependents.end()) {

        DPRINTF(MemDepUnit, "MemDepUnit: No dependency for inst PC "
                "%#x.\n", inst->readPC());

        unresolved_dependencies.storeDep = storeDependents.end();

        if (inst->readyToIssue()) {
            readyInsts.insert(inst_seq_num);
        } else {
            unresolved_dependencies.memDepReady = true;

            waitingInsts.insert(unresolved_dependencies);
        }
    } else {
        DPRINTF(MemDepUnit, "MemDepUnit: Adding to dependency list; "
                "inst PC %#x is dependent on seq num %i.\n",
                inst->readPC(), producing_store);

        if (inst->readyToIssue()) {
            unresolved_dependencies.regsReady = true;
        }

        // Find the store that this instruction is dependent on.
        sd_it_t store_loc = storeDependents.find(producing_store);

        assert(store_loc != storeDependents.end());

        // Record the location of the store that this instruction is
        // dependent on.
        unresolved_dependencies.storeDep = store_loc;

        // If it's not already ready, then add it to the renamed
        // list and the dependencies.
        dep_it_t inst_loc =
            (waitingInsts.insert(unresolved_dependencies)).first;

        // Add this instruction to the list of dependents.
        (*store_loc).second.push_back(inst_loc);

        assert(!(*store_loc).second.empty());

        if (inst->isLoad()) {
            ++conflictingLoads;
        } else {
            ++conflictingStores;
        }
    }

    if (inst->isStore()) {
        DPRINTF(MemDepUnit, "MemDepUnit: Inserting store PC %#x.\n",
                inst->readPC());

        depPred.insertStore(inst->readPC(), inst_seq_num);

        // Make sure this store isn't already in this list.
        assert(storeDependents.find(inst_seq_num) == storeDependents.end());

        // Put a dependency entry in at the store's sequence number.
        // Uh, not sure how this works...I want to create an entry but
        // I don't have anything to put into the value yet.
        storeDependents[inst_seq_num];

        assert(storeDependents.size() != 0);

        ++insertedStores;

    } else if (inst->isLoad()) {
        ++insertedLoads;
    } else {
        panic("MemDepUnit: Unknown type! (most likely a barrier).");
    }

    memInsts[inst_seq_num] = inst;
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::insertNonSpec(DynInstPtr &inst)
{
    InstSeqNum inst_seq_num = inst->seqNum;

    Dependency non_spec_inst(inst_seq_num);

    non_spec_inst.storeDep = storeDependents.end();

    waitingInsts.insert(non_spec_inst);

    // Might want to turn this part into an inline function or something.
    // It's shared between both insert functions.
    if (inst->isStore()) {
        DPRINTF(MemDepUnit, "MemDepUnit: Inserting store PC %#x.\n",
                inst->readPC());

        depPred.insertStore(inst->readPC(), inst_seq_num);

        // Make sure this store isn't already in this list.
        assert(storeDependents.find(inst_seq_num) == storeDependents.end());

        // Put a dependency entry in at the store's sequence number.
        // Uh, not sure how this works...I want to create an entry but
        // I don't have anything to put into the value yet.
        storeDependents[inst_seq_num];

        assert(storeDependents.size() != 0);

        ++insertedStores;

    } else if (inst->isLoad()) {
        ++insertedLoads;
    } else {
        panic("MemDepUnit: Unknown type! (most likely a barrier).");
    }

    memInsts[inst_seq_num] = inst;
}

template <class MemDepPred, class Impl>
typename Impl::DynInstPtr &
MemDepUnit<MemDepPred, Impl>::top()
{
    topInst = memInsts.find( (*readyInsts.begin()) );

    DPRINTF(MemDepUnit, "MemDepUnit: Top instruction is PC %#x.\n",
            (*topInst).second->readPC());

    return (*topInst).second;
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::pop()
{
    DPRINTF(MemDepUnit, "MemDepUnit: Removing instruction PC %#x.\n",
            (*topInst).second->readPC());

    wakeDependents((*topInst).second);

    issue((*topInst).second);

    memInsts.erase(topInst);

    topInst = memInsts.end();
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::regsReady(DynInstPtr &inst)
{
    DPRINTF(MemDepUnit, "MemDepUnit: Marking registers as ready for "
            "instruction PC %#x.\n",
            inst->readPC());

    InstSeqNum inst_seq_num = inst->seqNum;

    Dependency inst_to_find(inst_seq_num);

    dep_it_t waiting_inst = waitingInsts.find(inst_to_find);

    assert(waiting_inst != waitingInsts.end());

    if ((*waiting_inst).memDepReady) {
        DPRINTF(MemDepUnit, "MemDepUnit: Instruction has its memory "
                "dependencies resolved, adding it to the ready list.\n");

        moveToReady(waiting_inst);
    } else {
        DPRINTF(MemDepUnit, "MemDepUnit: Instruction still waiting on "
                "memory dependency.\n");

        (*waiting_inst).regsReady = true;
    }
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::nonSpecInstReady(DynInstPtr &inst)
{
    DPRINTF(MemDepUnit, "MemDepUnit: Marking non speculative "
            "instruction PC %#x as ready.\n",
            inst->readPC());

    InstSeqNum inst_seq_num = inst->seqNum;

    Dependency inst_to_find(inst_seq_num);

    dep_it_t waiting_inst = waitingInsts.find(inst_to_find);

    assert(waiting_inst != waitingInsts.end());

    moveToReady(waiting_inst);
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::issue(DynInstPtr &inst)
{
    assert(readyInsts.find(inst->seqNum) != readyInsts.end());

    DPRINTF(MemDepUnit, "MemDepUnit: Issuing instruction PC %#x.\n",
            inst->readPC());

    // Remove the instruction from the ready list.
    readyInsts.erase(inst->seqNum);

    depPred.issued(inst->readPC(), inst->seqNum, inst->isStore());
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::wakeDependents(DynInstPtr &inst)
{
    // Only stores have dependents.
    if (!inst->isStore()) {
        return;
    }

    // Wake any dependencies.
    sd_it_t sd_it = storeDependents.find(inst->seqNum);

    // If there's no entry, then return.  Really there should only be
    // no entry if the instruction is a load.
    if (sd_it == storeDependents.end()) {
        DPRINTF(MemDepUnit, "MemDepUnit: Instruction PC %#x, sequence "
                "number %i has no dependents.\n",
                inst->readPC(), inst->seqNum);

        return;
    }

    for (int i = 0; i < (*sd_it).second.size(); ++i ) {
        dep_it_t woken_inst = (*sd_it).second[i];

        DPRINTF(MemDepUnit, "MemDepUnit: Waking up a dependent inst, "
                "sequence number %i.\n",
                (*woken_inst).seqNum);
#if 0
        // Should we have reached instructions that are actually squashed,
        // there will be no more useful instructions in this dependency
        // list.  Break out early.
        if (waitingInsts.find(woken_inst) == waitingInsts.end()) {
            DPRINTF(MemDepUnit, "MemDepUnit: Dependents on inst PC %#x "
                    "are squashed, starting at SN %i.  Breaking early.\n",
                    inst->readPC(), woken_inst);
            break;
        }
#endif

        if ((*woken_inst).regsReady) {
            moveToReady(woken_inst);
        } else {
            (*woken_inst).memDepReady = true;
        }
    }

    storeDependents.erase(sd_it);
}

template <class MemDepPred, class Impl>
void
MemDepUnit<MemDepPred, Impl>::squash(const InstSeqNum &squashed_num)
{

    if (!waitingInsts.empty()) {
        dep_it_t waiting_it = waitingInsts.end();

        --waiting_it;

        // Remove entries from the renamed list as long as we haven't reached
        // the end and the entries continue to be younger than the squashed.
        while (!waitingInsts.empty() &&
               (*waiting_it).seqNum > squashed_num)
        {
            if (!(*waiting_it).memDepReady &&
                (*waiting_it).storeDep != storeDependents.end()) {
                sd_it_t sd_it = (*waiting_it).storeDep;

                // Make sure the iterator that the store has pointing
                // back is actually to this instruction.
                assert((*sd_it).second.back() == waiting_it);

                // Now remove this from the store's list of dependent
                // instructions.
                (*sd_it).second.pop_back();
            }

            waitingInsts.erase(waiting_it--);
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

    if (!storeDependents.empty()) {
        sd_it_t dep_it = storeDependents.end();

        --dep_it;

        // Same for the dependencies list.
        while (!storeDependents.empty() &&
               (*dep_it).first > squashed_num)
        {
            // This store's list of dependent instructions should be empty.
            assert((*dep_it).second.empty());

            storeDependents.erase(dep_it--);
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
    DPRINTF(MemDepUnit, "MemDepUnit: Passing violating PCs to store sets,"
            " load: %#x, store: %#x\n", violating_load->readPC(),
            store_inst->readPC());
    // Tell the memory dependence unit of the violation.
    depPred.violation(violating_load->readPC(), store_inst->readPC());
}

template <class MemDepPred, class Impl>
inline void
MemDepUnit<MemDepPred, Impl>::moveToReady(dep_it_t &woken_inst)
{
    DPRINTF(MemDepUnit, "MemDepUnit: Adding instruction sequence number %i "
            "to the ready list.\n", (*woken_inst).seqNum);

    // Add it to the ready list.
    readyInsts.insert((*woken_inst).seqNum);

    // Remove it from the waiting instructions.
    waitingInsts.erase(woken_inst);
}
