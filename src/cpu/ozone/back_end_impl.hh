/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 */

#include "cpu/ozone/back_end.hh"
#include "encumbered/cpu/full/op_class.hh"

template <class Impl>
BackEnd<Impl>::InstQueue::InstQueue(Params *params)
    : size(params->numIQEntries), numInsts(0), width(params->issueWidth)
{
}

template <class Impl>
std::string
BackEnd<Impl>::InstQueue::name() const
{
    return be->name() + ".iq";
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::regStats()
{
    using namespace Stats;

    occ_dist
        .init(1, 0, size, 2)
        .name(name() + "occ_dist")
        .desc("IQ Occupancy per cycle")
        .flags(total | cdf)
        ;

    inst_count
        .init(1)
        .name(name() + "cum_num_insts")
        .desc("Total occupancy")
        .flags(total)
        ;

    peak_inst_count
        .init(1)
        .name(name() + "peak_occupancy")
        .desc("Peak IQ occupancy")
        .flags(total)
        ;

    current_count
        .name(name() + "current_count")
        .desc("Occupancy this cycle")
        ;

    empty_count
        .name(name() + "empty_count")
        .desc("Number of empty cycles")
        ;

    fullCount
        .name(name() + "full_count")
        .desc("Number of full cycles")
        ;


    occ_rate
        .name(name() + "occ_rate")
        .desc("Average occupancy")
        .flags(total)
        ;
    occ_rate = inst_count / be->cpu->numCycles;

    avg_residency
        .name(name() + "avg_residency")
        .desc("Average IQ residency")
        .flags(total)
        ;
    avg_residency = occ_rate / be->cpu->numCycles;

    empty_rate
        .name(name() + "empty_rate")
        .desc("Fraction of cycles empty")
        ;
    empty_rate = 100 * empty_count / be->cpu->numCycles;

    full_rate
        .name(name() + "full_rate")
        .desc("Fraction of cycles full")
        ;
    full_rate = 100 * fullCount / be->cpu->numCycles;
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::setIssueExecQueue(TimeBuffer<IssueToExec> *i2e_queue)
{
    i2e = i2e_queue;
    numIssued = i2e->getWire(0);
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::insert(DynInstPtr &inst)
{
    numInsts++;
    inst_count[0]++;
    if (!inst->isNonSpeculative()) {
        DPRINTF(BE, "Instruction [sn:%lli] added to IQ\n", inst->seqNum);
        if (inst->readyToIssue()) {
            toBeScheduled.push_front(inst);
            inst->iqIt = toBeScheduled.begin();
            inst->iqItValid = true;
        } else {
            iq.push_front(inst);
            inst->iqIt = iq.begin();
            inst->iqItValid = true;
        }
    } else {
        DPRINTF(BE, "Nonspeculative instruction [sn:%lli] added to IQ\n", inst->seqNum);
        nonSpec.push_front(inst);
        inst->iqIt = nonSpec.begin();
        inst->iqItValid = true;
    }
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::scheduleReadyInsts()
{
    int scheduled = numIssued->size;
    InstListIt iq_it = --toBeScheduled.end();
    InstListIt iq_end_it = toBeScheduled.end();

    while (iq_it != iq_end_it && scheduled < width) {
//        if ((*iq_it)->readyToIssue()) {
            DPRINTF(BE, "Instruction [sn:%lli] PC:%#x is ready\n",
                    (*iq_it)->seqNum, (*iq_it)->readPC());
            readyQueue.push(*iq_it);
            readyList.push_front(*iq_it);

            (*iq_it)->iqIt = readyList.begin();

            toBeScheduled.erase(iq_it--);

            ++scheduled;
//        } else {
//            iq_it++;
//        }
    }

    numIssued->size+= scheduled;
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::scheduleNonSpec(const InstSeqNum &sn)
{
/*
    InstListIt non_spec_it = nonSpec.begin();
    InstListIt non_spec_end_it = nonSpec.end();

    while ((*non_spec_it)->seqNum != sn) {
        non_spec_it++;
        assert(non_spec_it != non_spec_end_it);
    }
*/
    DynInstPtr inst = nonSpec.back();

    DPRINTF(BE, "Nonspeculative instruction [sn:%lli] scheduled\n", inst->seqNum);

    assert(inst->seqNum == sn);

    assert(find(NonSpec, inst->iqIt));
    nonSpec.erase(inst->iqIt);
    readyList.push_front(inst);
    inst->iqIt = readyList.begin();
    readyQueue.push(inst);
    numIssued->size++;
}

template <class Impl>
typename Impl::DynInstPtr
BackEnd<Impl>::InstQueue::getReadyInst()
{
    assert(!readyList.empty());

    DynInstPtr inst = readyQueue.top();
    readyQueue.pop();
    assert(find(ReadyList, inst->iqIt));
    readyList.erase(inst->iqIt);
    inst->iqItValid = false;
//    if (!inst->isMemRef())
        --numInsts;
    return inst;
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::squash(const InstSeqNum &sn)
{
    InstListIt iq_it = iq.begin();
    InstListIt iq_end_it = iq.end();

    while (iq_it != iq_end_it && (*iq_it)->seqNum > sn) {
        DPRINTF(BE, "Instruction [sn:%lli] removed from IQ\n", (*iq_it)->seqNum);
        (*iq_it)->iqItValid = false;
        iq.erase(iq_it++);
        --numInsts;
    }

    iq_it = nonSpec.begin();
    iq_end_it = nonSpec.end();

    while (iq_it != iq_end_it && (*iq_it)->seqNum > sn) {
        DPRINTF(BE, "Instruction [sn:%lli] removed from IQ\n", (*iq_it)->seqNum);
        (*iq_it)->iqItValid = false;
        nonSpec.erase(iq_it++);
        --numInsts;
    }

    iq_it = replayList.begin();
    iq_end_it = replayList.end();

    while (iq_it != iq_end_it) {
        if ((*iq_it)->seqNum > sn) {
            DPRINTF(BE, "Instruction [sn:%lli] removed from IQ\n", (*iq_it)->seqNum);
            (*iq_it)->iqItValid = false;
            replayList.erase(iq_it++);
            --numInsts;
        } else {
            iq_it++;
        }
    }

    assert(numInsts >= 0);
/*
    InstListIt ready_it = readyList.begin();
    InstListIt ready_end_it = readyList.end();

    while (ready_it != ready_end_it) {
        if ((*ready_it)->seqNum > sn) {
            readyList.erase(ready_it++);
        } else {
            ready_it++;
        }
    }
*/
}

template <class Impl>
int
BackEnd<Impl>::InstQueue::wakeDependents(DynInstPtr &inst)
{
    assert(!inst->isSquashed());
    std::vector<DynInstPtr> &dependents = inst->getDependents();
    int num_outputs = dependents.size();

    DPRINTF(BE, "Waking instruction [sn:%lli] dependents in IQ\n", inst->seqNum);

    for (int i = 0; i < num_outputs; i++) {
        DynInstPtr dep_inst = dependents[i];
        dep_inst->markSrcRegReady();
        DPRINTF(BE, "Marking source reg ready [sn:%lli] in IQ\n", dep_inst->seqNum);

        if (dep_inst->readyToIssue() && dep_inst->iqItValid) {
            if (dep_inst->isNonSpeculative()) {
                assert(find(NonSpec, dep_inst->iqIt));
                nonSpec.erase(dep_inst->iqIt);
            } else {
                assert(find(IQ, dep_inst->iqIt));
                iq.erase(dep_inst->iqIt);
            }

            toBeScheduled.push_front(dep_inst);
            dep_inst->iqIt = toBeScheduled.begin();
        }
    }
    return num_outputs;
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::rescheduleMemInst(DynInstPtr &inst)
{
    DPRINTF(BE, "Rescheduling memory instruction [sn:%lli]\n", inst->seqNum);
    assert(!inst->iqItValid);
    replayList.push_front(inst);
    inst->iqIt = replayList.begin();
    inst->iqItValid = true;
    ++numInsts;
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::replayMemInst(DynInstPtr &inst)
{
    DPRINTF(BE, "Replaying memory instruction [sn:%lli]\n", inst->seqNum);
    assert(find(ReplayList, inst->iqIt));
    InstListIt iq_it = --replayList.end();
    InstListIt iq_end_it = replayList.end();
    while (iq_it != iq_end_it) {
        DynInstPtr rescheduled_inst = (*iq_it);

        DPRINTF(BE, "Memory instruction [sn:%lli] also replayed\n", inst->seqNum);
        replayList.erase(iq_it--);
        toBeScheduled.push_front(rescheduled_inst);
        rescheduled_inst->iqIt = toBeScheduled.begin();
    }
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::completeMemInst(DynInstPtr &inst)
{
    panic("Not implemented.");
}

template <class Impl>
bool
BackEnd<Impl>::InstQueue::find(queue q, InstListIt it)
{
    InstListIt iq_it, iq_end_it;
    switch(q) {
      case NonSpec:
        iq_it = nonSpec.begin();
        iq_end_it = nonSpec.end();
        break;
      case IQ:
        iq_it = iq.begin();
        iq_end_it = iq.end();
        break;
      case ToBeScheduled:
        iq_it = toBeScheduled.begin();
        iq_end_it = toBeScheduled.end();
        break;
      case ReadyList:
        iq_it = readyList.begin();
        iq_end_it = readyList.end();
        break;
      case ReplayList:
        iq_it = replayList.begin();
        iq_end_it = replayList.end();
    }

    while (iq_it != it && iq_it != iq_end_it) {
        iq_it++;
    }
    if (iq_it == it) {
        return true;
    } else {
        return false;
    }
}

template <class Impl>
void
BackEnd<Impl>::InstQueue::dumpInsts()
{
    cprintf("IQ size: %i\n", iq.size());

    InstListIt inst_list_it = --iq.end();

    int num = 0;
    int valid_num = 0;
    while (inst_list_it != iq.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it--;
        ++num;
    }

    cprintf("nonSpec size: %i\n", nonSpec.size());

    inst_list_it = --nonSpec.end();

    while (inst_list_it != nonSpec.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it--;
        ++num;
    }

    cprintf("toBeScheduled size: %i\n", toBeScheduled.size());

    inst_list_it = --toBeScheduled.end();

    while (inst_list_it != toBeScheduled.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it--;
        ++num;
    }

    cprintf("readyList size: %i\n", readyList.size());

    inst_list_it = --readyList.end();

    while (inst_list_it != readyList.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it--;
        ++num;
    }
}

template<class Impl>
BackEnd<Impl>::LdWritebackEvent::LdWritebackEvent(DynInstPtr &_inst,
                                                  BackEnd<Impl> *_be)
    : Event(&mainEventQueue), inst(_inst), be(_be)
{
    this->setFlags(Event::AutoDelete);
}

template<class Impl>
void
BackEnd<Impl>::LdWritebackEvent::process()
{
    DPRINTF(BE, "Load writeback event [sn:%lli]\n", inst->seqNum);
//    DPRINTF(Activity, "Activity: Ld Writeback event [sn:%lli]\n", inst->seqNum);

    //iewStage->ldstQueue.removeMSHR(inst->threadNumber,inst->seqNum);

//    iewStage->wakeCPU();

    if (inst->isSquashed()) {
        inst = NULL;
        return;
    }

    if (!inst->isExecuted()) {
        inst->setExecuted();

        // Execute again to copy data to proper place.
        inst->completeAcc();
    }

    // Need to insert instruction into queue to commit
    be->instToCommit(inst);

    //wroteToTimeBuffer = true;
//    iewStage->activityThisCycle();

    inst = NULL;
}

template<class Impl>
const char *
BackEnd<Impl>::LdWritebackEvent::description() const
{
    return "Load writeback";
}


template <class Impl>
BackEnd<Impl>::DCacheCompletionEvent::DCacheCompletionEvent(BackEnd *_be)
    : Event(&mainEventQueue, CPU_Tick_Pri), be(_be)
{
}

template <class Impl>
void
BackEnd<Impl>::DCacheCompletionEvent::process()
{
}

template <class Impl>
const char *
BackEnd<Impl>::DCacheCompletionEvent::description() const
{
    return "Cache completion";
}

template <class Impl>
BackEnd<Impl>::BackEnd(Params *params)
    : d2i(5, 5), i2e(5, 5), e2c(5, 5), numInstsToWB(5, 5),
      xcSquash(false), IQ(params),
      cacheCompletionEvent(this), width(params->backEndWidth),
      exactFullStall(true)
{
    numROBEntries = params->numROBEntries;
    numInsts = 0;
    numDispatchEntries = 32;
    IQ.setBE(this);
    LSQ.setBE(this);

    // Setup IQ and LSQ with their parameters here.
    instsToDispatch = d2i.getWire(-1);

    instsToExecute = i2e.getWire(-1);

    IQ.setIssueExecQueue(&i2e);

    dispatchWidth = params->dispatchWidth ? params->dispatchWidth : width;
    issueWidth = params->issueWidth ? params->issueWidth : width;
    wbWidth = params->wbWidth ? params->wbWidth : width;
    commitWidth = params->commitWidth ? params->commitWidth : width;

    LSQ.init(params, params->LQEntries, params->SQEntries, 0);

    dispatchStatus = Running;
}

template <class Impl>
std::string
BackEnd<Impl>::name() const
{
    return cpu->name() + ".backend";
}

template <class Impl>
void
BackEnd<Impl>::regStats()
{
    using namespace Stats;
    rob_cap_events
        .init(cpu->numThreads)
        .name(name() + ".ROB:cap_events")
        .desc("number of cycles where ROB cap was active")
        .flags(total)
        ;

    rob_cap_inst_count
        .init(cpu->numThreads)
        .name(name() + ".ROB:cap_inst")
        .desc("number of instructions held up by ROB cap")
        .flags(total)
        ;

    iq_cap_events
        .init(cpu->numThreads)
        .name(name() +".IQ:cap_events" )
        .desc("number of cycles where IQ cap was active")
        .flags(total)
        ;

    iq_cap_inst_count
        .init(cpu->numThreads)
        .name(name() + ".IQ:cap_inst")
        .desc("number of instructions held up by IQ cap")
        .flags(total)
        ;


    exe_inst
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:count")
        .desc("number of insts issued")
        .flags(total)
        ;

    exe_swp
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:swp")
        .desc("number of swp insts issued")
        .flags(total)
        ;

    exe_nop
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:nop")
        .desc("number of nop insts issued")
        .flags(total)
        ;

    exe_refs
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:refs")
        .desc("number of memory reference insts issued")
        .flags(total)
        ;

    exe_loads
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:loads")
        .desc("number of load insts issued")
        .flags(total)
        ;

    exe_branches
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:branches")
        .desc("Number of branches issued")
        .flags(total)
        ;

    issued_ops
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:op_count")
        .desc("number of insts issued")
        .flags(total)
        ;

/*
    for (int i=0; i<Num_OpClasses; ++i) {
        stringstream subname;
        subname << opClassStrings[i] << "_delay";
        issue_delay_dist.subname(i, subname.str());
    }
*/
    //
    //  Other stats
    //
    lsq_forw_loads
        .init(cpu->numThreads)
        .name(name() + ".LSQ:forw_loads")
        .desc("number of loads forwarded via LSQ")
        .flags(total)
        ;

    inv_addr_loads
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:addr_loads")
        .desc("number of invalid-address loads")
        .flags(total)
        ;

    inv_addr_swpfs
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:addr_swpfs")
        .desc("number of invalid-address SW prefetches")
        .flags(total)
        ;

    lsq_blocked_loads
        .init(cpu->numThreads)
        .name(name() + ".LSQ:blocked_loads")
        .desc("number of ready loads not issued due to memory disambiguation")
        .flags(total)
        ;

    lsqInversion
        .name(name() + ".ISSUE:lsq_invert")
        .desc("Number of times LSQ instruction issued early")
        ;

    n_issued_dist
        .init(issueWidth + 1)
        .name(name() + ".ISSUE:issued_per_cycle")
        .desc("Number of insts issued each cycle")
        .flags(total | pdf | dist)
        ;
    issue_delay_dist
        .init(Num_OpClasses,0,99,2)
        .name(name() + ".ISSUE:")
        .desc("cycles from operands ready to issue")
        .flags(pdf | cdf)
        ;

    queue_res_dist
        .init(Num_OpClasses, 0, 99, 2)
        .name(name() + ".IQ:residence:")
        .desc("cycles from dispatch to issue")
        .flags(total | pdf | cdf )
        ;
    for (int i = 0; i < Num_OpClasses; ++i) {
        queue_res_dist.subname(i, opClassStrings[i]);
    }

    writeback_count
        .init(cpu->numThreads)
        .name(name() + ".WB:count")
        .desc("cumulative count of insts written-back")
        .flags(total)
        ;

    producer_inst
        .init(cpu->numThreads)
        .name(name() + ".WB:producers")
        .desc("num instructions producing a value")
        .flags(total)
        ;

    consumer_inst
        .init(cpu->numThreads)
        .name(name() + ".WB:consumers")
        .desc("num instructions consuming a value")
        .flags(total)
        ;

    wb_penalized
        .init(cpu->numThreads)
        .name(name() + ".WB:penalized")
        .desc("number of instrctions required to write to 'other' IQ")
        .flags(total)
        ;


    wb_penalized_rate
        .name(name() + ".WB:penalized_rate")
        .desc ("fraction of instructions written-back that wrote to 'other' IQ")
        .flags(total)
        ;

    wb_penalized_rate = wb_penalized / writeback_count;

    wb_fanout
        .name(name() + ".WB:fanout")
        .desc("average fanout of values written-back")
        .flags(total)
        ;

    wb_fanout = producer_inst / consumer_inst;

    wb_rate
        .name(name() + ".WB:rate")
        .desc("insts written-back per cycle")
        .flags(total)
        ;
    wb_rate = writeback_count / cpu->numCycles;

    stat_com_inst
        .init(cpu->numThreads)
        .name(name() + ".COM:count")
        .desc("Number of instructions committed")
        .flags(total)
        ;

    stat_com_swp
        .init(cpu->numThreads)
        .name(name() + ".COM:swp_count")
        .desc("Number of s/w prefetches committed")
        .flags(total)
        ;

    stat_com_refs
        .init(cpu->numThreads)
        .name(name() +  ".COM:refs")
        .desc("Number of memory references committed")
        .flags(total)
        ;

    stat_com_loads
        .init(cpu->numThreads)
        .name(name() +  ".COM:loads")
        .desc("Number of loads committed")
        .flags(total)
        ;

    stat_com_membars
        .init(cpu->numThreads)
        .name(name() +  ".COM:membars")
        .desc("Number of memory barriers committed")
        .flags(total)
        ;

    stat_com_branches
        .init(cpu->numThreads)
        .name(name() + ".COM:branches")
        .desc("Number of branches committed")
        .flags(total)
        ;
    n_committed_dist
        .init(0,commitWidth,1)
        .name(name() + ".COM:committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(pdf)
        ;

    //
    //  Commit-Eligible instructions...
    //
    //  -> The number of instructions eligible to commit in those
    //  cycles where we reached our commit BW limit (less the number
    //  actually committed)
    //
    //  -> The average value is computed over ALL CYCLES... not just
    //  the BW limited cycles
    //
    //  -> The standard deviation is computed only over cycles where
    //  we reached the BW limit
    //
    commit_eligible
        .init(cpu->numThreads)
        .name(name() + ".COM:bw_limited")
        .desc("number of insts not committed due to BW limits")
        .flags(total)
        ;

    commit_eligible_samples
        .name(name() + ".COM:bw_lim_events")
        .desc("number cycles where commit BW limit reached")
        ;

    ROB_fcount
        .name(name() + ".ROB:full_count")
        .desc("number of cycles where ROB was full")
        ;

    ROB_count
        .init(cpu->numThreads)
        .name(name() + ".ROB:occupancy")
        .desc(name() + ".ROB occupancy (cumulative)")
        .flags(total)
        ;

    ROB_full_rate
        .name(name() + ".ROB:full_rate")
        .desc("ROB full per cycle")
        ;
    ROB_full_rate = ROB_fcount / cpu->numCycles;

    ROB_occ_rate
        .name(name() + ".ROB:occ_rate")
        .desc("ROB occupancy rate")
        .flags(total)
        ;
    ROB_occ_rate = ROB_count / cpu->numCycles;

    ROB_occ_dist
        .init(cpu->numThreads, 0, numROBEntries, 2)
        .name(name() + ".ROB:occ_dist")
        .desc("ROB Occupancy per cycle")
        .flags(total | cdf)
        ;

    IQ.regStats();
}

template <class Impl>
void
BackEnd<Impl>::setCommBuffer(TimeBuffer<CommStruct> *_comm)
{
    comm = _comm;
    toIEW = comm->getWire(0);
    fromCommit = comm->getWire(-1);
}

template <class Impl>
void
BackEnd<Impl>::tick()
{
    DPRINTF(BE, "Ticking back end\n");

    ROB_count[0]+= numInsts;

    wbCycle = 0;

    if (xcSquash) {
        squashFromXC();
    }

    // Read in any done instruction information and update the IQ or LSQ.
    updateStructures();

    if (dispatchStatus != Blocked) {
        d2i.advance();
        dispatchInsts();
    } else {
        checkDispatchStatus();
    }

    i2e.advance();
    scheduleReadyInsts();

    e2c.advance();
    executeInsts();

    numInstsToWB.advance();
    writebackInsts();

    commitInsts();

    DPRINTF(BE, "IQ entries in use: %i, ROB entries in use: %i, LSQ loads: %i, LSQ stores: %i\n",
            IQ.numInsts, numInsts, LSQ.numLoads(), LSQ.numStores());

    assert(numInsts == instList.size());
}

template <class Impl>
void
BackEnd<Impl>::updateStructures()
{
    if (fromCommit->doneSeqNum) {
        IQ.commit(fromCommit->doneSeqNum);
        LSQ.commitLoads(fromCommit->doneSeqNum);
        LSQ.commitStores(fromCommit->doneSeqNum);
    }

    if (fromCommit->nonSpecSeqNum) {
        if (fromCommit->uncached) {
            LSQ.executeLoad(fromCommit->lqIdx);
        } else {
            IQ.scheduleNonSpec(
                fromCommit->nonSpecSeqNum);
        }
    }
}

template <class Impl>
void
BackEnd<Impl>::addToIQ(DynInstPtr &inst)
{
    // Do anything IQ specific here?
    IQ.insert(inst);
}

template <class Impl>
void
BackEnd<Impl>::addToLSQ(DynInstPtr &inst)
{
    // Do anything LSQ specific here?
    LSQ.insert(inst);
}

template <class Impl>
void
BackEnd<Impl>::dispatchInsts()
{
    DPRINTF(BE, "Trying to dispatch instructions.\n");

    // Pull instructions out of the front end.
    int disp_width = dispatchWidth ? dispatchWidth : width;

    // Could model dispatching time, but in general 1 cycle is probably
    // good enough.

    if (dispatchSize < numDispatchEntries) {
        for (int i = 0; i < disp_width; i++) {
            // Get instructions
            DynInstPtr inst = frontEnd->getInst();

            if (!inst) {
                // No more instructions to get
                break;
            }

            DPRINTF(BE, "Processing instruction [sn:%lli] PC:%#x\n",
                    inst->seqNum, inst->readPC());

            for (int i = 0; i < inst->numDestRegs(); ++i)
                renameTable[inst->destRegIdx(i)] = inst;

            // Add to queue to be dispatched.
            dispatch.push_back(inst);

            d2i[0].size++;
            ++dispatchSize;
        }
    }

    assert(dispatch.size() < 64);

    for (int i = 0; i < instsToDispatch->size; ++i) {
        assert(!dispatch.empty());
        // Get instruction from front of time buffer
        DynInstPtr inst = dispatch.front();
        dispatch.pop_front();
        --dispatchSize;

        if (inst->isSquashed())
            continue;

        ++numInsts;
        instList.push_back(inst);

        DPRINTF(BE, "Dispatching instruction [sn:%lli] PC:%#x\n",
                inst->seqNum, inst->readPC());

        addToIQ(inst);

        if (inst->isMemRef()) {
            addToLSQ(inst);
        }

        if (inst->isNonSpeculative()) {
            inst->setCanCommit();
        }

        // Check if IQ or LSQ is full.  If so we'll need to break and stop
        // removing instructions.  Also update the number of insts to remove
        // from the queue.
        if (exactFullStall) {
            bool stall = false;
            if (IQ.isFull()) {
                DPRINTF(BE, "IQ is full!\n");
                stall = true;
            } else if (LSQ.isFull()) {
                DPRINTF(BE, "LSQ is full!\n");
                stall = true;
            } else if (isFull()) {
                DPRINTF(BE, "ROB is full!\n");
                stall = true;
                ROB_fcount++;
            }
            if (stall) {
                instsToDispatch->size-= i+1;
                dispatchStall();
                return;
            }
        }
    }

    // Check if IQ or LSQ is full.  If so we'll need to break and stop
    // removing instructions.  Also update the number of insts to remove
    // from the queue.  Check here if we don't care about exact stall
    // conditions.

    bool stall = false;
    if (IQ.isFull()) {
        DPRINTF(BE, "IQ is full!\n");
        stall = true;
    } else if (LSQ.isFull()) {
        DPRINTF(BE, "LSQ is full!\n");
        stall = true;
    } else if (isFull()) {
        DPRINTF(BE, "ROB is full!\n");
        stall = true;
        ROB_fcount++;
    }
    if (stall) {
        d2i.advance();
        dispatchStall();
        return;
    }
}

template <class Impl>
void
BackEnd<Impl>::dispatchStall()
{
    dispatchStatus = Blocked;
    if (!cpu->decoupledFrontEnd) {
        // Tell front end to stall here through a timebuffer, or just tell
        // it directly.
    }
}

template <class Impl>
void
BackEnd<Impl>::checkDispatchStatus()
{
    DPRINTF(BE, "Checking dispatch status\n");
    assert(dispatchStatus == Blocked);
    if (!IQ.isFull() && !LSQ.isFull() && !isFull()) {
        DPRINTF(BE, "Dispatch no longer blocked\n");
        dispatchStatus = Running;
        dispatchInsts();
    }
}

template <class Impl>
void
BackEnd<Impl>::scheduleReadyInsts()
{
    // Tell IQ to put any ready instructions into the instruction list.
    // Probably want to have a list of DynInstPtrs returned here.  Then I
    // can choose to either put them into a time buffer to simulate
    // IQ scheduling time, or hand them directly off to the next stage.
    // Do you ever want to directly hand it off to the next stage?
    DPRINTF(BE, "Trying to schedule ready instructions\n");
    IQ.scheduleReadyInsts();
}

template <class Impl>
void
BackEnd<Impl>::executeInsts()
{
    int insts_to_execute = instsToExecute->size;

    issued_ops[0]+= insts_to_execute;
    n_issued_dist[insts_to_execute]++;

    DPRINTF(BE, "Trying to execute %i instructions\n", insts_to_execute);

    fetchRedirect[0] = false;

    while (insts_to_execute > 0) {
        // Get ready instruction from the IQ (or queue coming out of IQ)
        // Execute the ready instruction.
        // Wakeup any dependents if it's done.
        DynInstPtr inst = IQ.getReadyInst();

        DPRINTF(BE, "Executing inst [sn:%lli] PC: %#x\n",
                inst->seqNum, inst->readPC());

        ++funcExeInst;

        // Check if the instruction is squashed; if so then skip it
        // and don't count it towards the FU usage.
        if (inst->isSquashed()) {
            DPRINTF(BE, "Execute: Instruction was squashed.\n");

            // Not sure how to handle this plus the method of sending # of
            // instructions to use.  Probably will just have to count it
            // towards the bandwidth usage, but not the FU usage.
            --insts_to_execute;

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();

//            ++iewExecSquashedInsts;

            continue;
        }

        Fault fault = NoFault;

        // Execute instruction.
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef() &&
            (!inst->isDataPrefetch() && !inst->isInstPrefetch())) {
            DPRINTF(BE, "Execute: Initiating access for memory "
                    "reference.\n");

            // Tell the LDSTQ to execute this instruction (if it is a load).
            if (inst->isLoad()) {
                // Loads will mark themselves as executed, and their writeback
                // event adds the instruction to the queue to commit
                fault = LSQ.executeLoad(inst);

//                ++iewExecLoadInsts;
            } else if (inst->isStore()) {
                LSQ.executeStore(inst);

//                ++iewExecStoreInsts;

                if (!(inst->req->isLLSC())) {
                    inst->setExecuted();

                    instToCommit(inst);
                }
                // Store conditionals will mark themselves as executed, and
                // their writeback event will add the instruction to the queue
                // to commit.
            } else {
                panic("Unexpected memory type!\n");
            }

        } else {
            inst->execute();

//            ++iewExecutedInsts;

            inst->setExecuted();

            instToCommit(inst);
        }

        updateExeInstStats(inst);

        // Probably should have some sort of function for this.
        // More general question of how to handle squashes?  Have some sort of
        // squash unit that controls it?  Probably...
        // Check if branch was correct.  This check happens after the
        // instruction is added to the queue because even if the branch
        // is mispredicted, the branch instruction itself is still valid.
        // Only handle this if there hasn't already been something that
        // redirects fetch in this group of instructions.

        // This probably needs to prioritize the redirects if a different
        // scheduler is used.  Currently the scheduler schedules the oldest
        // instruction first, so the branch resolution order will be correct.
        ThreadID tid = inst->threadNumber;

        if (!fetchRedirect[tid]) {

            if (inst->mispredicted()) {
                fetchRedirect[tid] = true;

                DPRINTF(BE, "Execute: Branch mispredict detected.\n");
                DPRINTF(BE, "Execute: Redirecting fetch to PC: %#x.\n",
                        inst->nextPC);

                // If incorrect, then signal the ROB that it must be squashed.
                squashDueToBranch(inst);

                if (inst->predTaken()) {
//                    predictedTakenIncorrect++;
                } else {
//                    predictedNotTakenIncorrect++;
                }
            } else if (LSQ.violation()) {
                fetchRedirect[tid] = true;

                // Get the DynInst that caused the violation.  Note that this
                // clears the violation signal.
                DynInstPtr violator;
                violator = LSQ.getMemDepViolator();

                DPRINTF(BE, "LDSTQ detected a violation.  Violator PC: "
                        "%#x, inst PC: %#x.  Addr is: %#x.\n",
                        violator->readPC(), inst->readPC(), inst->physEffAddr);

                // Tell the instruction queue that a violation has occured.
//                IQ.violation(inst, violator);

                // Squash.
//                squashDueToMemOrder(inst,tid);
                squashDueToBranch(inst);

//                ++memOrderViolationEvents;
            } else if (LSQ.loadBlocked()) {
                fetchRedirect[tid] = true;

                DPRINTF(BE, "Load operation couldn't execute because the "
                        "memory system is blocked.  PC: %#x [sn:%lli]\n",
                        inst->readPC(), inst->seqNum);

                squashDueToMemBlocked(inst);
            }
        }

//        instList.pop_front();

        --insts_to_execute;

        // keep an instruction count
        thread->numInst++;
        thread->numInsts++;
    }

    assert(insts_to_execute >= 0);
}

template<class Impl>
void
BackEnd<Impl>::instToCommit(DynInstPtr &inst)
{
    int wb_width = wbWidth;
    // First check the time slot that this instruction will write
    // to.  If there are free write ports at the time, then go ahead
    // and write the instruction to that time.  If there are not,
    // keep looking back to see where's the first time there's a
    // free slot.  What happens if you run out of free spaces?
    // For now naively assume that all instructions take one cycle.
    // Otherwise would have to look into the time buffer based on the
    // latency of the instruction.

    DPRINTF(BE, "Sending instructions to commit [sn:%lli] PC %#x.\n",
            inst->seqNum, inst->readPC());

    while (numInstsToWB[wbCycle].size >= wb_width) {
        ++wbCycle;

        assert(wbCycle < 5);
    }

    // Add finished instruction to queue to commit.
    writeback.push_back(inst);
    numInstsToWB[wbCycle].size++;

    if (wbCycle)
        wb_penalized[0]++;
}

template <class Impl>
void
BackEnd<Impl>::writebackInsts()
{
    int wb_width = wbWidth;
    // Using this method I'm not quite sure how to prevent an
    // instruction from waking its own dependents multiple times,
    // without the guarantee that commit always has enough bandwidth
    // to accept all instructions being written back.  This guarantee
    // might not be too unrealistic.
    InstListIt wb_inst_it = writeback.begin();
    InstListIt wb_end_it = writeback.end();
    int inst_num = 0;
    int consumer_insts = 0;

    for (; inst_num < wb_width &&
             wb_inst_it != wb_end_it; inst_num++) {
        DynInstPtr inst = (*wb_inst_it);

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Uncached loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the uncached load.
        if (!inst->isSquashed()) {
            DPRINTF(BE, "Writing back instruction [sn:%lli] PC %#x.\n",
                    inst->seqNum, inst->readPC());

            inst->setCanCommit();
            inst->setResultReady();

            if (inst->isExecuted()) {
                int dependents = IQ.wakeDependents(inst);
                if (dependents) {
                    producer_inst[0]++;
                    consumer_insts+= dependents;
                }
            }
        }

        writeback.erase(wb_inst_it++);
    }
    LSQ.writebackStores();
    consumer_inst[0]+= consumer_insts;
    writeback_count[0]+= inst_num;
}

template <class Impl>
bool
BackEnd<Impl>::commitInst(int inst_num)
{
    // Read instruction from the head of the ROB
    DynInstPtr inst = instList.front();

    // Make sure instruction is valid
    assert(inst);

    if (!inst->readyToCommit())
        return false;

    DPRINTF(BE, "Trying to commit instruction [sn:%lli] PC:%#x\n",
            inst->seqNum, inst->readPC());

    // If the instruction is not executed yet, then it is a non-speculative
    // or store inst.  Signal backwards that it should be executed.
    if (!inst->isExecuted()) {
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
//        thread->funcExeInst--;

        if (inst->isNonSpeculative()) {
            // Hack to make sure syscalls aren't executed until all stores
            // write back their data.  This direct communication shouldn't
            // be used for anything other than this.
            if (inst_num > 0 || LSQ.hasStoresToWB()) {
                DPRINTF(BE, "Waiting for all stores to writeback.\n");
                return false;
            }

            DPRINTF(BE, "Encountered a store or non-speculative "
                    "instruction at the head of the ROB, PC %#x.\n",
                    inst->readPC());

            // Send back the non-speculative instruction's sequence number.
            toIEW->nonSpecSeqNum = inst->seqNum;

            // Change the instruction so it won't try to commit again until
            // it is executed.
            inst->clearCanCommit();

//            ++commitNonSpecStalls;

            return false;
        } else if (inst->isLoad()) {
            DPRINTF(BE, "[sn:%lli]: Uncached load, PC %#x.\n",
                    inst->seqNum, inst->readPC());

            // Send back the non-speculative instruction's sequence
            // number.  Maybe just tell the lsq to re-execute the load.
            toIEW->nonSpecSeqNum = inst->seqNum;
            toIEW->uncached = true;
            toIEW->lqIdx = inst->lqIdx;

            inst->clearCanCommit();

            return false;
        } else {
            panic("Trying to commit un-executed instruction "
                  "of unknown type!\n");
        }
    }

    // Now check if it's one of the special trap or barrier or
    // serializing instructions.
    if (inst->isThreadSync())
    {
        // Not handled for now.
        panic("Barrier instructions are not handled yet.\n");
    }

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = inst->getFault();

    if (inst_fault != NoFault) {
        if (!inst->isNop()) {
            DPRINTF(BE, "Inst [sn:%lli] PC %#x has a fault\n",
                    inst->seqNum, inst->readPC());

//            assert(!thread->inSyscall);

//            thread->inSyscall = true;

            // Consider holding onto the trap and waiting until the trap event
            // happens for this to be executed.
            inst_fault->invoke(thread->getXCProxy());

            // Exit state update mode to avoid accidental updating.
//            thread->inSyscall = false;

//            commitStatus = TrapPending;

            // Generate trap squash event.
//            generateTrapEvent();

            return false;
        }
    }

    if (inst->isControl()) {
//        ++commitCommittedBranches;
    }

    int freed_regs = 0;

    for (int i = 0; i < inst->numDestRegs(); ++i) {
        DPRINTF(BE, "Commit rename map setting register %i to [sn:%lli]\n",
                (int)inst->destRegIdx(i), inst->seqNum);
        thread->renameTable[inst->destRegIdx(i)] = inst;
        ++freed_regs;
    }

    if (inst->traceData) {
        inst->traceData->finalize();
        inst->traceData = NULL;
    }

    inst->clearDependents();

    frontEnd->addFreeRegs(freed_regs);

    instList.pop_front();

    --numInsts;
    cpu->numInst++;
    thread->numInsts++;
    ++thread->funcExeInst;
    thread->PC = inst->readNextPC();
    updateComInstStats(inst);

    // Write the done sequence number here.
    toIEW->doneSeqNum = inst->seqNum;

    int count = 0;
    Addr oldpc;
    do {
        if (count == 0)
            assert(!thread->inSyscall && !thread->trapPending);
        oldpc = thread->readPC();
        cpu->system->pcEventQueue.service(
            thread->getXCProxy());
        count++;
    } while (oldpc != thread->readPC());
    if (count > 1) {
        DPRINTF(BE, "PC skip function event, stopping commit\n");
//        completed_last_inst = false;
//        squashPending = true;
        return false;
    }
    return true;
}

template <class Impl>
void
BackEnd<Impl>::commitInsts()
{
    int commit_width = commitWidth ? commitWidth : width;

    // Not sure this should be a loop or not.
    int inst_num = 0;
    while (!instList.empty() && inst_num < commit_width) {
        if (instList.front()->isSquashed()) {
            panic("No squashed insts should still be on the list!");
            instList.front()->clearDependents();
            instList.pop_front();
            continue;
        }

        if (!commitInst(inst_num++)) {
            break;
        }
    }
    n_committed_dist.sample(inst_num);
}

template <class Impl>
void
BackEnd<Impl>::squash(const InstSeqNum &sn)
{
    IQ.squash(sn);
    LSQ.squash(sn);

    int freed_regs = 0;
    InstListIt dispatch_end = dispatch.end();
    InstListIt insts_it = dispatch.end();
    insts_it--;

    while (insts_it != dispatch_end && (*insts_it)->seqNum > sn)
    {
        if ((*insts_it)->isSquashed()) {
            --insts_it;
            continue;
        }
        DPRINTF(BE, "Squashing instruction on dispatch list PC %#x, [sn:%lli].\n",
                (*insts_it)->readPC(),
                (*insts_it)->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*insts_it)->setSquashed();

        (*insts_it)->setCanCommit();

        // Be careful with IPRs and such here
        for (int i = 0; i < (*insts_it)->numDestRegs(); ++i) {
            DynInstPtr prev_dest = (*insts_it)->getPrevDestInst(i);
            DPRINTF(BE, "Commit rename map setting register %i to [sn:%lli]\n",
                    (int)(*insts_it)->destRegIdx(i), prev_dest);
            renameTable[(*insts_it)->destRegIdx(i)] = prev_dest;
            ++freed_regs;
        }

        (*insts_it)->clearDependents();

        --insts_it;
    }

    insts_it = instList.end();
    insts_it--;

    while (!instList.empty() && (*insts_it)->seqNum > sn)
    {
        if ((*insts_it)->isSquashed()) {
            --insts_it;
            continue;
        }
        DPRINTF(BE, "Squashing instruction on inst list PC %#x, [sn:%lli].\n",
                (*insts_it)->readPC(),
                (*insts_it)->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*insts_it)->setSquashed();

        (*insts_it)->setCanCommit();

        for (int i = 0; i < (*insts_it)->numDestRegs(); ++i) {
            DynInstPtr prev_dest = (*insts_it)->getPrevDestInst(i);
            DPRINTF(BE, "Commit rename map setting register %i to [sn:%lli]\n",
                    (int)(*insts_it)->destRegIdx(i), prev_dest);
            renameTable[(*insts_it)->destRegIdx(i)] = prev_dest;
            ++freed_regs;
        }

        (*insts_it)->clearDependents();

        instList.erase(insts_it--);
        --numInsts;
    }

    frontEnd->addFreeRegs(freed_regs);
}

template <class Impl>
void
BackEnd<Impl>::squashFromXC()
{
    xcSquash = true;
}

template <class Impl>
void
BackEnd<Impl>::squashDueToBranch(DynInstPtr &inst)
{
    // Update the branch predictor state I guess
    squash(inst->seqNum);
    frontEnd->squash(inst->seqNum, inst->readNextPC(),
                     true, inst->mispredicted());
}

template <class Impl>
void
BackEnd<Impl>::squashDueToMemBlocked(DynInstPtr &inst)
{
    DPRINTF(IEW, "Memory blocked, squashing load and younger insts, "
            "PC: %#x [sn:%i].\n", inst->readPC(), inst->seqNum);

    squash(inst->seqNum - 1);
    frontEnd->squash(inst->seqNum - 1, inst->readPC());
}

template <class Impl>
void
BackEnd<Impl>::fetchFault(Fault &fault)
{
    faultFromFetch = fault;
}

template <class Impl>
void
BackEnd<Impl>::updateExeInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    exe_inst[tid]++;

    //
    //  Control operations
    //
    if (inst->isControl())
        exe_branches[tid]++;

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        exe_refs[tid]++;

        if (inst->isLoad())
            exe_loads[tid]++;
    }
}

template <class Impl>
void
BackEnd<Impl>::updateComInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    //
    //  Pick off the software prefetches
    //
#ifdef TARGET_ALPHA
    if (inst->isDataPrefetch()) {
        stat_com_swp[tid]++;
    } else {
        stat_com_inst[tid]++;
    }
#else
    stat_com_inst[tid]++;
#endif

    //
    //  Control Instructions
    //
    if (inst->isControl())
        stat_com_branches[tid]++;

    //
    //  Memory references
    //
    if (inst->isMemRef()) {
        stat_com_refs[tid]++;

        if (inst->isLoad()) {
            stat_com_loads[tid]++;
        }
    }

    if (inst->isMemBarrier()) {
        stat_com_membars[tid]++;
    }
}

template <class Impl>
void
BackEnd<Impl>::dumpInsts()
{
    int num = 0;
    int valid_num = 0;

    InstListIt inst_list_it = instList.begin();

    cprintf("Inst list size: %i\n", instList.size());

    while (inst_list_it != instList.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it++;
        ++num;
    }

    cprintf("Dispatch list size: %i\n", dispatch.size());

    inst_list_it = dispatch.begin();

    while (inst_list_it != dispatch.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it++;
        ++num;
    }

    cprintf("Writeback list size: %i\n", writeback.size());

    inst_list_it = writeback.begin();

    while (inst_list_it != writeback.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it++;
        ++num;
    }
}
