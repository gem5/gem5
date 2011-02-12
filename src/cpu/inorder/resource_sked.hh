/*
 * Copyright (c) 2010 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 *
 */

#ifndef __CPU_INORDER_RESOURCE_SKED_HH__
#define __CPU_INORDER_RESOURCE_SKED_HH__

#include <vector>
#include <list>

/** ScheduleEntry class represents a single function that an instruction
    wants to do at any pipeline stage. For example, if an instruction
    needs to be decoded and do a branch prediction all in one stage
    then each of those tasks would need it's own ScheduleEntry.

    Each schedule entry corresponds to some resource that the instruction
    wants to interact with.

    The file pipeline_traits.cc shows how a typical instruction schedule is
    made up of these schedule entries.
*/
class ScheduleEntry {
  public:
    ScheduleEntry(int stage_num, int _priority, int res_num, int _cmd = 0,
                  int _idx = 0) :
        stageNum(stage_num), resNum(res_num), cmd(_cmd),
        idx(_idx), priority(_priority)
    { }

    /** Stage number to perform this service. */
    int stageNum;

    /** Resource ID to access */
    int resNum;

    /** See specific resource for meaning */
    unsigned cmd;

    /** See specific resource for meaning */
    unsigned idx;

    /** Some Resources May Need Priority */
    int priority;
};

/** The ResourceSked maintains the complete schedule
    for an instruction. That schedule includes what
    resources an instruction wants to acquire at each
    pipeline stage and is represented by a collection
    of ScheduleEntry objects (described above) that
    must be executed in-order.

    In every pipeline stage, the InOrder model will
    process all entries on the resource schedule for
    that stage and then send the instruction to the next
    stage if and only if the instruction successfully
    completed each ScheduleEntry.
*/
class ResourceSked {
  public:
    typedef std::list<ScheduleEntry*>::iterator SkedIt;

    ResourceSked();

    /** Initializee the current entry pointer to
        pipeline stage 0 and the 1st schedule entry
    */
    void init();

    /** Goes through the remaining stages on the schedule
        and sums all the remaining entries left to be
        processed
    */
    int size();

    /** Is the schedule empty? */
    bool empty();

    /** What is the next task for this instruction schedule? */
    ScheduleEntry* top();

    /** Top() Task is completed, remove it from schedule */
    void pop();

    /** Add To Schedule based on stage num and priority of
        Schedule Entry
    */
    void push(ScheduleEntry* sked_entry);

    /** Add Schedule Entry to be in front of another Entry */
    void pushBefore(ScheduleEntry* sked_entry, int sked_cmd, int sked_cmd_idx);

    /** Print what's left on the instruction schedule */
    void print();

  private:
    /** Current Schedule Entry Pointer */
    SkedIt curSkedEntry;

    /** The Resource Schedule: Resized to Number of Stages in
        the constructor
    */
    std::vector<std::list<ScheduleEntry*> > sked;

    /** Find a place to insert the instruction using  the
        schedule entries priority
    */
    SkedIt findIterByPriority(ScheduleEntry *sked_entry, int stage_num);

    /** Find a place to insert the instruction using a particular command
        to look for.
    */
    SkedIt findIterByCommand(ScheduleEntry *sked_entry, int stage_num,
                             int sked_cmd, int sked_cmd_idx = -1);
};

#endif //__CPU_INORDER_RESOURCE_SKED_HH__
