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

class ScheduleEntry {
  public:
    ScheduleEntry(int stage_num, int _priority, int res_num, int _cmd = 0,
                  int _idx = 0) :
        stageNum(stage_num), resNum(res_num), cmd(_cmd),
        idx(_idx), priority(_priority)
    { }

    // Stage number to perform this service.
    int stageNum;

    // Resource ID to access
    int resNum;

    // See specific resource for meaning
    unsigned cmd;

    // See specific resource for meaning
    unsigned idx;

    // Some Resources May Need Priority
    int priority;
};

class ResourceSked {
  public:
    typedef std::list<ScheduleEntry*>::iterator SkedIt;

    ResourceSked();

    void init();

    int size();
    bool empty();
    ScheduleEntry* top();
    void pop();
    void push(ScheduleEntry* sked_entry);
    void pushBefore(ScheduleEntry* sked_entry, int sked_cmd, int sked_cmd_idx);
    void print();

  private:
    SkedIt curSkedEntry;
    std::vector<std::list<ScheduleEntry*> > sked;

    SkedIt findIterByPriority(ScheduleEntry *sked_entry, int stage_num);
    SkedIt findIterByCommand(ScheduleEntry *sked_entry, int stage_num,
                             int sked_cmd, int sked_cmd_idx = -1);
};

#endif //__CPU_INORDER_RESOURCE_SKED_HH__
