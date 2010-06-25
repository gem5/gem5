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

#include "cpu/inorder/resource_sked.hh"
#include "cpu/inorder/pipeline_traits.hh"

#include <vector>
#include <list>
#include <stdio.h>

using namespace std;
using namespace ThePipeline;

ResourceSked::ResourceSked()
{
    sked.resize(NumStages);
}

void
ResourceSked::init()
{
    assert(!sked[0].empty());

    curSkedEntry = sked[0].begin();
}

int
ResourceSked::size()
{
    int total = 0;
    for (int i = 0; i < sked.size(); i++) {
        total += sked[i].size();
    }

    return total;
}

bool
ResourceSked::empty()
{
    return size() == 0;
}

ScheduleEntry*
ResourceSked::top()
{
    assert(size() > 0);

    return *curSkedEntry;
}

void
ResourceSked::pop()
{
    int stage_num = (*curSkedEntry)->stageNum;

    sked[stage_num].erase(curSkedEntry);

    if (!sked[stage_num].empty()) {
        curSkedEntry = sked[stage_num].begin();
    } else {
        int next_stage = stage_num + 1;

        while (next_stage < NumStages) {
            if (sked[next_stage].empty()) {
                next_stage++;
            } else {
                curSkedEntry = sked[next_stage].begin();
                break;
            }
        }
    }
}

void
ResourceSked::push(ScheduleEntry* sked_entry)
{
    int stage_num = sked_entry->stageNum;
    assert(stage_num < NumStages);

    SkedIt pri_iter = findIterByPriority(sked_entry, stage_num);

    sked[stage_num].insert(pri_iter, sked_entry);
}

void
ResourceSked::pushBefore(ScheduleEntry* sked_entry, int sked_cmd,
                         int sked_cmd_idx)
{

    int stage_num = sked_entry->stageNum;
    assert(stage_num < NumStages);

    SkedIt pri_iter = findIterByCommand(sked_entry, stage_num,
                                        sked_cmd, sked_cmd_idx);

    assert(pri_iter != sked[stage_num].end() &&
           "Could not find command to insert in front of.");

    sked[stage_num].insert(pri_iter, sked_entry);
}

ResourceSked::SkedIt
ResourceSked::findIterByPriority(ScheduleEntry* sked_entry, int stage_num)
{
    if (sked[stage_num].empty()) {
        return sked[stage_num].end();
    }

    int priority = sked_entry->priority;

    SkedIt sked_it = sked[stage_num].begin();
    SkedIt sked_end = sked[stage_num].end();

    while (sked_it != sked_end) {
        if ((*sked_it)->priority > priority)
            break;

        sked_it++;
    }

    return sked_it;
}

ResourceSked::SkedIt
ResourceSked::findIterByCommand(ScheduleEntry* sked_entry, int stage_num,
                                int sked_cmd, int sked_cmd_idx)
{
    if (sked[stage_num].empty()) {
        return sked[stage_num].end();
    }

    SkedIt sked_it = sked[stage_num].begin();
    SkedIt sked_end = sked[stage_num].end();

    while (sked_it != sked_end) {
        if ((*sked_it)->cmd == sked_cmd &&
            (sked_cmd_idx != -1) ? (*sked_it)->idx == sked_cmd_idx : true)
            break;

        sked_it++;
    }

    return sked_it;
}

void
ResourceSked::print()
{
    for (int i = 0; i < sked.size(); i++) {
        cprintf("Stage %i\n====\n", i);
        SkedIt sked_it = sked[i].begin();
        SkedIt sked_end = sked[i].end();
        while (sked_it != sked_end) {
            cprintf("\t res:%i cmd:%i idx:%i\n", (*sked_it)->resNum, (*sked_it)->cmd, (*sked_it)->idx);
            sked_it++;
        }
    }
}
