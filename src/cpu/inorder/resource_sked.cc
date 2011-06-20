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

#include <cstdio>
#include <list>
#include <vector>

#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource_sked.hh"
#include "debug/SkedCache.hh"

using namespace std;
using namespace ThePipeline;

ResourceSked::ResourceSked()
{
    stages.resize(NumStages);
}

void
ResourceSked::init()
{
    assert(!stages[0].empty());

    curSkedEntry = stages[0].begin();
}

int
ResourceSked::size()
{
    int total = 0;
    for (int i = 0; i < stages.size(); i++) {
        total += stages[i].size();
    }

    return total;
}

bool
ResourceSked::empty()
{
    return size() == 0;
}


ResourceSked::SkedIt
ResourceSked::begin()
{
    int num_stages = stages.size();
    for (int i = 0; i < num_stages; i++) {
        if (stages[i].size() > 0)
            return stages[i].begin();
    }

    return stages[num_stages - 1].end();
}

ResourceSked::SkedIt
ResourceSked::end()
{
    int num_stages = stages.size();
    return stages[num_stages - 1].end();
}

ResourceSked::SkedIt
ResourceSked::end(int stage_num)
{
    return stages[stage_num].end();
}

ResourceSked::SkedIt
ResourceSked::find(int stage_num, int cmd)
{
    SkedIt stage_it = stages[stage_num].begin();
    SkedIt stage_end = stages[stage_num].end();

    while (stage_it != stage_end) {
        if ((*stage_it)->cmd == cmd)
            return stage_it;
        stage_it++;
    }

    return stages[stage_num].end();
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

    stages[stage_num].erase(curSkedEntry);

    if (!stages[stage_num].empty()) {
        curSkedEntry = stages[stage_num].begin();
    } else {
        int next_stage = stage_num + 1;

        while (next_stage < NumStages) {
            if (stages[next_stage].empty()) {
                next_stage++;
            } else {
                curSkedEntry = stages[next_stage].begin();
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

    stages[stage_num].insert(pri_iter, sked_entry);
}

void
ResourceSked::pushBefore(ScheduleEntry* sked_entry, int sked_cmd,
                         int sked_cmd_idx)
{

    int stage_num = sked_entry->stageNum;
    assert(stage_num < NumStages);

    SkedIt pri_iter = findIterByCommand(sked_entry, stage_num,
                                        sked_cmd, sked_cmd_idx);

    assert(pri_iter != stages[stage_num].end() &&
           "Could not find command to insert in front of.");

    stages[stage_num].insert(pri_iter, sked_entry);
}

ResourceSked::SkedIt
ResourceSked::findIterByPriority(ScheduleEntry* sked_entry, int stage_num)
{
    if (stages[stage_num].empty()) {
        return stages[stage_num].end();
    }

    int priority = sked_entry->priority;

    SkedIt sked_it = stages[stage_num].begin();
    SkedIt sked_end = stages[stage_num].end();

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
    if (stages[stage_num].empty()) {
        return stages[stage_num].end();
    }

    SkedIt sked_it = stages[stage_num].begin();
    SkedIt sked_end = stages[stage_num].end();

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
    for (int i = 0; i < stages.size(); i++) {
        //ccprintf(cerr, "Stage %i\n====\n", i);
        SkedIt sked_it = stages[i].begin();
        SkedIt sked_end = stages[i].end();
        while (sked_it != sked_end) {
            DPRINTF(SkedCache, "\t stage:%i res:%i cmd:%i idx:%i\n",
                    (*sked_it)->stageNum,
                    (*sked_it)->resNum,
                    (*sked_it)->cmd,
                    (*sked_it)->idx);
            sked_it++;
        }
    }
}
