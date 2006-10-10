/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 *          Dave Greene
 */

/**
 * @file
 * Miss Status and Handling Register (MSHR) definitions.
 */

#include <assert.h>
#include <string>
#include <vector>

#include "mem/cache/miss/mshr.hh"
#include "sim/root.hh" // for curTick
#include "sim/host.hh"
#include "base/misc.hh"
#include "mem/cache/cache.hh"

using namespace std;

MSHR::MSHR()
{
    inService = false;
    ntargets = 0;
    threadNum = -1;
}

void
MSHR::allocate(Packet::Command cmd, Addr _addr, int size,
               Packet * &target)
{
    addr = _addr;
    if (target)
    {
        //Have a request, just use it
        pkt = new Packet(target->req, cmd, Packet::Broadcast, size);
        pkt->time = curTick;
        pkt->allocate();
        pkt->senderState = (Packet::SenderState *)this;
        allocateTarget(target);
    }
    else
    {
        //need a request first
        Request * req = new Request();
        req->setPhys(addr, size, 0);
        //Thread context??
        pkt = new Packet(req, cmd, Packet::Broadcast, size);
        pkt->time = curTick;
        pkt->allocate();
        pkt->senderState = (Packet::SenderState *)this;
    }
}

// Since we aren't sure if data is being used, don't copy here.
/**
 * @todo When we have a "global" data flag, might want to copy data here.
 */
void
MSHR::allocateAsBuffer(Packet * &target)
{
    addr = target->getAddr();
    threadNum = 0/*target->req->getThreadNum()*/;
    pkt = new Packet(target->req, target->cmd, -1);
    pkt->allocate();
    pkt->senderState = (Packet::SenderState*)this;
    pkt->time = curTick;
}

void
MSHR::deallocate()
{
    assert(targets.empty());
    assert(ntargets == 0);
    delete pkt;
    pkt = NULL;
    inService = false;
    //allocIter = NULL;
    //readyIter = NULL;
}

/*
 * Adds a target to an MSHR
 */
void
MSHR::allocateTarget(Packet * &target)
{
    //If we append an invalidate and we issued a read to the bus,
    //but now have some pending writes, we need to move
    //the invalidate to before the first non-read
    if (inService && pkt->isRead() && target->isInvalidate()) {
        std::list<Packet *> temp;

        while (!targets.empty()) {
            if (!targets.front()->isRead()) break;
            //Place on top of temp stack
            temp.push_front(targets.front());
            //Remove from targets
            targets.pop_front();
        }

        //Now that we have all the reads off until first non-read, we can
        //place the invalidate on
        targets.push_front(target);

        //Now we pop off the temp_stack and put them back
        while (!temp.empty()) {
            targets.push_front(temp.front());
            temp.pop_front();
        }
    }
    else {
        targets.push_back(target);
    }

    ++ntargets;
    assert(targets.size() == ntargets);
    /**
     * @todo really prioritize the target commands.
     */

    if (!inService && target->isWrite()) {
        pkt->cmd = Packet::WriteReq;
    }
}



void
MSHR::dump()
{
    ccprintf(cerr,
             "inService: %d thread: %d\n"
             "Addr: %x ntargets %d\n"
             "Targets:\n",
             inService, threadNum, addr, ntargets);

    TargetListIterator tar_it = targets.begin();
    for (int i = 0; i < ntargets; i++) {
        assert(tar_it != targets.end());

        ccprintf(cerr, "\t%d: Addr: %x cmd: %d\n",
                 i, (*tar_it)->getAddr(), (*tar_it)->cmdToIndex());

        tar_it++;
    }
    ccprintf(cerr, "\n");
}

MSHR::~MSHR()
{
    if (pkt)
        pkt = NULL;
}
