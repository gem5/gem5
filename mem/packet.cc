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
 */

/**
 * @file
 * Definition of the Packet Class, a packet is a transaction occuring
 * between a single level of the memory heirarchy (ie L1->L2).
 */
#include "base/misc.hh"
#include "mem/packet.hh"


/** delete the data pointed to in the data pointer. Ok to call to matter how
 * data was allocted. */
void
Packet::deleteData() {
    assert(staticData || dynamicData);
    if (staticData)
        return;

    if (arrayData)
        delete [] data;
    else
        delete data;
}

/** If there isn't data in the packet, allocate some. */
void
Packet::allocate() {
    if (data)
        return;
    assert(!staticData);
    dynamicData = true;
    arrayData = true;
    data = new uint8_t[size];
}

/** Do the packet modify the same addresses. */
bool
Packet::intersect(Packet *p) {
    Addr s1 = addr;
    Addr e1 = addr + size;
    Addr s2 = p->addr;
    Addr e2 = p->addr + p->size;

    if (s1 >= s2 && s1 < e2)
        return true;
    if (e1 >= s2 && e1 < e2)
        return true;
    return false;
}

/** Minimally reset a packet so something like simple cpu can reuse it. */
void
Packet::reset() {
    result = Unknown;
    if (dynamicData) {
       deleteData();
       dynamicData = false;
       arrayData = false;
       time = curTick;
    }
}




bool fixPacket(Packet *func, Packet *timing)
{ panic("Need to implement!"); }
