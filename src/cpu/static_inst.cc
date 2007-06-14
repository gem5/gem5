/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#include <iostream>
#include "cpu/static_inst.hh"
#include "sim/core.hh"

StaticInstPtr StaticInst::nullStaticInstPtr;

// Define the decode cache hash map.
StaticInst::DecodeCache StaticInst::decodeCache;
StaticInst::AddrDecodeCache StaticInst::addrDecodeCache;
StaticInst::cacheElement StaticInst::recentDecodes[2];

void
StaticInst::dumpDecodeCacheStats()
{
    using namespace std;

    cerr << "Decode hash table stats @ " << curTick << ":" << endl;
    cerr << "\tnum entries = " << decodeCache.size() << endl;
    cerr << "\tnum buckets = " << decodeCache.bucket_count() << endl;
    vector<int> hist(100, 0);
    int max_hist = 0;
    for (int i = 0; i < decodeCache.bucket_count(); ++i) {
        int count = decodeCache.elems_in_bucket(i);
        if (count > max_hist)
            max_hist = count;
        hist[count]++;
    }
    for (int i = 0; i <= max_hist; ++i) {
        cerr << "\tbuckets of size " << i << " = " << hist[i] << endl;
    }
}

bool
StaticInst::hasBranchTarget(Addr pc, ThreadContext *tc, Addr &tgt) const
{
    if (isDirectCtrl()) {
        tgt = branchTarget(pc);
        return true;
    }

    if (isIndirectCtrl()) {
        tgt = branchTarget(tc);
        return true;
    }

    return false;
}

StaticInstPtr
StaticInst::fetchMicroop(MicroPC micropc)
{
    panic("StaticInst::fetchMicroop() called on instruction "
            "that is not microcoded.");
}

