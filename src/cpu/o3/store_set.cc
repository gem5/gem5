/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "base/trace.hh"
#include "cpu/o3/store_set.hh"

StoreSet::StoreSet(int _SSIT_size, int _LFST_size)
    : SSIT_size(_SSIT_size), LFST_size(_LFST_size)
{
    DPRINTF(StoreSet, "StoreSet: Creating store set object.\n");
    DPRINTF(StoreSet, "StoreSet: SSIT size: %i, LFST size: %i.\n",
            SSIT_size, LFST_size);

    SSIT = new SSID[SSIT_size];

    validSSIT.resize(SSIT_size);

    for (int i = 0; i < SSIT_size; ++i)
        validSSIT[i] = false;

    LFST = new InstSeqNum[LFST_size];

    validLFST.resize(LFST_size);

    SSCounters = new int[LFST_size];

    for (int i = 0; i < LFST_size; ++i)
    {
        validLFST[i] = false;
        SSCounters[i] = 0;
    }

    index_mask = SSIT_size - 1;

    offset_bits = 2;
}

void
StoreSet::violation(Addr store_PC, Addr load_PC)
{
    int load_index = calcIndex(load_PC);
    int store_index = calcIndex(store_PC);

    assert(load_index < SSIT_size && store_index < SSIT_size);

    bool valid_load_SSID = validSSIT[load_index];
    bool valid_store_SSID = validSSIT[store_index];

    if (!valid_load_SSID && !valid_store_SSID) {
        // Calculate a new SSID here.
        SSID new_set = calcSSID(load_PC);

        validSSIT[load_index] = true;

        SSIT[load_index] = new_set;

        validSSIT[store_index] = true;

        SSIT[store_index] = new_set;

        assert(new_set < LFST_size);

        SSCounters[new_set]++;


        DPRINTF(StoreSet, "StoreSet: Neither load nor store had a valid "
                "storeset, creating a new one: %i for load %#x, store %#x\n",
                new_set, load_PC, store_PC);
    } else if (valid_load_SSID && !valid_store_SSID) {
        SSID load_SSID = SSIT[load_index];

        validSSIT[store_index] = true;

        SSIT[store_index] = load_SSID;

        assert(load_SSID < LFST_size);

        SSCounters[load_SSID]++;

        DPRINTF(StoreSet, "StoreSet: Load had a valid store set.  Adding "
                "store to that set: %i for load %#x, store %#x\n",
                load_SSID, load_PC, store_PC);
    } else if (!valid_load_SSID && valid_store_SSID) {
        SSID store_SSID = SSIT[store_index];

        validSSIT[load_index] = true;

        SSIT[load_index] = store_SSID;

        // Because we are having a load point to an already existing set,
        // the size of the store set is not incremented.

        DPRINTF(StoreSet, "StoreSet: Store had a valid store set: %i for "
                "load %#x, store %#x\n",
                store_SSID, load_PC, store_PC);
    } else {
        SSID load_SSID = SSIT[load_index];
        SSID store_SSID = SSIT[store_index];

        assert(load_SSID < LFST_size && store_SSID < LFST_size);

        int load_SS_size = SSCounters[load_SSID];
        int store_SS_size = SSCounters[store_SSID];

        // If the load has the bigger store set, then assign the store
        // to the same store set as the load.  Otherwise vice-versa.
        if (load_SS_size > store_SS_size) {
            SSIT[store_index] = load_SSID;

            SSCounters[load_SSID]++;
            SSCounters[store_SSID]--;

            DPRINTF(StoreSet, "StoreSet: Load had bigger store set: %i; "
                    "for load %#x, store %#x\n",
                    load_SSID, load_PC, store_PC);
        } else {
            SSIT[load_index] = store_SSID;

            SSCounters[store_SSID]++;
            SSCounters[load_SSID]--;

            DPRINTF(StoreSet, "StoreSet: Store had bigger store set: %i; "
                    "for load %#x, store %#x\n",
                    store_SSID, load_PC, store_PC);
        }
    }
}

void
StoreSet::insertLoad(Addr load_PC, InstSeqNum load_seq_num)
{
    // Does nothing.
    return;
}

void
StoreSet::insertStore(Addr store_PC, InstSeqNum store_seq_num)
{
    int index = calcIndex(store_PC);

    int store_SSID;

    assert(index < SSIT_size);

    if (!validSSIT[index]) {
        // Do nothing if there's no valid entry.
        return;
    } else {
        store_SSID = SSIT[index];

        assert(store_SSID < LFST_size);

        // Update the last store that was fetched with the current one.
        LFST[store_SSID] = store_seq_num;

        validLFST[store_SSID] = 1;

        DPRINTF(StoreSet, "Store %#x updated the LFST, SSID: %i\n",
                store_PC, store_SSID);
    }
}

InstSeqNum
StoreSet::checkInst(Addr PC)
{
    int index = calcIndex(PC);

    int inst_SSID;

    assert(index < SSIT_size);

    if (!validSSIT[index]) {
        DPRINTF(StoreSet, "Inst %#x with index %i had no SSID\n",
                PC, index);

        // Return 0 if there's no valid entry.
        return 0;
    } else {
        inst_SSID = SSIT[index];

        assert(inst_SSID < LFST_size);

        if (!validLFST[inst_SSID]) {

            DPRINTF(StoreSet, "Inst %#x with index %i and SSID %i had no "
                    "dependency\n", PC, index, inst_SSID);

            return 0;
        } else {
            DPRINTF(StoreSet, "Inst %#x with index %i and SSID %i had LFST "
                    "inum of %i\n", PC, index, inst_SSID, LFST[inst_SSID]);

            return LFST[inst_SSID];
        }
    }
}

void
StoreSet::issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store)
{
    // This only is updated upon a store being issued.
    if (!is_store) {
        return;
    }

    int index = calcIndex(issued_PC);

    int store_SSID;

    assert(index < SSIT_size);

    // Make sure the SSIT still has a valid entry for the issued store.
    if (!validSSIT[index]) {
        return;
    }

    store_SSID = SSIT[index];

    assert(store_SSID < LFST_size);

    // If the last fetched store in the store set refers to the store that
    // was just issued, then invalidate the entry.
    if (validLFST[store_SSID] && LFST[store_SSID] == issued_seq_num) {
        DPRINTF(StoreSet, "StoreSet: store invalidated itself in LFST.\n");
        validLFST[store_SSID] = false;
    }
}

void
StoreSet::squash(InstSeqNum squashed_num)
{
    // Not really sure how to do this well.
    // Generally this is small enough that it should be okay; short circuit
    // evaluation should take care of invalid entries.

    DPRINTF(StoreSet, "StoreSet: Squashing until inum %i\n",
            squashed_num);

    for (int i = 0; i < LFST_size; ++i) {
        if (validLFST[i] && LFST[i] < squashed_num) {
            validLFST[i] = false;
        }
    }
}

void
StoreSet::clear()
{
    for (int i = 0; i < SSIT_size; ++i) {
        validSSIT[i] = false;
    }

    for (int i = 0; i < LFST_size; ++i) {
        validLFST[i] = false;
    }
}

