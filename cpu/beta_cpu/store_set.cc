#include "cpu/beta_cpu/store_set.hh"
#include "base/trace.hh"

StoreSet::StoreSet(int _SSIT_size, int _LFST_size)
    : SSIT_size(_SSIT_size), LFST_size(_LFST_size)
{
    DPRINTF(StoreSet, "StoreSet: Creating store set object.\n");

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
StoreSet::violation(Addr load_PC, Addr store_PC)
{
    int load_index = calcIndex(load_PC);
    int store_index = calcIndex(store_PC);

    bool valid_load_SSID = validSSIT[load_index];
    bool valid_store_SSID = validSSIT[store_index];

    if (!valid_load_SSID && !valid_store_SSID) {
        // Calculate a new SSID here.
        SSID new_set = calcSSID(load_PC);

        validSSIT[load_index] = true;

        SSIT[load_index] = new_set;

        validSSIT[store_index] = true;

        SSIT[store_index] = new_set;

        SSCounters[new_set]++;
    } else if (valid_load_SSID && !valid_store_SSID) {
        SSID load_SSID = SSIT[load_index];

        validSSIT[store_index] = true;

        SSIT[store_index] = load_SSID;

        SSCounters[load_SSID]++;
    } else if (!valid_load_SSID && valid_store_SSID) {
        SSID store_SSID = SSIT[store_index];

        validSSIT[load_index] = true;

        SSIT[load_index] = store_SSID;

        // Because we are having a load point to an already existing set,
        // the size of the store set is not incremented.
    } else {
        SSID load_SSID = SSIT[load_index];
        SSID store_SSID = SSIT[store_index];

        int load_SS_size = SSCounters[load_SSID];
        int store_SS_size = SSCounters[store_SSID];

        // If the load has the bigger store set, then assign the store
        // to the same store set as the load.  Otherwise vice-versa.
        if (load_SS_size > store_SS_size) {
            SSIT[store_index] = load_SSID;

            SSCounters[load_SSID]++;
            SSCounters[store_SSID]--;
        } else {
            SSIT[load_index] = store_SSID;

            SSCounters[store_SSID]++;
            SSCounters[load_SSID]--;
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

    if (!validSSIT[index]) {
        // Do nothing if there's no valid entry.
        return;
    } else {
        store_SSID = SSIT[index];

        assert(store_SSID < LFST_size);

        // Update the last store that was fetched with the current one.
        LFST[store_SSID] = store_seq_num;
    }
}

InstSeqNum
StoreSet::checkInst(Addr PC)
{
    int index = calcIndex(PC);

    int inst_SSID;

    if (!validSSIT[index]) {
        // Return 0 if there's no valid entry.
        return 0;
    } else {
        inst_SSID = SSIT[index];

        assert(inst_SSID < LFST_size);

        if (!validLFST[inst_SSID]) {
            return 0;
        } else {
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

    // Make sure the SSIT still has a valid entry for the issued store.
    assert(validSSIT[index]);

    store_SSID = SSIT[index];

    // If the last fetched store in the store set refers to the store that
    // was just issued, then invalidate the entry.
    if (validLFST[store_SSID] && LFST[store_SSID] == issued_seq_num) {
        validLFST[store_SSID] = false;
    }
}

void
StoreSet::squash(InstSeqNum squashed_num)
{
    // Not really sure how to do this well.

    for (int i = 0; i < LFST_size; ++i) {
        if (LFST[i] < squashed_num) {
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

