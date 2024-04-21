#include "cpu/o3/comm.hh"

#include <algorithm>

#include "cpu/o3/dyn_inst.hh"

namespace gem5
{

namespace o3
{

/**
 * Remove instructions belonging to given thread from plain
 * instruction array. Automatically updates the array size.
 */
static void
removeThreadInsts(ThreadID tid, DynInstPtr *insts, int &size)
{
    auto has_tid = [tid] (const DynInstPtr &inst) -> bool {
        return inst->threadNumber == tid;
    };
    DynInstPtr *last = std::remove_if(insts, insts + size, has_tid);
    size = last - insts;
}

void
FetchStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
DecodeStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
RenameStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
}

void
IEWStruct::clearStates(ThreadID tid)
{
    removeThreadInsts(tid, insts, size);
    mispredictInst[tid] = nullptr;
    pc[tid] = nullptr;
    squash[tid] = false;
    branchMispredict[tid] = false;
}

void
TimeStruct::clearStates(ThreadID tid)
{
    // Reset the thread's decode, rename, IEW, and commit structs to
    // their default values.
    decodeInfo[tid] = DecodeComm();
    renameInfo[tid] = RenameComm();
    iewInfo[tid] = IewComm();
    commitInfo[tid] = CommitComm();

    decodeBlock[tid] = false;
    decodeUnblock[tid] = false;
    renameBlock[tid] = false;
    renameUnblock[tid] = false;
    iewBlock[tid] = false;
    iewUnblock[tid] = false;
}

}
}
