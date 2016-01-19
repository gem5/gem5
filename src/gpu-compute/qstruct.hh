/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Brad Beckmann, Marc Orr
 */

#ifndef __Q_STRUCT_HH__
#define __Q_STRUCT_HH__

#include <bitset>
#include <cstdint>

// Maximum number of arguments
static const int KER_NUM_ARGS = 32;
// Kernel argument buffer size
static const int KER_ARGS_LENGTH = 512;

class LdsChunk;
struct NDRange;

// Be very careful of alignment in this structure. The structure
// must compile to the same layout in both 32-bit and 64-bit mode.
struct HsaQueueEntry
{
    // Base pointer for array of instruction pointers
    uint64_t code_ptr;
    // Grid Size (3 dimensions)
    uint32_t gdSize[3];
    // Workgroup Size (3 dimensions)
    uint32_t wgSize[3];
    uint16_t sRegCount;
    uint16_t dRegCount;
    uint16_t cRegCount;
    uint64_t privMemStart;
    uint32_t privMemPerItem;
    uint32_t privMemTotal;
    uint64_t spillMemStart;
    uint32_t spillMemPerItem;
    uint32_t spillMemTotal;
    uint64_t roMemStart;
    uint32_t roMemTotal;
    // Size (in bytes) of LDS
    uint32_t ldsSize;
    // Virtual Memory Id (unused right now)
    uint32_t vmId;

    // Pointer to dependency chain (unused now)
    uint64_t depends;

    // pointer to bool
    uint64_t addrToNotify;
    // pointer to uint32_t
    uint64_t numDispLeft;

    // variables to pass arguments when running in standalone mode,
    // will be removed when run.py and sh.cpp have been updated to
    // use args and offset arrays
    uint64_t arg1;
    uint64_t arg2;
    uint64_t arg3;
    uint64_t arg4;

    // variables to pass arguments when running in cpu+gpu mode
    uint8_t args[KER_ARGS_LENGTH];
    uint16_t offsets[KER_NUM_ARGS];
    uint16_t num_args;
};

// State used to start (or restart) a WF
struct WFContext
{
    // 32 bit values
    // barrier state
    int bar_cnt[VSZ];

    // id (which WF in the WG)
    int cnt;

    // more barrier state
    int max_bar_cnt;
    int old_barrier_cnt;
    int barrier_cnt;

    // More Program Counter Stuff
    uint32_t pc;

    // Program counter of the immediate post-dominator instruction
    uint32_t rpc;

    // WG wide state (I don't see how to avoid redundancy here)
    int cu_id;
    uint32_t wg_id;
    uint32_t barrier_id;

    // 64 bit values (these values depend on the wavefront size)
    // masks
    uint64_t init_mask;
    uint64_t exec_mask;

    // private memory;
    Addr privBase;
    Addr spillBase;

    LdsChunk *ldsChunk;

    /*
     * Kernel wide state
     * This is a hack. This state should be moved through simulated memory
     * during a yield. Though not much is being used here, so it's probably
     * probably not a big deal.
     *
     * Just to add to this comment... The ndr is derived from simulated
     * memory when the cl-runtime allocates an HsaQueueEntry and populates it
     * for a kernel launch. So in theory the runtime should be able to keep
     * that state around. Then a WF can reference it upon restart to derive
     * kernel wide state. The runtime can deallocate the state when the
     * kernel completes.
     */
    NDRange *ndr;
};

// State that needs to be passed between the simulation and simulated app, a
// pointer to this struct can be passed through the depends field in the
// HsaQueueEntry struct
struct HostState
{
    // cl_event* has original HsaQueueEntry for init
    uint64_t event;
};

// Total number of HSA queues
static const int HSAQ_NQUEUES = 8;

// These values will eventually live in memory mapped registers
// and be settable by the kernel mode driver.

// Number of entries in each HSA queue
static const int HSAQ_SIZE = 64;
// Address of first HSA queue index
static const int HSAQ_INDX_BASE = 0x10000ll;
// Address of first HSA queue
static const int HSAQ_BASE = 0x11000ll;
// Suggested start of HSA code
static const int HSA_CODE_BASE = 0x18000ll;

// These are shortcuts for deriving the address of a specific
// HSA queue or queue index
#define HSAQ(n) (HSAQ_BASE + HSAQ_SIZE * sizeof(struct fsaQueue) * n)
#define HSAQE(n,i) (HSAQ_BASE + (HSAQ_SIZE * n + i) * sizeof(struct fsaQueue))
#define HSAQ_RI(n) (HSAQ_INDX_BASE + sizeof(int) * (n * 3 + 0))
#define HSAQ_WI(n) (HSAQ_INDX_BASE + sizeof(int) * (n * 3 + 1))
#define HSAQ_CI(n) (HSAQ_INDX_BASE + sizeof(int) * (n * 3 + 2))

/*
 * Example code for writing to a queue
 *
 * void
 * ToQueue(int n,struct fsaQueue *val)
 * {
 *     int wi = *(int*)HSAQ_WI(n);
 *     int ri = *(int*)HSAQ_RI(n);
 *     int ci = *(int*)HSAQ_CI(n);
 *
 *     if (ci - ri < HSAQ_SIZE) {
 *         (*(int*)HSAQ_CI(n))++;
 *         *(HsaQueueEntry*)(HSAQE(n, (wi % HSAQ_SIZE))) = *val;
 *         (*(int*)HSAQ_WI(n))++;
 *     }
 * }
 */

#endif // __Q_STRUCT_HH__
