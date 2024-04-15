/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __CPU_PRED_RAS_HH__
#define __CPU_PRED_RAS_HH__

#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/pred/branch_type.hh"
#include "cpu/static_inst.hh"
#include "params/ReturnAddrStack.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

/** Return address stack class, implements a simple RAS. */
class ReturnAddrStack : public SimObject
{
  public:
    /** Subclass that implements the actual address stack. ******
     */
    class AddrStack
    {
      public:
        AddrStack(ReturnAddrStack &_parent) : parent(_parent) {}

        /** Initializes RAS with a specified number of entries.
         *  @param numEntries Number of entries in the RAS.
         */
        void init(unsigned numEntries);

        void reset();

        /** Returns the top address on the RAS. */
        const PCStateBase *top();

        /** Returns the index of the top of the RAS. */
        unsigned
        topIdx()
        {
            return tos;
        }

        /** Pushes an address onto the RAS. */
        void push(const PCStateBase &return_addr);

        /** Pops the top address from the RAS. */
        void pop();

        /** Changes index to the top of the RAS, and replaces the top address
         *  with a new target.
         *  @param top_of_stack the index saved at the time of the prediction.
         *  @param restored The new target address of the new top of the RAS.
         */
        void restore(unsigned top_of_stack, const PCStateBase *restored);

        bool
        empty()
        {
            return usedEntries == 0;
        }

        bool
        full()
        {
            return usedEntries >= numEntries;
        }

        /** Returns the top n entries of the stack as string. For debugging. */
        std::string toString(int n);

        /** Increments the top of stack index. */
        inline void
        incrTos()
        {
            if (++tos == numEntries)
                tos = 0;
        }

        /** Decrements the top of stack index. */
        inline void
        decrTos()
        {
            tos = (tos == 0 ? numEntries - 1 : tos - 1);
        }

        /** The Stack itself. */
        std::vector<std::unique_ptr<PCStateBase>> addrStack;

        /** The number of entries in the RAS. */
        unsigned numEntries;

        /** The number of used entries in the RAS. */
        unsigned usedEntries;

        /** The top of stack index. */
        unsigned tos;

      protected:
        ReturnAddrStack &parent;
    };

  public:
    // typedef RASParams Params;
    typedef ReturnAddrStackParams Params;

    // ReturnAddrStack(BPredUnit &_parent, const RASParams);
    ReturnAddrStack(const Params &p);

    void reset();

    /**
     * Pushes an address onto the RAS.
     * @param PC The current PC (should be a call).
     * @param ras_history Pointer that will be set to an object that
     * has the return address state associated when the address was pushed.
     */
    void push(ThreadID tid, const PCStateBase &pc, void *&ras_history);

    /**
     * Pops the top address from the RAS.
     * @param ras_history Pointer that will be set to an object that
     * has the return address state associated when an address was poped.
     * @return The address that got poped from the stack.
     *  */
    const PCStateBase *pop(ThreadID tid, void *&ras_history);

    /**
     * The branch (call/return) got squashed.
     * Restores the state of the RAS and delete history
     *  @param res_history The pointer to the history object.
     */
    void squash(ThreadID tid, void *&ras_history);

    /**
     * A branch got finally got finally commited.
     * @param misp Whether the branch was mispredicted.
     * @param brType The type of the branch.
     * @param ras_history The pointer to the history object.
     */
    void commit(ThreadID tid, bool misp, const BranchType brType,
                void *&ras_history);

  private:
    class RASHistory
    {
      public:
        /* Was the RAS pushed or poped for this branch. */
        bool pushed = false;
        bool poped = false;
        /* Was it a call */
        bool wasReturn = false;
        bool wasCall = false;
        /** The entry that poped from the RAS (only valid if a return). */
        std::unique_ptr<PCStateBase> ras_entry;
        /** The RAS index (top of stack pointer) of the instruction */
        unsigned tos = 0;
    };

    void makeRASHistory(void *&ras_history);

    /** The RAS itself. */
    std::vector<AddrStack> addrStacks;

    /** The number of entries in the RAS. */
    unsigned numEntries;
    /** The number of threads */
    unsigned numThreads;

    struct ReturnAddrStackStats : public statistics::Group
    {
        ReturnAddrStackStats(statistics::Group *parent);
        statistics::Scalar pushes;
        statistics::Scalar pops;
        statistics::Scalar squashes;
        statistics::Scalar used;
        statistics::Scalar correct;
        statistics::Scalar incorrect;
    } stats;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_RAS_HH__
