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

#ifndef __CPU_PRED_RAS_HH__
#define __CPU_PRED_RAS_HH__

#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/types.hh"

namespace gem5
{

namespace branch_prediction
{

/** Return address stack class, implements a simple RAS. */
class ReturnAddrStack
{
  public:
    /** Creates a return address stack, but init() must be called prior to
     *  use.
     */
    ReturnAddrStack() {}

    /** Initializes RAS with a specified number of entries.
     *  @param numEntries Number of entries in the RAS.
     */
    void init(unsigned numEntries);

    void reset();

    /** Returns the top address on the RAS. */
    const PCStateBase *top() { return addrStack[tos].get(); }

    /** Returns the index of the top of the RAS. */
    unsigned topIdx() { return tos; }

    /** Pushes an address onto the RAS. */
    void push(const PCStateBase &return_addr);

    /** Pops the top address from the RAS. */
    void pop();

    /** Changes index to the top of the RAS, and replaces the top address with
     *  a new target.
     *  @param top_entry_idx The index of the RAS that will now be the top.
     *  @param restored The new target address of the new top of the RAS.
     */
    void restore(unsigned top_entry_idx, const PCStateBase *restored);

    bool empty() { return usedEntries == 0; }

    bool full() { return usedEntries == numEntries; }
  private:
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

    /** The RAS itself. */
    std::vector<std::unique_ptr<PCStateBase>> addrStack;

    /** The number of entries in the RAS. */
    unsigned numEntries;

    /** The number of used entries in the RAS. */
    unsigned usedEntries;

    /** The top of stack index. */
    unsigned tos;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_RAS_HH__
