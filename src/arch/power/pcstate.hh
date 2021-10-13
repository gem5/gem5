/*
 * Copyright (c) 2009 The University of Edinburgh
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

#ifndef __ARCH_POWER_PCSTATE_HH__
#define __ARCH_POWER_PCSTATE_HH__

#include "arch/generic/pcstate.hh"
#include "arch/power/types.hh"
#include "enums/ByteOrder.hh"

namespace gem5
{

namespace PowerISA
{

class PCState : public GenericISA::SimplePCState<4>
{
  private:
    ByteOrder guestByteOrder = ByteOrder::big;

  public:
    using GenericISA::SimplePCState<4>::SimplePCState;

    PCState(const PCState &other) :
        GenericISA::SimplePCState<4>(other),
        guestByteOrder(other.guestByteOrder)
    {}
    PCState &operator=(const PCState &other) = default;

    PCStateBase *clone() const override { return new PCState(*this); }

    void
    update(const PCStateBase &other) override
    {
        GenericISA::SimplePCState<4>::update(other);
        auto &pcstate = other.as<PCState>();
        guestByteOrder = pcstate.guestByteOrder;
    }

    ByteOrder
    byteOrder() const
    {
        return guestByteOrder;
    }

    void
    byteOrder(ByteOrder order)
    {
        guestByteOrder = order;
    }
};

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_PCSTATE_HH__
