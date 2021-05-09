/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_PROTOCOL_EXPORTED_CLOCK_RATE_CONTROL_HH__
#define __ARCH_ARM_FASTMODEL_PROTOCOL_EXPORTED_CLOCK_RATE_CONTROL_HH__

#include <string>
#include <systemc>
#include <tlm>

namespace gem5
{

// This protocol is an exportable version of the clock rate protocol native to
// fast models. It's identical to the original, except it has some extra info
// which lets it be exported into systemc.

struct ClockRateControlDummyProtocolType {};

class ClockRateControlFwIf : public virtual sc_core::sc_interface
{
  public:
    virtual ~ClockRateControlFwIf() {}
    virtual void set_mul_div(uint64_t mul, uint64_t div) = 0;
};

class ClockRateControlBwIf : public virtual sc_core::sc_interface
{
  public:
    virtual ~ClockRateControlBwIf() {}
};

class ClockRateControlSlaveBase : public ClockRateControlFwIf
{
  public:
    ClockRateControlSlaveBase(const std::string &name) {}
};

class ClockRateControlInitiatorSocket :
    public tlm::tlm_base_initiator_socket<64, ClockRateControlFwIf,
                                          ClockRateControlBwIf>
{
  private:
    ClockRateControlBwIf dummyBwIf;

  public:
    typedef tlm::tlm_base_initiator_socket<64, ClockRateControlFwIf,
                                           ClockRateControlBwIf> Base;

    using Base::bind;
    using Base::operator();

    ClockRateControlInitiatorSocket() : Base()
    {
        get_base_export().bind(dummyBwIf);
    }
    ClockRateControlInitiatorSocket(const char *name) : Base(name)
    {
        get_base_export().bind(dummyBwIf);
    }

    const char *
    kind() const override
    {
        return "ClockRateControlInitiatorSocket";
    }

    std::type_index
    get_protocol_types() const override
    {
        return typeid(ClockRateControlDummyProtocolType);
    }
};

class ClockRateControlTargetSocket :
    public tlm::tlm_base_target_socket<64, ClockRateControlFwIf,
                                       ClockRateControlBwIf>
{
  public:
    typedef tlm::tlm_base_target_socket<64, ClockRateControlFwIf,
                                        ClockRateControlBwIf> Base;

    using Base::bind;
    using Base::operator();

    using Base::Base;

    const char *
    kind() const override
    {
        return "ClockRateControlInitiatorSocket";
    }

    std::type_index
    get_protocol_types() const override
    {
        return typeid(ClockRateControlDummyProtocolType);
    }
};

} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_PROTOCOL_EXPORTED_CLOCK_RATE_CONTROL_HH__
