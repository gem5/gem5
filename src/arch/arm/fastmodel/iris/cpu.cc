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
 *
 * Authors: Gabe Black
 */

#include "arch/arm/fastmodel/iris/cpu.hh"

#include "arch/arm/fastmodel/iris/thread_context.hh"

namespace Iris
{

BaseCPU::BaseCPU(BaseCPUParams *params, sc_core::sc_module *_evs) :
    ::BaseCPU::BaseCPU(params), evs(_evs),
    clockEvent(nullptr), periodAttribute(nullptr)
{
    sc_core::sc_attr_base *base;

    base = evs->get_attribute(Gem5CpuAttributeName);
    auto *gem5_cpu_attr =
        dynamic_cast<sc_core::sc_attribute<::BaseCPU *> *>(base);
    panic_if(base && !gem5_cpu_attr,
             "The EVS gem5 CPU attribute was not of type "
             "sc_attribute<::BaesCPU *>.");
    if (gem5_cpu_attr)
        gem5_cpu_attr->value = this;

    const auto &event_vec = evs->get_child_events();
    auto event_it = std::find_if(event_vec.begin(), event_vec.end(),
            [](const sc_core::sc_event *e) -> bool {
                return e->basename() == ClockEventName; });
    if (event_it != event_vec.end())
        clockEvent = *event_it;

    base = evs->get_attribute(PeriodAttributeName);
    periodAttribute = dynamic_cast<sc_core::sc_attribute<Tick> *>(base);
    panic_if(base && !periodAttribute,
            "The EVS clock period attribute is not of type "
            "sc_attribute<Tick>.");
}

BaseCPU::~BaseCPU()
{
    for (auto &tc: threadContexts)
        delete tc;
    threadContexts.clear();
}

Counter
BaseCPU::totalInsts() const
{
    Counter count = 0;
    for (auto *tc: threadContexts) {
        auto *itc = dynamic_cast<Iris::ThreadContext *>(tc);
        count += itc->totalInsts();
    }
    return count;
}

} // namespace Iris
