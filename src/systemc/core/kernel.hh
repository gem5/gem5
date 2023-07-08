/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_KERNEL_HH__
#define __SYSTEMC_KERNEL_HH__

#include "params/SystemC_Kernel.hh"
#include "sim/sim_object.hh"
#include "systemc/ext/core/sc_main.hh"

namespace sc_gem5
{

/*
 * This class represents the systemc kernel. There should be exactly one in
 * the simulation. It receives gem5 SimObject lifecycle callbacks (init,
 * regStats, etc.) and manages the lifecycle of the systemc simulation
 * accordingly. It also acts as a collecting point for systemc related
 * control functionality.
 */
class Kernel : public gem5::SimObject
{
  public:
    typedef gem5::SystemC_KernelParams Params;
    Kernel(const Params &params, int);

    void init() override;
    void regStats() override;
    void startup() override;

    void t0Handler();

    static sc_core::sc_status status();
    static void status(sc_core::sc_status s);

    static void stop();

    static bool startOfSimulationComplete();
    static bool endOfSimulationComplete();

  private:
    static void stopWork();

    gem5::MemberEventWrapper<&Kernel::t0Handler> t0Event;
};

extern Kernel *kernel;

} // namespace sc_gem5

#endif // __SYSTEMC_KERNEL_H__
