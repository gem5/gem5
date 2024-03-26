/*
 * Copyright 2022 Fraunhofer IESE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __SYSTEC_TLM_GEM5_EXAMPLE__
#define __SYSTEC_TLM_GEM5_EXAMPLE__

#include <tlm_utils/simple_target_socket.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "base/trace.hh"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/tlm_port_wrapper.hh"

#include "systemc/ext/systemc"
#include "systemc/ext/tlm"

using namespace std;
using namespace sc_core;
using namespace gem5;

SC_MODULE(Target)
{
  public:
    tlm_utils::simple_target_socket<Target> tSocket;
    sc_gem5::TlmTargetWrapper<32> wrapper;

  private:
    // unsigned char mem[512];
    unsigned char *mem;

  public:
    SC_HAS_PROCESS(Target);
    Target(sc_module_name name)
        : sc_module(name),
          tSocket("tSocket"),
          wrapper(tSocket, std::string(name) + ".tlm", InvalidPortID)
    {
        tSocket.register_b_transport(this, &Target::b_transport);
        mem = (unsigned char *)malloc(16 * 1024 * 1024);
        std::cout << "TLM Target Online" << std::endl;
    }

    gem5::Port &gem5_getPort(const std::string &if_name, int idx = -1)
        override;

    virtual void b_transport(tlm::tlm_generic_payload & trans,
                             sc_time & delay);

    void executeTransaction(tlm::tlm_generic_payload & trans);
};

#endif // __SYSTEC_TLM_GEM5_EXAMPLE__
