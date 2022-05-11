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

#include <tlm_utils/simple_target_socket.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "base/trace.hh"
#include "params/TLM_Target.hh"
#include "sc_tlm_target.hh"

#include "systemc/ext/systemc"
#include "systemc/ext/tlm"

using namespace std;
using namespace sc_core;
using namespace gem5;

void Target::b_transport(tlm::tlm_generic_payload& trans,
                            sc_time& delay)
{
    executeTransaction(trans);
}

void Target::executeTransaction(tlm::tlm_generic_payload& trans)
{
    tlm::tlm_command cmd = trans.get_command();
    sc_dt::uint64    adr = trans.get_address();
    unsigned char*   ptr = trans.get_data_ptr();
    unsigned int     len = trans.get_data_length();
    unsigned char*   byt = trans.get_byte_enable_ptr();

    if (trans.get_address() >= 16*1024*1024) {
        cout << "\033[1;31m("
             << "Address Error"
             << "\033[0m" << endl;
        trans.set_response_status( tlm::TLM_ADDRESS_ERROR_RESPONSE );
        return;
    }
    if (byt != 0) {
        cout << "\033[1;31m("
             << "Byte Enable Error"
             << "\033[0m" << endl;
        trans.set_response_status( tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE );
        return;
    }
    if (len < 1 || len > 64) {
        cout << "\033[1;31m("
             << "Burst Error"
             << "\033[0m" << endl;
        trans.set_response_status( tlm::TLM_BURST_ERROR_RESPONSE );
        return;
    }

    if (cmd == tlm::TLM_READ_COMMAND)
    {
        memcpy(mem+trans.get_address(), // destination
                trans.get_data_ptr(),      // source
                trans.get_data_length());  // size
    }
    else if (cmd == tlm::TLM_WRITE_COMMAND)
    {
        memcpy(trans.get_data_ptr(),      // destination
                mem + trans.get_address(), // source
                trans.get_data_length());  // size
    }

    cout << "\033[1;32m("
            << name()
            << ")@"  << setfill(' ') << setw(12) << sc_time_stamp()
            << ": " << setw(12) << (cmd ? "Exec. Write " : "Exec. Read ")
            << "Addr = " << setfill('0') << setw(8) << hex << adr
            << " Data = " << "0x" << setfill('0') << setw(8) << hex
            << *reinterpret_cast<int*>(ptr)
            << "\033[0m" << endl;

    trans.set_response_status( tlm::TLM_OK_RESPONSE );
}

// This "create" method bridges the python configuration and the systemc
// objects. It instantiates the Printer object and sets it up using the
// parameter values from the config, just like it would for a SimObject. The
// systemc object could accept those parameters however it likes, for instance
// through its constructor or by assigning them to a member variable.
Target *
gem5::TLM_TargetParams::create() const
{
    Target *target = new Target(name.c_str());
    return target;
}

gem5::Port
&Target::gem5_getPort(const std::string &if_name, int idx)
{
    return wrapper;
}
