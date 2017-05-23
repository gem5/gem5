/*
 * Copyright (c) 2015, University of Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Jung
 */

#ifndef __SIM_SC_TARGET_HH__
#define __SIM_SC_TARGET_HH__

#include <tlm_utils/peq_with_cb_and_phase.h>
#include <tlm_utils/simple_target_socket.h>

#include <iostream>
#include <systemc>
#include <tlm>

using namespace sc_core;
using namespace std;

struct Target: sc_module
{
    /** TLM interface socket: */
    tlm_utils::simple_target_socket<Target> socket;

    /** TLM related member variables: */
    tlm::tlm_generic_payload*  transaction_in_progress;
    sc_event                   target_done_event;
    bool                       response_in_progress;
    bool                       debug;
    tlm::tlm_generic_payload*  next_response_pending;
    tlm::tlm_generic_payload*  end_req_pending;
    tlm_utils::peq_with_cb_and_phase<Target> m_peq;

    /** Storage, may be implemented with a map for large devices */
    unsigned char *mem;

    Target(sc_core::sc_module_name name,
        bool debug,
        unsigned long long int size,
        unsigned int offset);
    SC_HAS_PROCESS(Target);

    /** TLM interface functions */
    virtual void b_transport(tlm::tlm_generic_payload& trans,
                             sc_time& delay);
    virtual unsigned int transport_dbg(tlm::tlm_generic_payload& trans);
    virtual tlm::tlm_sync_enum nb_transport_fw(
                tlm::tlm_generic_payload& trans,
                tlm::tlm_phase& phase,
                sc_time& delay);

    /** Callback of Payload Event Queue: */
    void peq_cb(tlm::tlm_generic_payload& trans,
                const tlm::tlm_phase& phase);

    /** Helping function common to b_transport and nb_transport */
    void execute_transaction(tlm::tlm_generic_payload& trans);

    /** Helping functions and processes: */
    void send_end_req(tlm::tlm_generic_payload& trans);
    void send_response(tlm::tlm_generic_payload& trans);

    /** Method process that runs on target_done_event */
    void execute_transaction_process();

    /** Helping function that checks if a requested address is with range */
    void check_address(unsigned long long int addr);

    /** Helping Variables **/
    unsigned long long int size;
    unsigned offset;
};

#endif //__SIM_SC_TARGET_HH__

