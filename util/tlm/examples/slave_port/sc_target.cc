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

#include "sc_target.hh"

using namespace sc_core;
using namespace std;

Target::Target(sc_core::sc_module_name name,
    bool debug,
    unsigned long long int size,
    unsigned int offset) :
    socket("socket"),
    transaction_in_progress(0),
    response_in_progress(false),
    next_response_pending(0),
    end_req_pending(0),
    m_peq(this, &Target::peq_cb),
    debug(debug),
    size(size),
    offset(offset)
{
    /* Register tlm transport functions */
    socket.register_b_transport(this, &Target::b_transport);
    socket.register_transport_dbg(this, &Target::transport_dbg);
    socket.register_nb_transport_fw(this, &Target::nb_transport_fw);


    /* allocate storage memory */
    mem = new unsigned char[size];

    SC_METHOD(execute_transaction_process);
    sensitive << target_done_event;
    dont_initialize();
}

void
Target::check_address(unsigned long long int addr)
{
    if (addr < offset || addr >= offset + size)
        SC_REPORT_FATAL("Target", "Address out of range. Did you set an "
                                  "appropriate size and offset?");
}

void
Target::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay)
{
    /* Execute the read or write commands */
    execute_transaction(trans);
}

unsigned int
Target::transport_dbg(tlm::tlm_generic_payload& trans)
{
    check_address(trans.get_address());

    tlm::tlm_command cmd = trans.get_command();
    sc_dt::uint64    adr = trans.get_address() - offset;
    unsigned char*   ptr = trans.get_data_ptr();
    unsigned int     len = trans.get_data_length();

    unsigned char *mem_array_ptr = mem + adr;

    /* Load / Store the access: */
    if ( cmd == tlm::TLM_READ_COMMAND ) {
        if (debug) {
            SC_REPORT_INFO("target", "tlm::TLM_READ_COMMAND");
        }
        std::memcpy(ptr, mem_array_ptr, len);
    } else if ( cmd == tlm::TLM_WRITE_COMMAND ) {
        if (debug) {
            SC_REPORT_INFO("target", "tlm::TLM_WRITE_COMMAND");
        }
        std::memcpy(mem_array_ptr, ptr, len);
    }

    return len;
}


/* TLM-2 non-blocking transport method */
tlm::tlm_sync_enum Target::nb_transport_fw(tlm::tlm_generic_payload& trans,
                                           tlm::tlm_phase& phase,
                                           sc_time& delay)
{
    /* Queue the transaction until the annotated time has elapsed */
    m_peq.notify(trans, phase, delay);
    return tlm::TLM_ACCEPTED;
}

void
Target::peq_cb(tlm::tlm_generic_payload& trans,
               const tlm::tlm_phase& phase)
{
    sc_time delay;

    if (phase == tlm::BEGIN_REQ) {
        if (debug) SC_REPORT_INFO("target", "tlm::BEGIN_REQ");

        /* Increment the transaction reference count */
        trans.acquire();

        if ( !transaction_in_progress ) {
            send_end_req(trans);
        } else {
            /* Put back-pressure on initiator by deferring END_REQ until
             * pipeline is clear */
            end_req_pending = &trans;
        }
    } else if (phase == tlm::END_RESP) {
        /* On receiving END_RESP, the target can release the transaction and
         * allow other pending transactions to proceed */
        if (!response_in_progress) {
            SC_REPORT_FATAL("TLM-2", "Illegal transaction phase END_RESP"
                            "received by target");
        }

        transaction_in_progress = 0;

        /* Target itself is now clear to issue the next BEGIN_RESP */
        response_in_progress = false;
        if (next_response_pending) {
            send_response( *next_response_pending );
            next_response_pending = 0;
        }

        /* ... and to unblock the initiator by issuing END_REQ */
        if (end_req_pending) {
            send_end_req( *end_req_pending );
            end_req_pending = 0;
        }

    } else /* tlm::END_REQ or tlm::BEGIN_RESP */ {
            SC_REPORT_FATAL("TLM-2", "Illegal transaction phase received by"
                            "target");
    }
}

void
Target::send_end_req(tlm::tlm_generic_payload& trans)
{
    tlm::tlm_phase bw_phase;
    sc_time delay;

    /* Queue the acceptance and the response with the appropriate latency */
    bw_phase = tlm::END_REQ;
    delay = sc_time(10.0, SC_NS); // Accept delay

    tlm::tlm_sync_enum status;
    status = socket->nb_transport_bw(trans, bw_phase, delay);

    /* Ignore return value;
     * initiator cannot terminate transaction at this point
     * Queue internal event to mark beginning of response: */
    delay = delay + sc_time(40.0, SC_NS); // Latency
    target_done_event.notify(delay);

    assert(transaction_in_progress == 0);
    transaction_in_progress = &trans;
}

void
Target::execute_transaction_process()
{
    /* Execute the read or write commands */
    execute_transaction( *transaction_in_progress );

    /* Target must honor BEGIN_RESP/END_RESP exclusion rule; i.e. must not
     * send BEGIN_RESP until receiving previous END_RESP or BEGIN_REQ */
    if (response_in_progress) {
        /* Target allows only two transactions in-flight */
        if (next_response_pending) {
            SC_REPORT_FATAL("TLM-2", "Attempt to have two pending responses"
                            "in target");
        }
        next_response_pending = transaction_in_progress;
    } else {
        send_response( *transaction_in_progress );
    }
}

void
Target::execute_transaction(tlm::tlm_generic_payload& trans)
{
    check_address(trans.get_address());

    tlm::tlm_command cmd = trans.get_command();
    sc_dt::uint64    adr = trans.get_address() - offset;
    unsigned char*   ptr = trans.get_data_ptr();
    unsigned int     len = trans.get_data_length();
    unsigned char*   byt = trans.get_byte_enable_ptr();
    unsigned int     wid = trans.get_streaming_width();

    if ( byt != 0 ) {
        cout << "Byte Error" << endl;
        trans.set_response_status( tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE );
        return;
    }

    //if ( len > 4 || wid < len ) {
    //    cout << "Burst Error len=" << len << " wid=" << wid << endl;
    //    trans.set_response_status( tlm::TLM_BURST_ERROR_RESPONSE );
    //    return;
    //}

    unsigned char *mem_array_ptr = mem + adr;

    /* Load / Store the access: */
    if ( cmd == tlm::TLM_READ_COMMAND ) {
        if (debug) {
            SC_REPORT_INFO("target", "tlm::TLM_READ_COMMAND");
        }
        std::memcpy(ptr, mem_array_ptr, len);
    } else if ( cmd == tlm::TLM_WRITE_COMMAND ) {
        if (debug) {
            SC_REPORT_INFO("target", "tlm::TLM_WRITE_COMMAND");
        }
        std::memcpy(mem_array_ptr, ptr, len);
    }

    trans.set_response_status( tlm::TLM_OK_RESPONSE );
}

void
Target::send_response(tlm::tlm_generic_payload& trans)
{
    tlm::tlm_sync_enum status;
    tlm::tlm_phase bw_phase;
    sc_time delay;

    response_in_progress = true;
    bw_phase = tlm::BEGIN_RESP;
    delay = sc_time(10.0, SC_NS);
    status = socket->nb_transport_bw( trans, bw_phase, delay );

    if (status == tlm::TLM_UPDATED) {
        /* The timing annotation must be honored */
        m_peq.notify(trans, bw_phase, delay);
    } else if (status == tlm::TLM_COMPLETED) {
        /* The initiator has terminated the transaction */
        transaction_in_progress = 0;
        response_in_progress = false;
    }
    trans.release();
}
