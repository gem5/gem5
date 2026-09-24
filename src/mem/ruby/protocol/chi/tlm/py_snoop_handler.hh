/*
 * Copyright (c) 2026 Arm Limited
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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_PY_SNOOP_HANDLER_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_PY_SNOOP_HANDLER_HH__

#include <list>
#include <unordered_map>

#include "mem/ruby/protocol/chi/tlm/snp_handler.hh"
#include "params/PySnoopHandler.hh"

namespace gem5
{

namespace tlm::chi
{

/**
 * The PySnoopHandler is following the same principle behind the TlmGenerator;
 * similar to Transactions which are defined and injected from python, we
 * define SnoopResponses whose handling logic resides purely in python.  There
 * is no internal state inside the PySnoopHandler.  We use the following API
 * to generate SnoopResponse objects and return a handle to the response.
 *
 * snoop = snoop_handler.add_snoop(address)
 *
 * This will inform the SnoopHandler it is expected to receive a
 * SNP request at the provided address. A user should then attach
 * Transaction::Actions to the SnoopResponse which will entirely
 * dictate how the handler will react to such snoop.
 */
class PySnoopHandler : public SnoopHandler
{
  public:
    PARAMS(PySnoopHandler);
    PySnoopHandler(const Params &p);

    void addSnoop(Addr address, SnoopResponse *transaction);

  protected:
    bool handleSnoop(ARM::CHI::Payload *payload,
                     ARM::CHI::Phase *phase) override;

    /**
     * True if the SnoopResponse transaction has to be discarded
     * when all its actions/callbacks have been called or whether
     * it will remain allocated to the matching address.
     **/
    const bool discardAfterSnoop;

    std::unordered_map<Addr, std::list<SnoopResponse *>> snoopTransactions;
};

} // namespace tlm::chi

} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_TLM_PY_SNOOP_HANDLER_HH__
