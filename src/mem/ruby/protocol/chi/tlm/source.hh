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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_SOURCE_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_SOURCE_HH__

#include <memory>
#include <vector>

#include "mem/ruby/protocol/chi/tlm/generator.hh"
#include "params/PySource.hh"
#include "params/TlmSource.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace tlm::chi
{

class TlmSource : public SimObject
{
  public:
    PARAMS(TlmSource);

    using Transaction = TlmGenerator::Transaction;
    using TransactionPtr = std::unique_ptr<Transaction>;

    explicit TlmSource(const Params &p);

    void setGenerator(TlmGenerator *generator);

    void injectTransaction(Transaction *transaction);
    void injectTransactionAt(Tick when, Transaction *transaction);

  protected:
    Transaction *injectOwnedTransaction(TransactionPtr transaction);
    Transaction *injectOwnedTransactionAt(Tick when,
                                          TransactionPtr transaction);

  private:
    TlmGenerator *generator;
    std::vector<TransactionPtr> transactions;
};

/**
 * PySource: CHI-TLM transaction source exposed to the python world for
 * flexible definition of the unit tests.
 *
 * To inject a CHI-TLM transaction at a specific tick in the simulation, the
 * following PySource method should be used:
 *
 * def inject(self, payload, phase, when=None):
 *
 * This will return a Transaction object and from that point that will be the
 * handle for managing the transaction: either adding transaction expectations
 * upon response (e.g, what will be the cacheline state), or by adding action
 * callbacks (execute some logic)
 *
 * By default the last kw argument (when) is set to None.
 * This means the new transaction will be added to a pending queue and will
 * only be scheduled in a FCFS policy. The generator will try to schedule
 * a configurable number of new transactions every clock cycle.
 *
 * If the when argument is instead provided, the transaction will be scheduled
 * to happen at a specific point in time, regardless of the existing backlog.
 */
class PySource : public TlmSource
{
  public:
    PARAMS(PySource);

    explicit PySource(const Params &p);
};

} // namespace tlm::chi

} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_TLM_SOURCE_HH__
