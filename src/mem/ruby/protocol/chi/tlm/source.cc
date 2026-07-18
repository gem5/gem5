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

#include "mem/ruby/protocol/chi/tlm/source.hh"

#include <utility>

#include "base/logging.hh"

namespace gem5
{

namespace tlm::chi
{

TlmSource::TlmSource(const Params &p)
    : SimObject(p), generator(nullptr), initDone(false)
{}

void
TlmSource::init()
{
    SimObject::init();

    initDone = true;
    fatal_if(generator == nullptr && !bufferedTransactions.empty(),
             "TlmSource %s is not connected to a TlmGenerator",
             name().c_str());
}

void
TlmSource::setGenerator(TlmGenerator *generator_)
{
    fatal_if(generator != nullptr && generator != generator_,
             "TlmSource %s is already connected to a TlmGenerator",
             name().c_str());

    generator = generator_;
    replayBufferedTransactions();
}

void
TlmSource::replayBufferedTransactions()
{
    for (auto &request : bufferedTransactions) {
        if (request.timed) {
            generator->scheduleTransaction(request.when, request.transaction);
        } else {
            generator->enqueueInput(request.transaction);
        }
    }

    bufferedTransactions.clear();
}

void
TlmSource::bufferTransaction(Transaction *transaction)
{
    fatal_if(initDone, "TlmSource %s is not connected to a TlmGenerator",
             name().c_str());

    bufferedTransactions.push_back({0, transaction, false});
}

void
TlmSource::bufferTransactionAt(Tick when, Transaction *transaction)
{
    fatal_if(initDone, "TlmSource %s is not connected to a TlmGenerator",
             name().c_str());

    bufferedTransactions.push_back({when, transaction, true});
}

void
TlmSource::injectTransaction(Transaction *transaction)
{
    if (generator == nullptr) {
        bufferTransaction(transaction);
        return;
    }

    generator->enqueueInput(transaction);
}

void
TlmSource::injectTransactionAt(Tick when, Transaction *transaction)
{
    if (generator == nullptr) {
        bufferTransactionAt(when, transaction);
        return;
    }

    generator->scheduleTransaction(when, transaction);
}

TlmSource::Transaction *
TlmSource::injectOwnedTransaction(TransactionPtr transaction)
{
    auto *transaction_ptr = transaction.get();
    transactions.push_back(std::move(transaction));
    injectTransaction(transaction_ptr);
    return transaction_ptr;
}

TlmSource::Transaction *
TlmSource::injectOwnedTransactionAt(Tick when, TransactionPtr transaction)
{
    auto *transaction_ptr = transaction.get();
    transactions.push_back(std::move(transaction));
    injectTransactionAt(when, transaction_ptr);
    return transaction_ptr;
}

PySource::PySource(const Params &p) : TlmSource(p)
{}

} // namespace tlm::chi

} // namespace gem5
