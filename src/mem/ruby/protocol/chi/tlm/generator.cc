/*
 * Copyright (c) 2024 Arm Limited
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

#include "mem/ruby/protocol/chi/tlm/generator.hh"

#include "mem/ruby/protocol/chi/tlm/controller.hh"
#include "debug/TLM.hh"

namespace gem5 {

namespace tlm::chi {

bool
TlmGenerator::Transaction::Expectation::run(Transaction *tran)
{
    auto res_print = csprintf("Checking %s...", name());
    if (cb(tran)) {
        inform("%s\n", res_print + " Success ");
        return true;
    } else {
        inform("%s\n", res_print + " Fail ");
        return false;
    }
}

bool
TlmGenerator::Transaction::Assertion::run(Transaction *tran)
{
    if (Expectation::run(tran)) {
        return true;
    } else {
        panic("Failing assertion\n");
    }
}

TlmGenerator::Transaction::Transaction(ARM::CHI::Payload *pa, ARM::CHI::Phase &ph)
  : passed(true), parent(nullptr), _payload(pa), _phase(ph)
{
    _payload->ref();
}

TlmGenerator::Transaction::~Transaction()
{
    _payload->unref();
}

void
TlmGenerator::Transaction::setGenerator(TlmGenerator *gen)
{
    parent = gen;
}

std::string
TlmGenerator::Transaction::str() const
{
    return transactionToString(*_payload, _phase);
}

void
TlmGenerator::Transaction::inject()
{
    parent->inject(this);
}

bool
TlmGenerator::Transaction::hasCallbacks() const
{
    return !actions.empty();
}

bool
TlmGenerator::Transaction::failed() const
{
    return !passed;
}

void
TlmGenerator::Transaction::addCallback(ActionPtr &&action)
{
    actions.push_back(std::move(action));
}

void
TlmGenerator::Transaction::runCallbacks()
{
    // print transaction
    auto it = actions.begin();
    while (it != actions.end()) {
        const bool is_passing = (*it)->run(this);
        if (!is_passing) {
            passed = false;
        }
        bool wait = (*it)->wait();

        it = actions.erase(it);

        if (wait) {
            break;
        }
    }
}

void
TlmGenerator::TransactionEvent::process()
{
    transaction->inject();
}

TlmGenerator::TlmGenerator(const Params &p)
  : SimObject(p), cpuId(p.cpu_id), controller(p.chi_controller)
{
    controller->bw = [this] (ARM::CHI::Payload *payload, ARM::CHI::Phase *phase)
    {
        this->recv(payload, phase);
    };

    registerExitCallback([this](){ passFailCheck(); });
}

void
TlmGenerator::scheduleTransaction(Tick when, Transaction *transaction)
{
    transaction->setGenerator(this);

    auto event = new TransactionEvent(transaction, when);

    scheduledTransactions.push(event);

    schedule(event, when);
}

void
TlmGenerator::inject(Transaction *transaction)
{
    auto payload = transaction->payload();
    ARM::CHI::Phase &phase = transaction->phase();

    if (transaction->hasCallbacks())
        pendingTransactions.insert({phase.txn_id, transaction});

    DPRINTF(TLM, "[c%d] send %s\n", cpuId, transactionToString(*payload, phase));

    controller->sendMsg(*payload, phase);
}

void
TlmGenerator::recv(ARM::CHI::Payload *payload, ARM::CHI::Phase *phase)
{
    DPRINTF(TLM, "[c%d] rcvd %s\n", cpuId, transactionToString(*payload, *phase));

    auto txn_id = phase->txn_id;
    if (auto it = pendingTransactions.find(txn_id);
        it != pendingTransactions.end()) {
        // Copy the new phase
        it->second->phase() = *phase;

        // Check existing expectations
        it->second->runCallbacks();
    } else {
        warn("Transaction untested\n");
    }
}

void
TlmGenerator::passFailCheck()
{
    for (auto [txn_id, transaction] : pendingTransactions) {
        // We are failing either if a condition hasn't been met,
        // or if there are pending actions when simulation exits
        if (transaction->failed()) {
            inform(" Suite Fail: failed transaction ");
            return;
        }
        if (transaction->hasCallbacks()) {
            inform(" Suite Fail: non-empty action queue ");
            return;
        }
    }
    inform(" Suite Success ");
}

} // namespace tlm::chi

} // namespace gem5
