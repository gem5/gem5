/*
 * Copyright (c) 2024-2025 Arm Limited
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

TlmGenerator::Transaction::Transaction(ARM::CHI::Payload *pa,
                                       ARM::CHI::Phase &ph)
    : passed(true), parent(nullptr), _payload(pa), _phase(ph), _start(0)
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

void
TlmGenerator::Transaction::send()
{
    parent->send(this);
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

    // Once we have run out of callback we consider this
    // as terminated and we can remove it
    if (it == actions.end()) {
        parent->terminate(this);
    }
}

void
TlmGenerator::TransactionEvent::process()
{
    transaction->inject();
}

TlmGenerator::TlmGenerator(const Params &p)
    : ClockedObject(p),
      cpuId(p.cpu_id),
      transPerCycle(p.tran_per_cycle),
      maxPendingTrans(
          p.max_pending_tran.value_or(std::numeric_limits<uint16_t>::max())),
      tickEvent([this] { tick(); }, "TlmGenerator tick", false,
                Event::CPU_Tick_Pri),
      outPort(name() + ".out_port", 0, this),
      inPort(name() + ".in_port", 0, this),
      suiteFailure(false)
{
    inPort.onChange([this](const TlmData &data) {
        auto payload = data.first;
        auto phase = data.second;
        this->recv(payload, phase);
    });

    registerExitCallback([this](){ passFailCheck(); });
}

void
TlmGenerator::tick()
{
    unsigned pending_size = pendingTransactions.size();
    auto slots = std::min(transPerCycle, maxPendingTrans - pending_size);
    while (!unscheduledTransactions.empty() && slots > 0) {
        auto tran = unscheduledTransactions.front();
        scheduleTransaction(curTick(), tran);
        unscheduledTransactions.pop_front();
        slots--;
    }
    if (!unscheduledTransactions.empty()) {
        schedule(tickEvent, nextCycle());
    }
}

void
TlmGenerator::scheduleTransaction(Tick when, Transaction *transaction)
{
    transaction->setGenerator(this);
    transaction->setStart(when);

    auto event = new TransactionEvent(transaction, when);

    scheduledTransactions.push(event);

    schedule(event, when);
}

void
TlmGenerator::enqueueTransaction(Transaction *transaction)
{
    unscheduledTransactions.push_back(transaction);

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, nextCycle());
    }
}

void
TlmGenerator::inject(Transaction *transaction)
{
    ARM::CHI::Phase &phase = transaction->phase();

    pendingTransactions.insert({phase.txn_id, transaction});

    send(transaction);
}

void
TlmGenerator::send(Transaction *transaction)
{
    auto payload = transaction->payload();
    ARM::CHI::Phase &phase = transaction->phase();

    DPRINTF(TLM, "[c%d] send %s\n", cpuId, transactionToString(*payload, phase));

    auto tlm_data = TlmData(payload, &phase);
    outPort.send(tlm_data);
}

void
TlmGenerator::terminate(Transaction *transaction)
{
    ARM::CHI::Phase &phase = transaction->phase();
    if (auto it = pendingTransactions.find(phase.txn_id);
        it != pendingTransactions.end()) {

        pendingTransactions.erase(it);

        // If the transaction has failed, mark the suite as failure
        suiteFailure = suiteFailure || transaction->failed();
    } else {
        panic("Can't find transaction id: %u\n", phase.txn_id);
    }
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
    // We are failing either if a condition hasn't been met,
    // or if there are pending actions when simulation exits
    if (suiteFailure) {
        inform(" Suite Fail: failed transaction ");
    } else if (!pendingTransactions.empty()) {
        inform(" Suite Fail: non-empty transaction queue ");
    } else {
        inform(" Suite Success ");
    }
}

Port &
TlmGenerator::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "out_port") {
        return outPort;
    } else if (if_name == "in_port") {
        return inPort;
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

} // namespace tlm::chi

} // namespace gem5
