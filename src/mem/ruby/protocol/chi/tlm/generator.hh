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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_GENERATOR_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_GENERATOR_HH__

#include <queue>
#include <unordered_map>

#include <ARM/TLM/arm_chi.h>

#include "mem/ruby/protocol/chi/tlm/utils.hh"
#include "params/TlmGenerator.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5 {

namespace tlm::chi {

class CacheController;

/**
 * TlmGenerator: this class is basically a CHI-tlm traffic generator.
 * Its interface is mainly allowing to inject transactions at a specific
 * point in simulated time and to set expectations on what has
 * to be returned to the generator.
 * The intended use is as a testbench for the CHI implementation
 * in ruby. As such it requires to be hooked with a TlmController
 * as a bridge for converting the CHI-tlm transactions into ruby messages.
 *
 *                                         +---------------+
 * +--------------+   +---------------+    |               |
 * |              |   |               |    |               |
 * | TlmGenerator |-->| TlmController |--->|     Ruby      |
 * |              |<--|               |<---|               |
 * +--------------+   +---------------+    |               |
 *                                         +---------------+
 *
 * The APIs are entirely exposed to the python world for flexible
 * definition of the unit tests.
 *
 * To inject a CHI-tlm transaction at a specific tick in the
 * simulation, the following TlmGenerator method should be
 * used:
 *
 * def injectAt(self, when, payload, phase):
 *
 * This will return a Transaction object and from that point that will be the
 * handle for managing the transaction: either adding transaction expectations
 * upon response (e.g, what will be the cacheline state), or by adding action
 * callbacks (execute some logic)
 */
class TlmGenerator : public SimObject
{
  public:
    PARAMS(TlmGenerator);
    TlmGenerator(const Params &p);

    /**
     * Transaction object
     * It stores ARM::CHI::Payload and ARM::CHI::Phase objects,
     * and a list of action callables which will be executed
     * in the order they have been registered once a transaction
     * response arrives. These could be:
     *  1) Expectation callbacks (Transaction::Expectation)
     *     which will check a specific condition and will warn if
     *     the condition is not met
     *  2) Assertion callbacks (Transaction::Assertion)
     *     Identical to expectations, they will fail simulation instead
     *     of warning, which means later conditions won't be checked.
     *  3) Action callbacks (Transaction::Action)
     *     Does something without condition checking
     *
     * Once the response arrives, the Transaction::runCallbacks will
     * enter the dispatching loop. Actions/callbacks will be dispatched
     * until the list is empty or until a waiting action is encountered.
     * This will break the dispatching loop.
     */
    class Transaction
    {
      public:
        /**
         * Action:
         * Does something without condition checking. It is
         * possible to program from python whether the callback
         * is a waiting action
         */
        class Action
        {
          public:
            using Callback = std::function<
                bool(Transaction *tran)
            >;

            Action(Callback _cb, bool waiting)
              : cb(_cb), _wait(waiting)
            {}
            virtual ~Action() {};

            /* A basic action always returns true */
            virtual bool
            run(Transaction *tran)
            {
                cb(tran);
                return true;
            }

            /**
             * Returns true if the action dispatcher should break
             * the dispatching loop once the action has been executed
             */
            bool wait() const { return _wait; }

          protected:
            Callback cb;
            bool _wait;
        };

        /**
         * Expectation:
         * Will check for a specific condition and will warn if
         * the condition is not met.
         * The expectation is never a waiting action
         */
        class Expectation : public Action
        {
          public:
            Expectation(std::string exp_name, Callback _cb)
              : Action(_cb, false), _name(exp_name)
            {}

            bool run(Transaction *tran) override;

            std::string name() const { return _name; }

          private:
            std::string _name;
        };

        /**
         * Assertion:
         * Will check for a specific condition and will fail if
         * the condition is not met.
         * The assertion is never a waiting action
         */
        class Assertion : public Expectation
        {
          public:
            Assertion(std::string exp_name, Callback _cb)
              : Expectation(exp_name, _cb)
            {}

            bool run(Transaction *tran) override;
        };

        using ActionPtr = std::unique_ptr<Action>;
        using Actions = std::list<ActionPtr>;

        Transaction(const Transaction &rhs) = delete;
        Transaction(ARM::CHI::Payload *pa, ARM::CHI::Phase &ph);
        ~Transaction();

        /**
         * Registers the TlmGenerator in the transaction. This is
         * used as a reference to the generator is required when
         * injection the transaction
         */
        void setGenerator(TlmGenerator *gen);

        std::string str() const;

        void inject();

        /**
         * Returns true if the transaction has failed, false
         * otherwise
         */
        bool failed() const;

        /**
         * Returns true if the transaction has some
         * registered callbacks, false otherwise
         */
        bool hasCallbacks() const;

        /**
         * Appends a callback to the list of actions
         */
        void addCallback(ActionPtr &&action);

        /**
         * Enters the dispatching loop and runs the callbacks
         * in insertion order until a waiting callback is
         * encountered (or if there is a failing assertion)
         */
        void runCallbacks();

        ARM::CHI::Payload* payload() const { return _payload; }
        ARM::CHI::Phase& phase() { return _phase; }

      private:
        Actions actions;
        bool passed;

        TlmGenerator *parent;
        ARM::CHI::Payload *_payload;
        ARM::CHI::Phase _phase;
    };

    void scheduleTransaction(Tick when, Transaction *tr);

  protected:
    struct TransactionEvent : public Event
    {
      public:
        struct Compare
        {
            bool
            operator()(const TransactionEvent *lhs,
                       const TransactionEvent *rhs)
            {
                return lhs->when < rhs->when;
            }
        };

        TransactionEvent(Transaction *_transaction,
                         Tick _when)
          : Event(), transaction(_transaction), when(_when)
        {}

        void process() override;

        const char*
        description() const override
        {
            return "CHI Transaction event";
        }

        Transaction *transaction;
        Tick when;
    };

    void inject(Transaction *transaction);
    void recv(ARM::CHI::Payload *payload, ARM::CHI::Phase *phase);
    void passFailCheck();

  protected:
    /** cpuId to mimic the behaviour of a CPU */
    uint8_t cpuId;

    using SchedulingQueue = std::priority_queue<TransactionEvent*,
        std::vector<TransactionEvent*>,
        TransactionEvent::Compare>;

    /** PQ of transactions whose injection needs to be scheduled */
    SchedulingQueue scheduledTransactions;

    /** Map of pending (injected) transactions indexed by the txn_id */
    std::unordered_map<uint16_t, Transaction*> pendingTransactions;

    /** Pointer to the CHI-tlm controller */
    CacheController *controller;
};

} // namespace tlm::chi

} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_TLM_GENERATOR_HH__
