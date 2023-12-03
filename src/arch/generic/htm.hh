/*
 * Copyright (c) 2020 ARM Limited
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

/*
 * Gem5 Hardware Transactional Memory (HTM)
 *
 * Here we provide a brief note describing HTM support in Gem5 at
 * a high level.
 *
 * HTM is an architectural feature that enables speculative
 * concurrency in a shared-memory system; groups of instructions known as
 * transactions are executed as an atomic unit. The system allows that
 * transactions be executed concurrently but intervenes if a transaction's
 * atomicity/isolation is jeapordised and takes corrective action. In this
 * implementation, corrective active explicitely means rolling back a thread's
 * architectural state and reverting any memory updates to a point just
 * before the transaction began.
 *
 * This HTM implementation relies on--
 * (1) A checkpointing mechanism for architectural register state.
 * (2) Buffering speculative memory updates.
 *
 * The checkpointing mechanism is architecture dependent. Each ISA leveraging
 * HTM support must define a class HTMCheckpoint in src/arch/theISA/htm.hh.
 * Instances of this class live in O3's ThreadState and Atomic's SimpleThread.
 * It is up to the ISA to populate this instance when executing an instruction
 * that begins a new transaction.
 *
 * The buffering of speculative memory updates is currently implemented in
 * the MESI_Three_Level Ruby protocol. The core notifies the L0 cache
 * controller that a new transaction has started and the controller in turn
 * places itself in transactional state (htmTransactionalState := true).
 * When operating in transactional state, the usual MESI protocol changes
 * slightly. Lines loaded or stored are marked as part of a transaction's
 * read and write set respectively. If there is an invalidation request to
 * cache line in the read/write set, the transaction is marked as failed.
 * Similarly, if there is a read request by another core to a speculatively
 * written cache line, i.e. in the write set, the transaction is marked as
 * failed. If failed, all subsequent loads and stores from the core are
 * made benign, i.e. made into NOPS at the cache controller, and responses are
 * marked to indicate that the transactional state has failed. When the core
 * receives these marked responses, it generates a HtmFailureFault with the
 * reason for the transaction failure. Servicing this fault does two things--
 * (a) Restores the architectural checkpoint
 * (b) Sends an HTM abort signal to the cache controller
 *
 * The restoration includes all registers in the checkpoint as well as the
 * program counter of the instruction before the transaction started.
 *
 * The abort signal is sent to the L0 cache controller and resets the
 * failed transactional state. It resets the transactional read and write sets
 * and invalidates any speculatively written cache lines.  It also exits
 * the transactional state so that the MESI protocol operates as usual.
 *
 * Alternatively, if the instructions within a transaction complete without
 * triggering a HtmFailureFault, the transaction can be committed. The core
 * is responsible for notifying the cache controller that the transaction is
 * complete and the cache controller makes all speculative writes visible
 * to the rest of the system and exits the transactional state.
 *
 * Notifting the cache controller is done through HtmCmd Requests which are
 * a subtype of Load Requests.
 *
 * Most HTMs allow for a limited number of nested transactions, e.g. a nesting
 * depth of two would be inside a transaction started within another
 * transaction. The ExecContext class is extended with
 * getHtmTransactionalDepth() to return the current depth. For the
 * TimingSimpleCPU it is straightforward to track this, whereas for
 * O3DerivCPU it must be tracked in the frontend and commit stages as well as
 * be corrected on branch mispredictions. This is done in iew_impl.hh.
 */

#ifndef __ARCH_GENERIC_HTM_HH__
#define __ARCH_GENERIC_HTM_HH__

#include <cstdint>
#include <memory>

#include "mem/htm.hh"

namespace gem5
{

/**
 * @file
 *
 * Generic definitions for hardware transactional memory.
 */

class ThreadContext;
class BaseHTMCheckpoint;

typedef std::unique_ptr<BaseHTMCheckpoint> BaseHTMCheckpointPtr;

/**
 * Transactional Memory checkpoint.
 */
class BaseHTMCheckpoint
{
  private:
    static uint64_t globalHtmUid;
    uint64_t localHtmUid;

  public:
    BaseHTMCheckpoint() : localHtmUid(0), _valid(false) { reset(); }

    virtual ~BaseHTMCheckpoint() {}

    /**
     * Every ISA implementing HTM support should override the
     * save method. This is called once a transaction starts
     * and the architectural state needs to be saved.
     * This will checkpoint the arch state.
     *
     * @param tc: thread context state to be saved
     */
    virtual void
    save(ThreadContext *tc)
    {
        _valid = true;
    }

    /**
     * Every ISA implementing HTM support should override the
     * restore method. This is called once a transaction gets
     * aborted and the architectural state needs to be reverted.
     * This will restore the checkpointed arch state.
     *
     * @param tc: thread context to be restored
     * @param cause: the reason why the transaction has been aborted
     */
    virtual void
    restore(ThreadContext *tc, HtmFailureFaultCause cause)
    {
        reset();
    }

    bool
    valid() const
    {
        return _valid;
    }

    /**
     * Generates a new HTM identifier (used when starting a new transaction)
     */
    uint64_t
    newHtmUid()
    {
        localHtmUid = ++globalHtmUid;
        return localHtmUid;
    }

    /**
     * Returns the current HTM identifier
     */
    uint64_t
    getHtmUid() const
    {
        return localHtmUid;
    }

    /**
     * Sets the current HTM identifier
     */
    void
    setHtmUid(uint64_t new_htm_uid)
    {
        localHtmUid = new_htm_uid;
    }

  protected:
    /**
     * Resets the checkpoint once a transaction has completed.
     * The method is bringing up the checkpoint to a known
     * reset state so that it can be reused.
     * ISA specific checkpoints inheriting from this class should
     * override this method so that they can reset their own
     * ISA specific state.
     */
    virtual void
    reset()
    {
        _valid = false;
    }

    bool _valid;
};

} // namespace gem5

#endif // __ARCH_GENERIC_HTM_HH__
