/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved.
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
 *
 * Authors: Rune Holm
 *          Marco Elver
 */

#ifndef __MEM_MEM_CHECKER_HH__
#define __MEM_MEM_CHECKER_HH__

#include <list>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/misc.hh"
#include "base/types.hh"
#include "debug/MemChecker.hh"
#include "params/MemChecker.hh"
#include "sim/core.hh"
#include "sim/sim_object.hh"

/**
 * MemChecker. Verifies that reads observe the values from permissible writes.
 * As memory operations have a start and completion time, we consider them as
 * transactions which have a start and end time. Because of this, the lifetimes
 * of transactions of memory operations may be overlapping -- we assume that if
 * there is overlap between writes, they could be reordered by the memory
 * subsystem, and a read could any of these.  For more detail, see comments of
 * inExpectedData().
 *
 * For simplicity, the permissible values a read can observe are only dependent
 * on the particular location, and we do not consider the effect of multi-byte
 * reads or writes. This precludes us from discovering single-copy atomicity
 * violations.
*/
class MemChecker : public SimObject
{
  public:
    /**
     * The Serial type is used to be able to uniquely identify a transaction as
     * it passes through the system. It's value is independent of any other
     * system counters.
     */
    typedef uint64_t Serial;

    static const Serial  SERIAL_INITIAL = 0; //!< Initial serial

    /**
     * The initial tick the system starts with. Must not be larger than the
     * minimum value that curTick() could return at any time in the system's
     * execution.
     */
    static const Tick    TICK_INITIAL   = 0;

    /**
     * The maximum value that curTick() could ever return.
     */
    static const Tick    TICK_FUTURE  = MaxTick;

    /**
     * Initial data value. No requirements.
     */
    static const uint8_t DATA_INITIAL   = 0x00;

    /**
     * The Transaction class captures the lifetimes of read and write
     * operations, and the values they consumed or produced respectively.
     */
    class Transaction
    {
      public:

        Transaction(Serial _serial,
                    Tick _start, Tick _complete,
                    uint8_t _data = DATA_INITIAL)
            : serial(_serial),
              start(_start), complete(_complete),
              data(_data)
        {}

      public:
        Serial serial; //!< Unique identifying serial
        Tick start;    //!< Start tick
        Tick complete; //!< Completion tick

        /**
         * Depending on the memory operation, the data value either represents:
         * for writes, the value written upon start; for reads, the value read
         * upon completion.
         */
        uint8_t data;

        /**
         * Orders Transactions for use with std::map.
         */
        bool operator<(const Transaction& rhs) const
        { return serial < rhs.serial; }
    };

    /**
     * The WriteCluster class captures sets of writes where all writes are
     * overlapping with at least one other write. Capturing writes in this way
     * simplifies pruning of writes.
     */
    class WriteCluster
    {
      public:
        WriteCluster()
            : start(TICK_FUTURE), complete(TICK_FUTURE),
              completeMax(TICK_INITIAL), numIncomplete(0)
        {}

        /**
         * Starts a write transaction.
         *
         * @param serial  Unique identifier of the write.
         * @param _start  When the write was sent off to the memory subsystem.
         * @param data    The data that this write passed to the memory
         *                subsystem.
         */
        void startWrite(Serial serial, Tick _start, uint8_t data);

        /**
         * Completes a write transaction.
         *
         * @param serial    Unique identifier of a write *previously started*.
         * @param _complete When the write was sent off to the memory
         *                  subsystem.
         */
        void completeWrite(Serial serial, Tick _complete);

        /**
         * Aborts a write transaction.
         *
         * @param serial Unique identifier of a write *previously started*.
         */
        void abortWrite(Serial serial);

        /**
         * @return true if this cluster's write all completed, false otherwise.
         */
        bool isComplete() const { return complete != TICK_FUTURE; }

      public:
        Tick start;     //!< Start of earliest write in cluster
        Tick complete;  //!< Completion of last write in cluster

        /**
         * Map of Serial --> Transaction of all writes in cluster; contains
         * all, in-flight or already completed.
         */
        std::unordered_map<Serial, Transaction> writes;

      private:
        Tick completeMax;
        size_t numIncomplete;
    };

    typedef std::list<Transaction> TransactionList;
    typedef std::list<WriteCluster> WriteClusterList;

    /**
     * The ByteTracker keeps track of transactions for the *same byte* -- all
     * outstanding reads, the completed reads (and what they observed) and write
     * clusters (see WriteCluster).
     */
    class ByteTracker : public Named
    {
      public:

        ByteTracker(Addr addr = 0, const MemChecker *parent = NULL)
            : Named((parent != NULL ? parent->name() : "") +
                     csprintf(".ByteTracker@%#llx", addr))
        {
            // The initial transaction has start == complete == TICK_INITIAL,
            // indicating that there has been no real write to this location;
            // therefore, upon checking, we do not expect any particular value.
            readObservations.emplace_back(
                    Transaction(SERIAL_INITIAL, TICK_INITIAL, TICK_INITIAL,
                                DATA_INITIAL));
        }

        /**
         * Starts a read transaction.
         *
         * @param serial  Unique identifier for the read.
         * @param start   When the read was sent off to the memory subsystem.
         */
        void startRead(Serial serial, Tick start);

        /**
         * Given a start and end time (of any read transaction), this function
         * iterates through all data that such a read is expected to see. The
         * data parameter is the actual value that we observed, and the
         * function immediately returns true when a match is found, false
         * otherwise.
         *
         * The set of expected data are:
         *
         * 1. The last value observed by a read with a completion time before
         *    this start time (if any).
         *
         * 2. The data produced by write transactions with a completion after
         *    the last observed read start time. Only data produced in the
         *    closest overlapping / earlier write cluster relative to this check
         *    request is considered, as writes in separate clusters are not
         *    reordered.
         *
         * @param start     Start time of transaction to validate.
         * @param complete  End time of transaction to validate.
         * @param data      The value that we have actually seen.
         *
         * @return          True if a match is found, false otherwise.
         */
        bool inExpectedData(Tick start, Tick complete, uint8_t data);

        /**
         * Completes a read transaction that is still outstanding.
         *
         * @param serial   Unique identifier of a read *previously started*.
         * @param complete When the read got a response.
         * @param data     The data returned by the memory subsystem.
         */
        bool completeRead(Serial serial, Tick complete, uint8_t data);

        /**
         * Starts a write transaction. Wrapper to startWrite of WriteCluster
         * instance.
         *
         * @param serial  Unique identifier of the write.
         * @param start   When the write was sent off to the memory subsystem.
         * @param data    The data that this write passed to the memory
         *                subsystem.
         */
        void startWrite(Serial serial, Tick start, uint8_t data);

        /**
         * Completes a write transaction. Wrapper to startWrite of WriteCluster
         * instance.
         *
         * @param serial   Unique identifier of a write *previously started*.
         * @param complete When the write was sent off to the memory subsystem.
         */
        void completeWrite(Serial serial, Tick complete);

        /**
         * Aborts a write transaction. Wrapper to abortWrite of WriteCluster
         * instance.
         *
         * @param serial Unique identifier of a write *previously started*.
         */
        void abortWrite(Serial serial);

        /**
         * This function returns the expected data that inExpectedData iterated
         * through in the last call. If inExpectedData last returned true, the
         * set may be incomplete; if inExpectedData last returned false, the
         * vector will contain the full set.
         *
         * @return Reference to internally maintained vector maintaining last
         *         expected data that inExpectedData iterated through.
         */
        const std::vector<uint8_t>& lastExpectedData() const
        { return _lastExpectedData; }

      private:

        /**
         * Convenience function to return the most recent incomplete write
         * cluster. Instantiates new write cluster if the most recent one has
         * been completed.
         *
         * @return The most recent incomplete write cluster.
         */
        WriteCluster* getIncompleteWriteCluster();

        /**
         * Helper function to return an iterator to the entry of a container of
         * Transaction compatible classes, before a certain tick.
         *
         * @param before Tick value which should be greater than the
         *               completion tick of the returned element.
         *
         * @return Iterator into container.
         */
        template <class TList>
        typename TList::iterator lastCompletedTransaction(TList *l, Tick before)
        {
            assert(!l->empty());

            // Scanning backwards increases the chances of getting a match
            // quicker.
            auto it = l->end();

            for (--it; it != l->begin() && it->complete >= before; --it);

            return it;
        }

        /**
         * Prunes no longer needed transactions. We only keep up to the last /
         * most recent of each, readObservations and writeClusters, before the
         * first outstanding read.
         *
         * It depends on the contention / overlap between memory operations to
         * the same location of a particular workload how large each of them
         * would grow.
         */
        void pruneTransactions();

      private:

        /**
         * Maintains a map of Serial -> Transaction for all outstanding reads.
         *
         * Use an ordered map here, as this makes pruneTransactions() more
         * efficient (find first outstanding read).
         */
        std::map<Serial, Transaction> outstandingReads;

        /**
         * List of completed reads, i.e. observations of reads.
         */
        TransactionList readObservations;

        /**
         * List of write clusters for this address.
         */
        WriteClusterList writeClusters;

        /**
         * See lastExpectedData().
         */
        std::vector<uint8_t> _lastExpectedData;
    };

  public:

    MemChecker(const MemCheckerParams *p)
        : SimObject(p),
          nextSerial(SERIAL_INITIAL)
    {}

    virtual ~MemChecker() {}

    /**
     * Starts a read transaction.
     *
     * @param start  Tick this read was sent to the memory subsystem.
     * @param addr   Address for read.
     * @param size   Size of data expected.
     *
     * @return Serial representing the unique identifier for this transaction.
     */
    Serial startRead(Tick start, Addr addr, size_t size);

    /**
     * Starts a write transaction.
     *
     * @param start Tick when this write was sent to the memory subsystem.
     * @param addr  Address for write.
     * @param size  Size of data to be written.
     * @param data  Pointer to size bytes, containing data to be written.
     *
     * @return Serial representing the unique identifier for this transaction.
     */
    Serial startWrite(Tick start, Addr addr, size_t size, const uint8_t *data);

    /**
     * Completes a previously started read transaction.
     *
     * @param serial    A serial of a read that was previously started and
     *                  matches the address of the previously started read.
     * @param complete  Tick we received the response from the memory subsystem.
     * @param addr      Address for read.
     * @param size      Size of data received.
     * @param data      Pointer to size bytes, containing data received.
     *
     * @return True if the data we received is in the expected set, false
     *         otherwise.
     */
    bool completeRead(Serial serial, Tick complete,
                      Addr addr, size_t size, uint8_t *data);

    /**
     * Completes a previously started write transaction.
     *
     * @param serial    A serial of a write that was previously started and
     *                  matches the address of the previously started write.
     * @param complete  Tick we received acknowledgment of completion from the
     *                  memory subsystem.
     * @param addr      Address for write.
     * @param size      The size of the data written.
     */
    void completeWrite(Serial serial, Tick complete, Addr addr, size_t size);

    /**
     * Aborts a previously started write transaction.
     *
     * @param serial    A serial of a write that was previously started and
     *                  matches the address of the previously started write.
     * @param addr      Address for write.
     * @param size      The size of the data written.
     */
    void abortWrite(Serial serial, Addr addr, size_t size);

    /**
     * Resets the entire checker. Note that if there are transactions
     * in-flight, this will cause a warning to be issued if these are completed
     * after the reset. This does not reset nextSerial to avoid such a race
     * condition: where a transaction started before a reset with serial S,
     * then reset() was called, followed by a start of a transaction with the
     * same serial S and then receive a completion of the transaction before
     * the reset with serial S.
     */
    void reset()
    { byte_trackers.clear(); }

    /**
     * Resets an address-range. This may be useful in case other unmonitored
     * parts of the system caused modification to this memory, but we cannot
     * track their written values.
     *
     * @param addr Address base.
     * @param size Size of range to be invalidated.
     */
    void reset(Addr addr, size_t size);

    /**
     * In completeRead, if an error is encountered, this does not print nor
     * cause an error, but instead should be handled by the caller. However, to
     * record information about the cause of an error, completeRead creates an
     * errorMessage. This function returns the last error that was detected in
     * completeRead.
     *
     * @return Reference to string of error message.
     */
    const std::string& getErrorMessage() const { return errorMessage; }

  private:
    /**
     * Returns the instance of ByteTracker for the requested location.
     */
    ByteTracker* getByteTracker(Addr addr)
    {
        auto it = byte_trackers.find(addr);
        if (it == byte_trackers.end()) {
            it = byte_trackers.insert(
                std::make_pair(addr, ByteTracker(addr, this))).first;
        }
        return &it->second;
    };

  private:
    /**
     * Detailed error message of the last violation in completeRead.
     */
    std::string errorMessage;

    /**
     * Next distinct serial to be assigned to the next transaction to be
     * started.
     */
    Serial nextSerial;

    /**
     * Maintain a map of address --> byte-tracker. Per-byte entries are
     * initialized as needed.
     *
     * The required space for this obviously grows with the number of distinct
     * addresses used for a particular workload. The used size is independent on
     * the number of nodes in the system, those may affect the size of per-byte
     * tracking information.
     *
     * Access via getByteTracker()!
     */
    std::unordered_map<Addr, ByteTracker> byte_trackers;
};

inline MemChecker::Serial
MemChecker::startRead(Tick start, Addr addr, size_t size)
{
    DPRINTF(MemChecker,
            "starting read: serial = %d, start = %d, addr = %#llx, "
            "size = %d\n", nextSerial, start, addr , size);

    for (size_t i = 0; i < size; ++i) {
        getByteTracker(addr + i)->startRead(nextSerial, start);
    }

    return nextSerial++;
}

inline MemChecker::Serial
MemChecker::startWrite(Tick start, Addr addr, size_t size, const uint8_t *data)
{
    DPRINTF(MemChecker,
            "starting write: serial = %d, start = %d, addr = %#llx, "
            "size = %d\n", nextSerial, start, addr, size);

    for (size_t i = 0; i < size; ++i) {
        getByteTracker(addr + i)->startWrite(nextSerial, start, data[i]);
    }

    return nextSerial++;
}

inline void
MemChecker::completeWrite(MemChecker::Serial serial, Tick complete,
                          Addr addr, size_t size)
{
    DPRINTF(MemChecker,
            "completing write: serial = %d, complete = %d, "
            "addr = %#llx, size = %d\n", serial, complete, addr, size);

    for (size_t i = 0; i < size; ++i) {
        getByteTracker(addr + i)->completeWrite(serial, complete);
    }
}

inline void
MemChecker::abortWrite(MemChecker::Serial serial, Addr addr, size_t size)
{
    DPRINTF(MemChecker,
            "aborting write: serial = %d, addr = %#llx, size = %d\n",
            serial, addr, size);

    for (size_t i = 0; i < size; ++i) {
        getByteTracker(addr + i)->abortWrite(serial);
    }
}

#endif // __MEM_MEM_CHECKER_HH__
