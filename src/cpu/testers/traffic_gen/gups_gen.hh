/*
 * Copyright (c) 2021 The Regents of the University of California.
 * All rights reserved.
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

#ifndef __CPU_TESTERS_TRAFFIC_GEN_GUPS_GEN_HH__
#define __CPU_TESTERS_TRAFFIC_GEN_GUPS_GEN_HH__

/**
 * @file gups_gen.hh
 * Contatins the description of the class GUPSGen. GUPSGen is a simobject
 * that will generate a memory access pattern to that of RandomAccess benchmark
 * specified by HPCC benchmarks.
 * Find more details: [https://icl.cs.utk.edu/projectsfiles/hpcc/RandomAccess/]
 */

#include <queue>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/GUPSGen.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

namespace gem5
{

class GUPSGen : public ClockedObject
{
  private:

    /**
     * @brief definition of the GenPort class which is of the type RequestPort.
     * It defines the functionalities required for outside communication of
     * memory packets from the GUPSGen simobject. It also implements
     * functionalities of receiving responses from outside.
     */
    class GenPort : public RequestPort
    {
      private:

        /**
         * @brief Pointer to the GUPSGen this port belongs to
         */
        GUPSGen *owner;

        /**
         * @brief Boolean value to remember if the port is previously blocked
         * and is occupied by a previous request, it should not accept new
         * requests if it is blocked, instead, it will be freed by a
         * recvReqRetry call from outside.
         */
        bool _blocked;

        /**
         * @brief Pointer to the blocked packet in the port. It is initialized
         * as nullptr and will be set to the address of the packet which is
         * blocked. It will reset upon recvReqRetry call.
         */
        PacketPtr blockedPacket;

      public:

        GenPort(const std::string& name, GUPSGen *owner) :
            RequestPort(name, owner), owner(owner), _blocked(false),
            blockedPacket(nullptr)
        {}

        /**
         * @brief Return whether the port is blocked.
         *
         * @return true if the port is blocked.
         * @return false if the port is free.
         */
        bool blocked(){
            return _blocked;
        }

        /**
         * @brief This function send a timing request to the port's peer
         * responsePort
         * @param pkt Pointer to the packet for the timing request
         */
        void sendTimingPacket(PacketPtr pkt);

        /**
         * @brief This function send a functional request to the port's peer
         * responsePort
         * @param pkt Pointer to the packet for the timing request
         */
        void sendFunctionalPacket(PacketPtr pkt);

      protected:

        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    virtual void init() override;

    virtual void startup() override;

    /**
     * @brief Convert and index from array to its physical address
     * in the memory
     * @param index Index of the element in the array
     * @return Addr Physical address corresponding to the index.
     */
    Addr indexToAddr (uint64_t index);

    /**
     * @brief Generate a read request to be sent to the outside.
     *
     * @param addr Physical address to which the request is sent to
     * @param size The number of bytes to be read by the request.
     * @return Pointer to the generated packet. This packet includes
     * the request to the outside.
     */
    PacketPtr getReadPacket(Addr addr, unsigned int size);

    /**
     * @brief Generate a write request to be sent to the outside.
     *
     * @param addr Physical address to which the request is sent to
     * @param size The number of bytes to be written by the request.
     * @param data Pointer to the data that should be written.
     * @return Pointer to the generated packet. This packet includes
     * the request to the outside.
     */
    PacketPtr getWritePacket(Addr addr, unsigned int size, uint8_t* data);

    /**
     * @brief Handles the incoming responses from the outside.
     * @param pkt Pointer to the packet that includes the response.
     */
    void handleResponse(PacketPtr pkt);

    /**
     * @brief This function allows the port to wake its owner GUPSGen object
     * in case it has stopped working due to back pressure, it will wake up
     * as soon as back pressure is resolved.
     */
    void wakeUp();

    /**
     * @brief Create the next request and store in the requestPool.
     */
    void createNextReq();

    /**
     * @brief Corresponding event to the createNextReq function. Scheduled
     * whenever a request needs to and can be created.
     */
    EventFunctionWrapper nextCreateEvent;

    /**
     * @brief Send outstanding requests from requestPool to the port.
     */
    void sendNextReq();

    /**
     * @brief Corresponding event to the sendNextReq function. Scheduled
     * whenever a request needs to and can be sent.
     */
    EventFunctionWrapper nextSendEvent;

    /**
     * @brief Use an unordered map to store future updates on current reads
     * as the updated value depends on the index which is know when a read
     * request is created. The respective write will use the value stored to
     * update the value.
     */
    std::unordered_map<RequestPtr, uint64_t> updateTable;

    /**
     * @brief Use an unordered map to track the time at which each request
     * exits the GUPSGen. This is useful when we need to compute request
     * latency.
     */
    std::unordered_map<RequestPtr, Tick> exitTimes;

    /**
     * @brief A queue to store the outstanding requests whether read or write.
     * The element at the front of the queue is sent to outside everytime
     * nextSendEvent is scheduled.
     */
    std::queue<PacketPtr> requestPool;

    /**
     * @brief A queue to store response packets from reads. each response in
     * the response pool will generate a write request. The write request
     * updates the value of data in the response packet with its corresponding
     * value in the update table.
     */
    std::queue<PacketPtr> responsePool;

    /**
     * @brief The total number of updates (one read and one write) to do for
     * running the benchmark to completion.
     */
    int64_t numUpdates;

    /**
     * @brief Number of elements in the allocated array.
     */
    int64_t tableSize;

    /**
     * @brief Pointer to the system object this GUPSGen belongs to, the system
     * object is used to acquire a requestorId.
     */
    System *const system;

    /**
     * @brief Used to identify each requestor in a system object. Every GUPSGen
     * has its own unique requestorId.
     */
    const RequestorID requestorId;

    /**
     * @brief An instance of GenPort to communicate with the outside.
     */
    GenPort port;

    /**
     * @brief The beginning address for allocating the array.
     */
    Addr startAddr;

    /**
     * @brief Size of the memory in bytes that will be allocated for the array.
     */
    const uint32_t memSize;

    /**
     * @brief The number of updates to do before creating an exit event.
     */
    int updateLimit;

    /**
     * @brief size of each element in the array (in bytes). Every element
     * should be uint64_t, therefore this is equal to size_of(uint64_t) = 8
     */
    const int elementSize;

    /**
     * @brief  The maximum number of outstanding requests (i.e. maximum length
     * of the requestPool), specified as 1024 by the HPCC benchmark.
     */
    int reqQueueSize;

    /**
     * @brief Boolean value to determine whether we need to initialize the
     * array with the right values, as we don't care about the values, this
     * is defaulted to false in python wrapper for this simobject.
     */
    bool initMemory;

    /**
     * @brief Boolean to indicate whether the generator is done creating read
     * requests, which means number of reads equal either updateLimit or
     * numUpdates.
     */
    bool doneReading;

    /**
     * @brief The number of requests that have existed this GUPSGen and have
     * no corresponding response (they are being serviced some where in the
     * system).
     */
    int onTheFlyRequests;

    /**
     * @brief The number of read requests currently created.
     */
    int readRequests;

    struct GUPSGenStat : public statistics::Group
    {
        GUPSGenStat(GUPSGen* parent);
        void regStats() override;

        statistics::Scalar totalUpdates;
        statistics::Formula GUPS;

        statistics::Scalar totalReads;
        statistics::Scalar totalBytesRead;
        statistics::Formula avgReadBW;
        statistics::Scalar totalReadLat;
        statistics::Formula avgReadLat;

        statistics::Scalar totalWrites;
        statistics::Scalar totalBytesWritten;
        statistics::Formula avgWriteBW;
        statistics::Scalar totalWriteLat;
        statistics::Formula avgWriteLat;
    } stats;

  public:

    GUPSGen(const GUPSGenParams &params);

    Port &getPort(const std::string &if_name,
                PortID idx=InvalidPortID) override;
};

}

#endif // __CPU_TESTERS_TRAFFIC_GEN_GUPS_GEN_HH__
