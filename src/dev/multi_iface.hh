/*
 * Copyright (c) 2015 ARM Limited
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
 *
 * Authors: Gabor Dozsa
 */

/* @file
 * The interface class for multi gem5 simulations.
 *
 * Multi gem5 is an extension to gem5 to enable parallel simulation of a
 * distributed system (e.g. simulation of a pool of machines
 * connected by Ethernet links). A multi gem5 run consists of seperate gem5
 * processes running in parallel. Each gem5 process executes
 * the simulation of a component of the simulated distributed system.
 * (An example component can be a multi-core board with an Ethernet NIC.)
 * The MultiIface class below provides services to transfer data and
 * control messages among the gem5 processes. The main such services are
 * as follows.
 *
 * 1. Send a data packet coming from a simulated Ethernet link. The packet
 * will be transferred to (all) the target(s) gem5 processes. The send
 * operation is always performed by the simulation thread, i.e. the gem5
 * thread that is processing the event queue associated with the simulated
 * Ethernet link.
 *
 * 2. Spawn a receiver thread to process messages coming in from the
 * from other gem5 processes. Each simulated Ethernet link has its own
 * associated receiver thread. The receiver thread saves the incoming packet
 * and schedule an appropriate receive event in the event queue.
 *
 * 3. Schedule a global barrier event periodically to keep the gem5
 * processes in sync.
 * Periodic barrier event to keep peer gem5 processes in sync. The basic idea
 * is that no gem5 process can go ahead further than the simulated link
 * transmission delay to ensure that a corresponding receive event can always
 * be scheduled for any message coming in from a peer gem5 process.
 *
 *
 *
 * This interface is an abstract class (sendRaw() and recvRaw()
 * methods are pure virtual). It can work with various low level
 * send/receive service implementations (e.g. TCP/IP, MPI,...). A TCP
 * stream socket version is implemented in dev/src/tcp_iface.[hh,cc].
 */
#ifndef __DEV_MULTI_IFACE_HH__
#define __DEV_MULTI_IFACE_HH__

#include <array>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "dev/etherpkt.hh"
#include "dev/multi_packet.hh"
#include "sim/core.hh"
#include "sim/drain.hh"
#include "sim/global_event.hh"

class EventManager;

/**
 * The interface class to talk to peer gem5 processes.
 */
class MultiIface : public Drainable
{
  public:
    /*!
     * The possible reasons a multi sync among gem5 peers is needed for.
     */
    enum
    class SyncTrigger {
        periodic, /*!< Regular periodic sync. This can be interrupted by a
                   checkpoint sync request */
        ckpt,     /*!< sync before taking a checkpoint */
        atomic    /*!< sync that cannot be interrupted (e.g. sync at startup) */
    };

  private:
    typedef MultiHeaderPkt::MsgType MsgType;

    /** Sync State-Machine
     \dot
     digraph Sync {
     node [shape=box, fontsize=10];
     idle -> busy
     [ label="new trigger\n by run()" fontsize=8 ];
     busy -> busy
     [ label="new message by progress():\n(msg == SyncAck &&\nwaitNum > 1) || \n(msg==CkptSyncReq &&\ntrigger == ckpt)" fontsize=8 ];
     busy -> idle
     [ label="new message by progress():\n(msg == SyncAck &&\nwaitNum == 1)" fontsize=8 ];
     busy -> interrupted
     [ label="new message by progress():\n(msg == CkptSyncReq &&\ntrigger == periodic)" fontsize=8 ];
     idle -> asyncCkpt
     [ label="new message by progress():\nmsg == CkptSyncReq" fontsize=8 ];
     asyncCkpt -> asyncCkpt
     [ label="new message by progress():\nmsg == CkptSyncReq" fontsize=8 ];
     asyncCkpt -> busy
     [ label="new trigger by run():\ntrigger == ckpt" fontsize=8 ];
     asyncCkpt -> idle
     [ label="new trigger by run():\n(trigger == periodic &&\nwaitNum == 0) " fontsize=8 ];
     asyncCkpt -> interrupted
     [ label="new trigger by run():\n(trigger == periodic &&\nwaitNum > 0) " fontsize=8 ];
     interrupted -> interrupted
     [ label="new message by progress():\n(msg == CkptSyncReq &&\nwaitNum > 1)" fontsize=8 ];
     interrupted -> idle
     [ label="new message by progress():\n(msg == CkptSyncReq &&\nwaitNum == 1)" fontsize=8 ];
     }
     \enddot
     */
    /** @class Sync
     * This class implements global sync operations among gem5 peer processes.
     *
     * @note This class is used as a singleton object (shared by all MultiIface
     * objects).
     */
    class Sync
    {
      private:
        /*!
         * Internal state of the sync singleton object.
         */
        enum class SyncState {
            busy,        /*!< There is an on-going sync. */
            interrupted, /*!< An on-going periodic sync was interrupted. */
            asyncCkpt,   /*!< A checkpoint (sim_exit) is already scheduled */
            idle         /*!< There is no active sync. */
        };
        /**
         * The lock to protect access to the MultiSync object.
         */
        std::mutex lock;
        /**
         * Condition variable for the simulation thread to wait on
         * until all receiver threads completes the current global
         * synchronisation.
         */
        std::condition_variable cv;
        /**
         * Number of receiver threads that not yet completed the current global
         * synchronisation.
         */
        unsigned waitNum;
        /**
         * The trigger for the most recent sync.
         */
        SyncTrigger trigger;
        /**
         * Map sync triggers to request messages.
         */
        std::array<MsgType, 3> triggerToMsg = {{
                MsgType::cmdPeriodicSyncReq,
                MsgType::cmdCkptSyncReq,
                MsgType::cmdAtomicSyncReq
            }};

        /**
         * Current sync state.
         */
        SyncState state;

      public:
        /**
         *  Core method to perform a full multi sync.
         *
         * @param t Sync trigger.
         * @param sync_tick The tick the sync was expected to happen at.
         * @return true if the sync completed, false if it was interrupted.
         *
         * @note In case of an interrupted periodic sync, sync_tick can be less
         * than curTick() when we resume (i.e. re-run) it
         */
        bool run(SyncTrigger t, Tick sync_tick);
        /**
         * Callback when the receiver thread gets a sync message.
         */
        void progress(MsgType m);

        Sync() : waitNum(0), state(SyncState::idle) {}
        ~Sync() {}
    };


    /**
     * The global event to schedule peridic multi sync. It is used as a
     * singleton object.
     *
     * The periodic synchronisation works as follows.
     * 1. A MultisyncEvent is scheduled as a global event when startup() is
     * called.
     * 2. The progress() method of the MultisyncEvent initiates a new barrier
     * for each simulated Ethernet links.
     * 3. Simulation thread(s) then waits until all receiver threads
     * completes the ongoing barrier. The global sync event is done.
     */
    class SyncEvent : public GlobalSyncEvent
    {
      public:
        /**
         * Flag to indicate that the most recent periodic sync was interrupted
         * (by a checkpoint request).
         */
        bool interrupted;
        /**
         * The tick when the most recent periodic synchronisation was scheduled
         * at.
         */
        Tick scheduledAt;
        /**
         * Flag to indicate an on-going drain cycle.
         */
         bool isDraining;

      public:
        /**
         * Only the firstly instanstiated MultiIface object will
         * call this constructor.
         */
        SyncEvent() : GlobalSyncEvent(Default_Pri, 0), interrupted(false),
                      scheduledAt(0), isDraining(false) {}

        ~SyncEvent() { assert (scheduled() == false); }
        /**
         * Schedule the first periodic sync event.
         *
         * @param start Start tick for multi synchronisation
         * @param repeat Frequency of multi synchronisation
         *
         */
        void start(Tick start, Tick repeat);
        /**
         * Reschedule (if necessary) the periodic sync event.
         *
         * @param start Start tick for multi synchronisation
         * @param repeat Frequency of multi synchronisation
         *
         * @note Useful if we have multiple MultiIface objects with
         * different 'start' and 'repeat' values for global sync.
         */
        void adjust(Tick start, Tick repeat);
        /**
         * This is a global event so process() will be called by each
         * simulation threads. (See further comments in the .cc file.)
         */
        void process() override;
        /**
         * Schedule periodic sync when resuming from a checkpoint.
         */
        void resume();

        void serialize(const std::string &base, CheckpointOut &cp) const;
        void unserialize(const std::string &base, CheckpointIn &cp);
    };

    /**
     * The receive thread needs to store the packet pointer and the computed
     * receive tick for each incoming data packet. This information is used
     * by the simulation thread when it processes the corresponding receive
     * event. (See more comments at the implemetation of the recvThreadFunc()
     * and RecvPacketIn() methods.)
     */
    typedef std::pair<EthPacketPtr, Tick> RecvInfo;

    /**
     * Comparison predicate for RecvInfo, needed by the recvQueue.
     */
    struct RecvInfoCompare {
        bool operator()(const RecvInfo &lhs, const RecvInfo &rhs)
        {
            return lhs.second > rhs.second;
        }
    };

    /**
     * Customized priority queue used to store incoming data packets info by
     * the receiver thread. We need to expose the underlying container to
     * enable iterator access for serializing.
     */
    class RecvQueue : public std::priority_queue<RecvInfo,
                                                 std::vector<RecvInfo>,
                                                 RecvInfoCompare>
    {
      public:
        std::vector<RecvInfo> &impl() { return c; }
        const std::vector<RecvInfo> &impl() const { return c; }
    };

    /*
     * The priority queue to store RecvInfo items ordered by receive ticks.
     */
    RecvQueue recvQueue;
    /**
     * The singleton Sync object to perform multi synchronisation.
     */
    static Sync *sync;
    /**
     * The singleton SyncEvent object to schedule periodic multi sync.
     */
    static SyncEvent *syncEvent;
    /**
     * Tick to schedule the first multi sync event.
     * This is just as optimization : we do not need any multi sync
     * event until the simulated NIC is brought up by the OS.
     */
    Tick syncStart;
    /**
     * Frequency of multi sync events in ticks.
     */
    Tick syncRepeat;
    /**
     * Receiver thread pointer.
     * Each MultiIface object must have exactly one receiver thread.
     */
    std::thread *recvThread;
    /**
     * The event manager associated with the MultiIface object.
     */
    EventManager *eventManager;

    /**
     * The receive done event for the simulated Ethernet link.
     * It is scheduled by the receiver thread for each incoming data
     * packet.
     */
    Event *recvDone;

    /**
     * The packet that belongs to the currently scheduled recvDone event.
     */
    EthPacketPtr scheduledRecvPacket;

    /**
     * The link delay in ticks for the simulated Ethernet link.
     */
    Tick linkDelay;

    /**
     * The rank of this process among the gem5 peers.
     */
    unsigned rank;
    /**
     * Total number of receiver threads (in this gem5 process).
     * During the simulation it should be constant and equal to the
     * number of MultiIface objects (i.e. simulated Ethernet
     * links).
     */
    static unsigned recvThreadsNum;
    /**
     * The very first MultiIface object created becomes the master. We need
     * a master to co-ordinate the global synchronisation.
     */
    static MultiIface *master;

  protected:
    /**
     * Low level generic send routine.
     * @param buf buffer that holds the data to send out
     * @param length number of bytes to send
     * @param dest_addr address of the target (simulated NIC). This may be
     * used by a subclass for optimization (e.g. optimize broadcast)
     */
    virtual void sendRaw(void *buf,
                         unsigned length,
                         const MultiHeaderPkt::AddressType dest_addr) = 0;
    /**
     * Low level generic receive routine.
     * @param buf the buffer to store the incoming message
     * @param length buffer size (in bytes)
     */
    virtual bool recvRaw(void *buf, unsigned length) = 0;
    /**
     * Low level request for synchronisation among gem5 processes. Only one
     * MultiIface object needs to call this (in each gem5 process) to trigger
     * a multi sync.
     *
     * @param sync_req Sync request command.
     * @param sync_tick The tick when sync is expected to happen in the sender.
     */
    virtual void syncRaw(MsgType sync_req, Tick sync_tick) = 0;
    /**
     * The function executed by a receiver thread.
     */
    void recvThreadFunc();
    /**
     * Receive a multi header packet. Called by the receiver thread.
     * @param header the structure to store the incoming header packet.
     * @return false if any error occured during the receive, true otherwise
     *
     * A header packet can carry a control command (e.g. 'barrier leave') or
     * information about a data packet that is following the header packet
     * back to back.
     */
    bool recvHeader(MultiHeaderPkt::Header &header);
    /**
     * Receive a data packet. Called by the receiver thread.
     * @param data_header The packet descriptor for the expected incoming data
     * packet.
     */
    void recvData(const MultiHeaderPkt::Header &data_header);

  public:

    /**
     * ctor
     * @param multi_rank Rank of this gem5 process within the multi run
     * @param sync_start Start tick for multi synchronisation
     * @param sync_repeat Frequency for multi synchronisation
     * @param em The event manager associated with the simulated Ethernet link
     */
    MultiIface(unsigned multi_rank,
               Tick sync_start,
               Tick sync_repeat,
               EventManager *em);

    virtual ~MultiIface();
    /**
     * Send out an Ethernet packet.
     * @param pkt The Ethernet packet to send.
     * @param send_delay The delay in ticks for the send completion event.
     */
    void packetOut(EthPacketPtr pkt, Tick send_delay);
    /**
     * Fetch the next packet from the receive queue.
     */
    EthPacketPtr packetIn();

    /**
     * spawn the receiver thread.
     * @param recv_done The receive done event associated with the simulated
     * Ethernet link.
     * @param link_delay The link delay for the simulated Ethernet link.
     */
    void spawnRecvThread(Event *recv_done,
                         Tick link_delay);
    /**
     * Initialize the random number generator with a different seed in each
     * peer gem5 process.
     */
    void initRandom();

    DrainState drain() override;

    /**
     * Callback when draining is complete.
     */
    void drainDone();

    /**
     * Initialize the periodic synchronisation among peer gem5 processes.
     */
    void startPeriodicSync();

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);

};


#endif
