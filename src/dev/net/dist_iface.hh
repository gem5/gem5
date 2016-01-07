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
 * The interface class for dist gem5 simulations.
 *
 * dist-gem5 is an extension to gem5 to enable parallel simulation of a
 * distributed system (e.g. simulation of a pool of machines
 * connected by Ethernet links). A dist gem5 run consists of seperate gem5
 * processes running in parallel. Each gem5 process executes
 * the simulation of a component of the simulated distributed system.
 * (An example component can be a dist-core board with an Ethernet NIC.)
 * The DistIface class below provides services to transfer data and
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
 * This interface is an abstract class. It can work with various low level
 * send/receive service implementations (e.g. TCP/IP, MPI,...). A TCP
 * stream socket version is implemented in src/dev/net/tcp_iface.[hh,cc].
 */
#ifndef __DEV_DIST_IFACE_HH__
#define __DEV_DIST_IFACE_HH__

#include <array>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "dev/net/dist_packet.hh"
#include "dev/net/etherpkt.hh"
#include "sim/core.hh"
#include "sim/drain.hh"
#include "sim/global_event.hh"
#include "sim/serialize.hh"

class EventManager;

/**
 * The interface class to talk to peer gem5 processes.
 */
class DistIface : public Drainable, public Serializable
{
  public:
    typedef DistHeaderPkt::Header Header;

  protected:
    typedef DistHeaderPkt::MsgType MsgType;
    typedef DistHeaderPkt::ReqType ReqType;

  private:
    class SyncEvent;
    /** @class Sync
     * This class implements global sync operations among gem5 peer processes.
     *
     * @note This class is used as a singleton object (shared by all DistIface
     * objects).
     */
    class Sync : public Serializable
    {
      protected:
        /**
         * The lock to protect access to the Sync object.
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
         * Flag is set if exit is permitted upon sync completion
         */
        bool doExit;
        /**
         * Flag is set if taking a ckpt is permitted upon sync completion
         */
        bool doCkpt;
        /**
         * The repeat value for the next periodic sync
         */
        Tick nextRepeat;
        /**
         * Tick for the very first periodic sync
         */
        Tick firstAt;
        /**
         * Tick for the next periodic sync (if the event is not scheduled yet)
         */
        Tick nextAt;

        friend class SyncEvent;

      public:
        /**
         * Initialize periodic sync params.
         *
         * @param start Start tick for dist synchronisation
         * @param repeat Frequency of dist synchronisation
         *
         */
        void init(Tick start, Tick repeat);
        /**
         *  Core method to perform a full dist sync.
         */
        virtual void run(bool same_tick) = 0;
        /**
         * Callback when the receiver thread gets a sync ack message.
         */
        virtual void progress(Tick send_tick,
                              Tick next_repeat,
                              ReqType do_ckpt,
                              ReqType do_exit) = 0;

        virtual void requestCkpt(ReqType req) = 0;
        virtual void requestExit(ReqType req) = 0;

        void drainComplete();

        virtual void serialize(CheckpointOut &cp) const override = 0;
        virtual void unserialize(CheckpointIn &cp) override = 0;
    };

    class SyncNode: public Sync
    {
      private:
        /**
         * Exit requested
         */
        ReqType needExit;
        /**
         * Ckpt requested
         */
        ReqType needCkpt;

      public:

        SyncNode();
        ~SyncNode() {}
        void run(bool same_tick) override;
        void progress(Tick max_req_tick,
                      Tick next_repeat,
                      ReqType do_ckpt,
                      ReqType do_exit) override;

        void requestCkpt(ReqType req) override;
        void requestExit(ReqType req) override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    class SyncSwitch: public Sync
    {
      private:
        /**
         * Counter for recording exit requests
         */
        unsigned numExitReq;
        /**
         * Counter for recording ckpt requests
         */
        unsigned numCkptReq;
        /**
         *  Number of connected simulated nodes
         */
        unsigned numNodes;

      public:
        SyncSwitch(int num_nodes);
        ~SyncSwitch() {}

        void run(bool same_tick) override;
        void progress(Tick max_req_tick,
                      Tick next_repeat,
                      ReqType do_ckpt,
                      ReqType do_exit) override;

        void requestCkpt(ReqType) override {
            panic("Switch requested checkpoint");
        }
        void requestExit(ReqType) override {
            panic("Switch requested exit");
        }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    /**
     * The global event to schedule periodic dist sync. It is used as a
     * singleton object.
     *
     * The periodic synchronisation works as follows.
     * 1. A SyncEvent is scheduled as a global event when startup() is
     * called.
     * 2. The process() method of the SyncEvent initiates a new barrier
     * for each simulated Ethernet link.
     * 3. Simulation thread(s) then waits until all receiver threads
     * complete the ongoing barrier. The global sync event is done.
     */
    class SyncEvent : public GlobalSyncEvent
    {
      private:
        /**
         * Flag to set when the system is draining
         */
        bool _draining;
      public:
        /**
         * Only the firstly instantiated DistIface object will
         * call this constructor.
         */
        SyncEvent() : GlobalSyncEvent(Sim_Exit_Pri, 0), _draining(false) {}

        ~SyncEvent() {}
        /**
         * Schedule the first periodic sync event.
         */
        void start();
        /**
         * This is a global event so process() will only be called by
         * exactly one simulation thread. (See further comments in the .cc
         * file.)
         */
        void process() override;

        bool draining() const { return _draining; }
        void draining(bool fl) { _draining = fl; }
    };
    /**
     * Class to encapsulate information about data packets received.

     * @note The main purpose of the class to take care of scheduling receive
     * done events for the simulated network link and store incoming packets
     * until they can be received by the simulated network link.
     */
    class RecvScheduler : public Serializable
    {
      private:
        /**
         * Received packet descriptor. This information is used by the receive
         * thread to schedule receive events and by the simulation thread to
         * process those events.
         */
        struct Desc : public Serializable
        {
            EthPacketPtr packet;
            Tick sendTick;
            Tick sendDelay;

            Desc() : sendTick(0), sendDelay(0) {}
            Desc(EthPacketPtr p, Tick s, Tick d) :
                packet(p), sendTick(s), sendDelay(d) {}
            Desc(const Desc &d) :
                packet(d.packet), sendTick(d.sendTick), sendDelay(d.sendDelay) {}

            void serialize(CheckpointOut &cp) const override;
            void unserialize(CheckpointIn &cp) override;
        };
        /**
         * The queue to store the receive descriptors.
         */
        std::queue<Desc> descQueue;
        /**
         * The tick when the most recent receive event was processed.
         *
         * @note This information is necessary to simulate possible receiver
         * link contention when calculating the receive tick for the next
         * incoming data packet (see the calcReceiveTick() method)
         */
        Tick prevRecvTick;
        /**
         * The receive done event for the simulated Ethernet link.
         *
         * @note This object is constructed by the simulated network link. We
         * schedule this object for each incoming data packet.
         */
        Event *recvDone;
        /**
         * The link delay in ticks for the simulated Ethernet link.
         *
         * @note This value is used for calculating the receive ticks for
         * incoming data packets.
         */
        Tick linkDelay;
        /**
         * The event manager associated with the simulated Ethernet link.
         *
         * @note It is used to access the event queue for scheduling receive
         * done events for the link.
         */
        EventManager *eventManager;
        /**
         * Calculate the tick to schedule the next receive done event.
         *
         * @param send_tick The tick the packet was sent.
         * @param send_delay The simulated delay at the sender side.
         * @param prev_recv_tick Tick when the last receive event was
         * processed.
         *
         * @note This method tries to take into account possible receiver link
         * contention and adjust receive tick for the incoming packets
         * accordingly.
         */
        Tick calcReceiveTick(Tick send_tick,
                             Tick send_delay,
                             Tick prev_recv_tick);

        /**
         * Flag to set if receive ticks for pending packets need to be
         * recalculated due to changed link latencies at a resume
         */
        bool ckptRestore;

      public:
        /**
         * Scheduler for the incoming data packets.
         *
         * @param em The event manager associated with the simulated Ethernet
         * link.
         */
        RecvScheduler(EventManager *em) :
            prevRecvTick(0), recvDone(nullptr), linkDelay(0),
            eventManager(em), ckptRestore(false) {}

        /**
         *  Initialize network link parameters.
         *
         * @note This method is called from the receiver thread (see
         * recvThreadFunc()).
         */
        void init(Event *recv_done, Tick link_delay);
        /**
         * Fetch the next packet that is to be received by the simulated network
         * link.
         *
         * @note This method is called from the process() method of the receive
         * done event associated with the network link.
         */
        EthPacketPtr popPacket();
        /**
         * Push a newly arrived packet into the desc queue.
         */
        void pushPacket(EthPacketPtr new_packet,
                        Tick send_tick,
                        Tick send_delay);

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
        /**
         * Adjust receive ticks for pending packets when restoring from a
         * checkpoint
         *
         * @note Link speed and delay parameters may change at resume.
         */
        void resumeRecvTicks();
    };
    /**
     * Tick to schedule the first dist sync event.
     * This is just as optimization : we do not need any dist sync
     * event until the simulated NIC is brought up by the OS.
     */
    Tick syncStart;
    /**
     * Frequency of dist sync events in ticks.
     */
    Tick syncRepeat;
    /**
     * Receiver thread pointer.
     * Each DistIface object must have exactly one receiver thread.
     */
    std::thread *recvThread;
    /**
     * Meta information about data packets received.
     */
    RecvScheduler recvScheduler;

  protected:
    /**
     * The rank of this process among the gem5 peers.
     */
    unsigned rank;
    /**
     * The number of gem5 processes comprising this dist simulation.
     */
    unsigned size;
    /**
     * Number of DistIface objects (i.e. dist links in this gem5 process)
     */
    static unsigned distIfaceNum;
    /**
     * Unique id for the dist link
     */
    unsigned distIfaceId;

    bool isMaster;

  private:
    /**
     * Number of receiver threads (in this gem5 process)
     */
    static unsigned recvThreadsNum;
    /**
     * The singleton Sync object to perform dist synchronisation.
     */
    static Sync *sync;
    /**
     * The singleton SyncEvent object to schedule periodic dist sync.
     */
    static SyncEvent *syncEvent;
    /**
     * The very first DistIface object created becomes the master. We need
     * a master to co-ordinate the global synchronisation.
     */
    static DistIface *master;

  private:
    /**
     * Send out a data packet to the remote end.
     * @param header Meta info about the packet (which needs to be transferred
     * to the destination alongside the packet).
     * @param packet Pointer to the packet to send.
     */
    virtual void sendPacket(const Header &header, const EthPacketPtr &packet) = 0;
    /**
     * Send out a control command to the remote end.
     * @param header Meta info describing the command (e.g. sync request)
     */
    virtual void sendCmd(const Header &header) = 0;
    /**
     * Receive a header (i.e. meta info describing a data packet or a control command)
     * from the remote end.
     * @param header The meta info structure to store the incoming header.
     */
    virtual bool recvHeader(Header &header) = 0;
    /**
     * Receive a packet from the remote end.
     * @param header Meta info about the incoming packet (obtanied by a previous
     * call to the recvHedaer() method).
     * @param Pointer to packet received.
     */
    virtual void recvPacket(const Header &header, EthPacketPtr &packet) = 0;
    /**
     * Init hook for the underlaying transport
     */
    virtual void initTransport() = 0;
    /**
     * spawn the receiver thread.
     * @param recv_done The receive done event associated with the simulated
     * Ethernet link.
     * @param link_delay The link delay for the simulated Ethernet link.
     */
    void spawnRecvThread(const Event *recv_done, Tick link_delay);
    /**
     * The function executed by a receiver thread.
     */
    void recvThreadFunc(Event *recv_done, Tick link_delay);

  public:

    /**
     * ctor
     * @param dist_rank Rank of this gem5 process within the dist run
     * @param sync_start Start tick for dist synchronisation
     * @param sync_repeat Frequency for dist synchronisation
     * @param em The event manager associated with the simulated Ethernet link
     */
    DistIface(unsigned dist_rank,
              unsigned dist_size,
              Tick sync_start,
              Tick sync_repeat,
              EventManager *em,
              bool is_switch,
              int num_nodes);

    virtual ~DistIface();
    /**
     * Send out an Ethernet packet.
     * @param pkt The Ethernet packet to send.
     * @param send_delay The delay in ticks for the send completion event.
     */
    void packetOut(EthPacketPtr pkt, Tick send_delay);
    /**
     * Fetch the packet scheduled to be received next by the simulated
     * network link.
     *
     * @note This method is called within the process() method of the link
     * receive done event. It also schedules the next receive event if the
     * receive queue is not empty.
     */
    EthPacketPtr packetIn() { return recvScheduler.popPacket(); }

    DrainState drain() override;
    void drainResume() override;
    void init(const Event *e, Tick link_delay);
    void startup();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    /**
     * Initiate the exit from the simulation.
     * @param delay Delay param from the m5 exit command. If Delay is zero
     * then a collaborative exit is requested (i.e. all nodes have to call
     * this method before the distributed simulation can exit). If Delay is
     * not zero then exit is requested asap (and it will happen at the next
     * sync tick).
     * @return False if we are in distributed mode (i.e. exit can happen only
     * at sync), True otherwise.
     */
    static bool readyToExit(Tick delay);
    /**
     * Initiate taking a checkpoint
     * @param delay Delay param from the m5 checkpoint command. If Delay is
     * zero then a collaborative checkpoint is requested (i.e. all nodes have
     * to call this method before the checkpoint can be taken). If Delay is
     * not zero then a checkpoint is requested asap (and it will happen at the
     * next sync tick).
     * @return False if we are in dist mode (i.e. exit can happen only at
     * sync), True otherwise.
     */
    static bool readyToCkpt(Tick delay, Tick period);
    /**
     * Getter for the dist rank param.
     */
    static uint64_t rankParam();
    /**
     * Getter for the dist size param.
     */
    static uint64_t sizeParam();
 };

#endif
