/*
 * Copyright (c) 2015-2016 ARM Limited
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

/* @file
 * The interface class for dist-gem5 simulations.
 */

#include "dev/net/dist_iface.hh"

#include <queue>
#include <thread>

#include "base/random.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/DistEthernet.hh"
#include "debug/DistEthernetPkt.hh"
#include "dev/net/etherpkt.hh"
#include "sim/cur_tick.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

DistIface::Sync *DistIface::sync = nullptr;
System *DistIface::sys = nullptr;
DistIface::SyncEvent *DistIface::syncEvent = nullptr;
unsigned DistIface::distIfaceNum = 0;
unsigned DistIface::recvThreadsNum = 0;
DistIface *DistIface::primary = nullptr;
bool DistIface::isSwitch = false;

void
DistIface::Sync::init(Tick start_tick, Tick repeat_tick)
{
    if (start_tick < nextAt) {
        nextAt = start_tick;
        inform("Next dist synchronisation tick is changed to %lu.\n", nextAt);
    }

    if (repeat_tick == 0)
        panic("Dist synchronisation interval must be greater than zero");

    if (repeat_tick < nextRepeat) {
        nextRepeat = repeat_tick;
        inform("Dist synchronisation interval is changed to %lu.\n",
               nextRepeat);
    }
}

void
DistIface::Sync::abort()
{
    std::unique_lock<std::mutex> sync_lock(lock);
    waitNum = 0;
    isAbort = true;
    sync_lock.unlock();
    cv.notify_one();
}

DistIface::SyncSwitch::SyncSwitch(int num_nodes)
{
    numNodes = num_nodes;
    waitNum = num_nodes;
    numExitReq = 0;
    numCkptReq = 0;
    numStopSyncReq = 0;
    doExit = false;
    doCkpt = false;
    doStopSync = false;
    nextAt = std::numeric_limits<Tick>::max();
    nextRepeat = std::numeric_limits<Tick>::max();
    isAbort = false;
}

DistIface::SyncNode::SyncNode()
{
    waitNum = 0;
    needExit = ReqType::none;
    needCkpt = ReqType::none;
    needStopSync = ReqType::none;
    doExit = false;
    doCkpt = false;
    doStopSync = false;
    nextAt = std::numeric_limits<Tick>::max();
    nextRepeat = std::numeric_limits<Tick>::max();
    isAbort = false;
}

bool
DistIface::SyncNode::run(bool same_tick)
{
    std::unique_lock<std::mutex> sync_lock(lock);
    Header header;

    assert(waitNum == 0);
    assert(!isAbort);
    waitNum = DistIface::recvThreadsNum;
    // initiate the global synchronisation
    header.msgType = MsgType::cmdSyncReq;
    header.sendTick = curTick();
    header.syncRepeat = nextRepeat;
    header.needCkpt = needCkpt;
    header.needStopSync = needStopSync;
    if (needCkpt != ReqType::none)
        needCkpt = ReqType::pending;
    header.needExit = needExit;
    if (needExit != ReqType::none)
        needExit = ReqType::pending;
    if (needStopSync != ReqType::none)
        needStopSync = ReqType::pending;
    DistIface::primary->sendCmd(header);
    // now wait until all receiver threads complete the synchronisation
    auto lf = [this] { return waitNum == 0; };
    cv.wait(sync_lock, lf);
    // global synchronisation is done.
    assert(isAbort || !same_tick || (nextAt == curTick()));
    return !isAbort;
}

bool
DistIface::SyncSwitch::run(bool same_tick)
{
    std::unique_lock<std::mutex> sync_lock(lock);
    Header header;
    // Wait for the sync requests from the nodes
    if (waitNum > 0) {
        auto lf = [this] { return waitNum == 0; };
        cv.wait(sync_lock, lf);
    }
    assert(waitNum == 0);
    if (isAbort) // sync aborted
        return false;
    assert(!same_tick || (nextAt == curTick()));
    waitNum = numNodes;
    // Complete the global synchronisation
    header.msgType = MsgType::cmdSyncAck;
    header.sendTick = nextAt;
    header.syncRepeat = nextRepeat;
    if (doCkpt || numCkptReq == numNodes) {
        doCkpt = true;
        header.needCkpt = ReqType::immediate;
        numCkptReq = 0;
    } else {
        header.needCkpt = ReqType::none;
    }
    if (doExit || numExitReq == numNodes) {
        doExit = true;
        header.needExit = ReqType::immediate;
    } else {
        header.needExit = ReqType::none;
    }
    if (doStopSync || numStopSyncReq == numNodes) {
        doStopSync = true;
        numStopSyncReq = 0;
        header.needStopSync = ReqType::immediate;
    } else {
        header.needStopSync = ReqType::none;
    }
    DistIface::primary->sendCmd(header);
    return true;
}

bool
DistIface::SyncSwitch::progress(Tick send_tick, Tick sync_repeat,
                                ReqType need_ckpt, ReqType need_exit,
                                ReqType need_stop_sync)
{
    std::unique_lock<std::mutex> sync_lock(lock);
    if (isAbort) // sync aborted
        return false;
    assert(waitNum > 0);

    if (send_tick > nextAt)
        nextAt = send_tick;
    if (nextRepeat > sync_repeat)
        nextRepeat = sync_repeat;

    if (need_ckpt == ReqType::collective)
        numCkptReq++;
    else if (need_ckpt == ReqType::immediate)
        doCkpt = true;
    if (need_exit == ReqType::collective)
        numExitReq++;
    else if (need_exit == ReqType::immediate)
        doExit = true;
    if (need_stop_sync == ReqType::collective)
        numStopSyncReq++;
    else if (need_stop_sync == ReqType::immediate)
        doStopSync = true;

    waitNum--;
    // Notify the simulation thread if the on-going sync is complete
    if (waitNum == 0) {
        sync_lock.unlock();
        cv.notify_one();
    }
    // The receive thread must keep alive in the switch until the node
    // closes the connection. Thus, we always return true here.
    return true;
}

bool
DistIface::SyncNode::progress(Tick max_send_tick, Tick next_repeat,
                              ReqType do_ckpt, ReqType do_exit,
                              ReqType do_stop_sync)
{
    std::unique_lock<std::mutex> sync_lock(lock);
    if (isAbort) // sync aborted
        return false;
    assert(waitNum > 0);

    nextAt = max_send_tick;
    nextRepeat = next_repeat;
    doCkpt = (do_ckpt != ReqType::none);
    doExit = (do_exit != ReqType::none);
    doStopSync = (do_stop_sync != ReqType::none);

    waitNum--;
    // Notify the simulation thread if the on-going sync is complete
    if (waitNum == 0) {
        sync_lock.unlock();
        cv.notify_one();
    }
    // The receive thread must finish when simulation is about to exit
    return !doExit;
}

void
DistIface::SyncNode::requestCkpt(ReqType req)
{
    std::lock_guard<std::mutex> sync_lock(lock);
    assert(req != ReqType::none);
    if (needCkpt != ReqType::none)
        warn("Ckpt requested multiple times (req:%d)\n",
             static_cast<int>(req));
    if (needCkpt == ReqType::none || req == ReqType::immediate)
        needCkpt = req;
}

void
DistIface::SyncNode::requestExit(ReqType req)
{
    std::lock_guard<std::mutex> sync_lock(lock);
    assert(req != ReqType::none);
    if (needExit != ReqType::none)
        warn("Exit requested multiple times (req:%d)\n",
             static_cast<int>(req));
    if (needExit == ReqType::none || req == ReqType::immediate)
        needExit = req;
}

void
DistIface::Sync::drainComplete()
{
    if (doCkpt) {
        // The first DistIface object called this right before writing the
        // checkpoint. We need to drain the underlying physical network here.
        // Note that other gem5 peers may enter this barrier at different
        // ticks due to draining.
        run(false);
        // Only the "first" DistIface object has to perform the sync
        doCkpt = false;
    }
}

void
DistIface::SyncNode::serialize(CheckpointOut &cp) const
{
    int need_exit = static_cast<int>(needExit);
    SERIALIZE_SCALAR(need_exit);
}

void
DistIface::SyncNode::unserialize(CheckpointIn &cp)
{
    int need_exit;
    UNSERIALIZE_SCALAR(need_exit);
    needExit = static_cast<ReqType>(need_exit);
}

void
DistIface::SyncSwitch::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(numExitReq);
}

void
DistIface::SyncSwitch::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(numExitReq);
}

void
DistIface::SyncEvent::start()
{
    // Note that this may be called either from startup() or drainResume()

    // At this point, all DistIface objects has already called Sync::init() so
    // we have a local minimum of the start tick and repeat for the periodic
    // sync.
    repeat = DistIface::sync->nextRepeat;
    // Do a global barrier to agree on a common repeat value (the smallest
    // one from all participating nodes.
    if (!DistIface::sync->run(false))
        panic("DistIface::SyncEvent::start() aborted\n");

    assert(!DistIface::sync->doCkpt);
    assert(!DistIface::sync->doExit);
    assert(!DistIface::sync->doStopSync);
    assert(DistIface::sync->nextAt >= curTick());
    assert(DistIface::sync->nextRepeat <= repeat);

    if (curTick() == 0)
        assert(!scheduled());

    // Use the maximum of the current tick for all participating nodes or a
    // user provided starting tick.
    if (scheduled())
        reschedule(DistIface::sync->nextAt);
    else
        schedule(DistIface::sync->nextAt);

    inform("Dist sync scheduled at %lu and repeats %lu\n", when(),
           DistIface::sync->nextRepeat);
}

void
DistIface::SyncEvent::process()
{
    // We may not start a global periodic sync while draining before taking a
    // checkpoint.  This is due to the possibility that peer gem5 processes
    // may not hit the same periodic sync before they complete draining and
    // that would make this periodic sync clash with sync called from
    // DistIface::serialize() by other gem5 processes.
    // We would need a 'distributed drain' solution to eliminate this
    // restriction.
    // Note that if draining was not triggered by checkpointing then we are
    // fine since no extra global sync will happen (i.e. all peer gem5 will
    // hit this periodic sync eventually).
    panic_if(_draining && DistIface::sync->doCkpt,
             "Distributed sync is hit while draining");
    /*
     * Note that this is a global event so this process method will be called
     * by only exactly one thread.
     */
    /*
     * We hold the eventq lock at this point but the receiver thread may
     * need the lock to schedule new recv events while waiting for the
     * dist sync to complete.
     * Note that the other simulation threads also release their eventq
     * locks while waiting for us due to the global event semantics.
     */
    {
        EventQueue::ScopedRelease sr(curEventQueue());
        // we do a global sync here that is supposed to happen at the same
        // tick in all gem5 peers
        if (!DistIface::sync->run(true))
            return; // global sync aborted
        // global sync completed
    }
    if (DistIface::sync->doCkpt)
        exitSimLoop("checkpoint");
    if (DistIface::sync->doExit) {
        exitSimLoop("exit request from gem5 peers");
        return;
    }
    if (DistIface::sync->doStopSync) {
        DistIface::sync->doStopSync = false;
        inform("synchronization disabled at %lu\n", curTick());

        // The switch node needs to wait for the next sync immediately.
        if (DistIface::isSwitch) {
            start();
        } else {
            // Wake up thread contexts on non-switch nodes.
            for (auto *tc : primary->sys->threads) {
                if (tc->status() == ThreadContext::Suspended)
                    tc->activate();
                else
                    warn_once("Tried to wake up thread in dist-gem5, but it "
                              "was already awake!\n");
            }
        }
        return;
    }
    // schedule the next periodic sync
    repeat = DistIface::sync->nextRepeat;
    schedule(curTick() + repeat);
}

void
DistIface::RecvScheduler::init(Event *recv_done, Tick link_delay)
{
    // This is called from the receiver thread when it starts running. The new
    // receiver thread shares the event queue with the simulation thread
    // (associated with the simulated Ethernet link).
    curEventQueue(eventManager->eventQueue());

    recvDone = recv_done;
    linkDelay = link_delay;
}

Tick
DistIface::RecvScheduler::calcReceiveTick(Tick send_tick, Tick send_delay,
                                          Tick prev_recv_tick)
{
    Tick recv_tick = send_tick + send_delay + linkDelay;
    // sanity check (we need atleast a send delay long window)
    assert(recv_tick >= prev_recv_tick + send_delay);
    panic_if(prev_recv_tick + send_delay > recv_tick,
             "Receive window is smaller than send delay");
    panic_if(recv_tick <= curTick(),
             "Simulators out of sync - missed packet receive by %llu ticks"
             "(rev_recv_tick: %lu send_tick: %lu send_delay: %lu "
             "linkDelay: %lu )",
             curTick() - recv_tick, prev_recv_tick, send_tick, send_delay,
             linkDelay);

    return recv_tick;
}

void
DistIface::RecvScheduler::resumeRecvTicks()
{
    // Schedule pending packets asap in case link speed/delay changed when
    // restoring from the checkpoint.
    // This may be done during unserialize except that curTick() is unknown
    // so we call this during drainResume().
    // If we are not restoring from a checkppint then link latency could not
    // change so we just return.
    if (!ckptRestore)
        return;

    std::vector<Desc> v;
    while (!descQueue.empty()) {
        Desc d = descQueue.front();
        descQueue.pop();
        d.sendTick = curTick();
        d.sendDelay = d.packet->simLength; // assume 1 tick/byte max link speed
        v.push_back(d);
    }

    for (auto &d : v)
        descQueue.push(d);

    if (recvDone->scheduled()) {
        assert(!descQueue.empty());
        eventManager->reschedule(recvDone, curTick());
    } else {
        assert(descQueue.empty() && v.empty());
    }
    ckptRestore = false;
}

void
DistIface::RecvScheduler::pushPacket(EthPacketPtr new_packet, Tick send_tick,
                                     Tick send_delay)
{
    // Note : this is called from the receiver thread
    curEventQueue()->lock();
    Tick recv_tick = calcReceiveTick(send_tick, send_delay, prevRecvTick);

    DPRINTF(DistEthernetPkt,
            "DistIface::recvScheduler::pushPacket "
            "send_tick:%llu send_delay:%llu link_delay:%llu recv_tick:%llu\n",
            send_tick, send_delay, linkDelay, recv_tick);
    // Every packet must be sent and arrive in the same quantum
    assert(send_tick >
           primary->syncEvent->when() - primary->syncEvent->repeat);
    // No packet may be scheduled for receive in the arrival quantum
    assert(send_tick + send_delay + linkDelay > primary->syncEvent->when());

    // Now we are about to schedule a recvDone event for the new data packet.
    // We use the same recvDone object for all incoming data packets. Packet
    // descriptors are saved in the ordered queue. The currently scheduled
    // packet is always on the top of the queue.
    // NOTE:  we use the event queue lock to protect the receive desc queue,
    // too, which is accessed both by the receiver thread and the simulation
    // thread.
    descQueue.emplace(new_packet, send_tick, send_delay);
    if (descQueue.size() == 1) {
        assert(!recvDone->scheduled());
        eventManager->schedule(recvDone, recv_tick);
    } else {
        assert(recvDone->scheduled());
        panic_if(descQueue.front().sendTick + descQueue.front().sendDelay >
                     recv_tick,
                 "Out of order packet received (recv_tick: %lu top(): %lu\n",
                 recv_tick,
                 descQueue.front().sendTick + descQueue.front().sendDelay);
    }
    curEventQueue()->unlock();
}

EthPacketPtr
DistIface::RecvScheduler::popPacket()
{
    // Note : this is called from the simulation thread when a receive done
    // event is being processed for the link. We assume that the thread holds
    // the event queue queue lock when this is called!
    EthPacketPtr next_packet = descQueue.front().packet;
    descQueue.pop();

    if (descQueue.size() > 0) {
        Tick recv_tick =
            calcReceiveTick(descQueue.front().sendTick,
                            descQueue.front().sendDelay, curTick());
        eventManager->schedule(recvDone, recv_tick);
    }
    prevRecvTick = curTick();
    return next_packet;
}

void
DistIface::RecvScheduler::Desc::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(sendTick);
    SERIALIZE_SCALAR(sendDelay);
    packet->serialize("rxPacket", cp);
}

void
DistIface::RecvScheduler::Desc::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(sendTick);
    UNSERIALIZE_SCALAR(sendDelay);
    packet = std::make_shared<EthPacketData>();
    packet->unserialize("rxPacket", cp);
}

void
DistIface::RecvScheduler::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(prevRecvTick);
    // serialize the receive desc queue
    std::queue<Desc> tmp_queue(descQueue);
    unsigned n_desc_queue = descQueue.size();
    assert(tmp_queue.size() == descQueue.size());
    SERIALIZE_SCALAR(n_desc_queue);
    for (int i = 0; i < n_desc_queue; i++) {
        tmp_queue.front().serializeSection(cp, csprintf("rxDesc_%d", i));
        tmp_queue.pop();
    }
    assert(tmp_queue.empty());
}

void
DistIface::RecvScheduler::unserialize(CheckpointIn &cp)
{
    assert(descQueue.size() == 0);
    assert(!recvDone->scheduled());
    assert(!ckptRestore);

    UNSERIALIZE_SCALAR(prevRecvTick);
    // unserialize the receive desc queue
    unsigned n_desc_queue;
    UNSERIALIZE_SCALAR(n_desc_queue);
    for (int i = 0; i < n_desc_queue; i++) {
        Desc recv_desc;
        recv_desc.unserializeSection(cp, csprintf("rxDesc_%d", i));
        descQueue.push(recv_desc);
    }
    ckptRestore = true;
}

DistIface::DistIface(unsigned dist_rank, unsigned dist_size, Tick sync_start,
                     Tick sync_repeat, EventManager *em, bool use_pseudo_op,
                     bool is_switch, int num_nodes)
    : syncStart(sync_start),
      syncRepeat(sync_repeat),
      recvThread(nullptr),
      recvScheduler(em),
      syncStartOnPseudoOp(use_pseudo_op),
      rank(dist_rank),
      size(dist_size)
{
    DPRINTF(DistEthernet, "DistIface() ctor rank:%d\n", dist_rank);
    isPrimary = false;
    if (primary == nullptr) {
        assert(sync == nullptr);
        assert(syncEvent == nullptr);
        isSwitch = is_switch;
        if (is_switch)
            sync = new SyncSwitch(num_nodes);
        else
            sync = new SyncNode();
        syncEvent = new SyncEvent();
        primary = this;
        isPrimary = true;
    }
    distIfaceId = distIfaceNum;
    distIfaceNum++;
}

DistIface::~DistIface()
{
    assert(recvThread);
    recvThread->join();
    delete recvThread;
    if (distIfaceNum-- == 0) {
        assert(syncEvent);
        delete syncEvent;
        assert(sync);
        delete sync;
    }
    if (this == primary)
        primary = nullptr;
}

void
DistIface::packetOut(EthPacketPtr pkt, Tick send_delay)
{
    Header header;

    // Prepare a dist header packet for the Ethernet packet we want to
    // send out.
    header.msgType = MsgType::dataDescriptor;
    header.sendTick = curTick();
    header.sendDelay = send_delay;

    header.dataPacketLength = pkt->length;
    header.simLength = pkt->simLength;

    // Send out the packet and the meta info.
    sendPacket(header, pkt);

    DPRINTF(DistEthernetPkt,
            "DistIface::sendDataPacket() done size:%d send_delay:%llu\n",
            pkt->length, send_delay);
}

void
DistIface::recvThreadFunc(Event *recv_done, Tick link_delay)
{
    EthPacketPtr new_packet;
    DistHeaderPkt::Header header;

    // Initialize receive scheduler parameters
    recvScheduler.init(recv_done, link_delay);

    // Main loop to wait for and process any incoming message.
    for (;;) {
        // recvHeader() blocks until the next dist header packet comes in.
        if (!recvHeader(header)) {
            // We lost connection to the peer gem5 processes most likely
            // because one of them called m5 exit. So we stop here.
            // Grab the eventq lock to stop the simulation thread
            curEventQueue()->lock();
            exitSimLoop("connection to gem5 peer got closed");
            curEventQueue()->unlock();
            // The simulation thread may be blocked in processing an on-going
            // global synchronisation. Abort the sync to give the simulation
            // thread a chance to make progress and process the exit event.
            sync->abort();
            // Finish receiver thread
            break;
        }

        // We got a valid dist header packet, let's process it
        if (header.msgType == MsgType::dataDescriptor) {
            recvPacket(header, new_packet);
            recvScheduler.pushPacket(new_packet, header.sendTick,
                                     header.sendDelay);
        } else {
            // everything else must be synchronisation related command
            if (!sync->progress(header.sendTick, header.syncRepeat,
                                header.needCkpt, header.needExit,
                                header.needStopSync))
                // Finish receiver thread if simulation is about to exit
                break;
        }
    }
}

void
DistIface::spawnRecvThread(const Event *recv_done, Tick link_delay)
{
    assert(recvThread == nullptr);

    recvThread = new std::thread(&DistIface::recvThreadFunc, this,
                                 const_cast<Event *>(recv_done), link_delay);
    recvThreadsNum++;
}

DrainState
DistIface::drain()
{
    DPRINTF(DistEthernet, "DistIFace::drain() called\n");
    // This can be called multiple times in the same drain cycle.
    if (this == primary)
        syncEvent->draining(true);
    return DrainState::Drained;
}

void
DistIface::drainResume()
{
    DPRINTF(DistEthernet, "DistIFace::drainResume() called\n");
    if (this == primary)
        syncEvent->draining(false);
    recvScheduler.resumeRecvTicks();
}

void
DistIface::serialize(CheckpointOut &cp) const
{
    // Drain the dist interface before the checkpoint is taken. We cannot call
    // this as part of the normal drain cycle because this dist sync has to be
    // called exactly once after the system is fully drained.
    sync->drainComplete();

    unsigned rank_orig = rank, dist_iface_id_orig = distIfaceId;

    SERIALIZE_SCALAR(rank_orig);
    SERIALIZE_SCALAR(dist_iface_id_orig);

    recvScheduler.serializeSection(cp, "recvScheduler");
    if (this == primary) {
        sync->serializeSection(cp, "Sync");
    }
}

void
DistIface::unserialize(CheckpointIn &cp)
{
    unsigned rank_orig, dist_iface_id_orig;
    UNSERIALIZE_SCALAR(rank_orig);
    UNSERIALIZE_SCALAR(dist_iface_id_orig);

    panic_if(rank != rank_orig, "Rank mismatch at resume (rank=%d, orig=%d)",
             rank, rank_orig);
    panic_if(distIfaceId != dist_iface_id_orig,
             "Dist iface ID mismatch "
             "at resume (distIfaceId=%d, orig=%d)",
             distIfaceId, dist_iface_id_orig);

    recvScheduler.unserializeSection(cp, "recvScheduler");
    if (this == primary) {
        sync->unserializeSection(cp, "Sync");
    }
}

void
DistIface::init(const Event *done_event, Tick link_delay)
{
    // Init hook for the underlaying message transport to setup/finalize
    // communication channels
    initTransport();

    // Spawn a new receiver thread that will process messages
    // coming in from peer gem5 processes.
    // The receive thread will also schedule a (receive) doneEvent
    // for each incoming data packet.
    spawnRecvThread(done_event, link_delay);

    // Adjust the periodic sync start and interval. Different DistIface
    // might have different requirements. The singleton sync object
    // will select the minimum values for both params.
    assert(sync != nullptr);
    sync->init(syncStart, syncRepeat);

    // Initialize the seed for random generator to avoid the same sequence
    // in all gem5 peer processes
    assert(primary != nullptr);
    if (this == primary)
        random_mt.init(5489 * (rank + 1) + 257);
}

void
DistIface::startup()
{
    DPRINTF(DistEthernet, "DistIface::startup() started\n");
    // Schedule synchronization unless we are not a switch in pseudo_op mode.
    if (this == primary && (!syncStartOnPseudoOp || isSwitch))
        syncEvent->start();
    DPRINTF(DistEthernet, "DistIface::startup() done\n");
}

bool
DistIface::readyToCkpt(Tick delay, Tick period)
{
    bool ret = true;
    DPRINTF(DistEthernet,
            "DistIface::readyToCkpt() called, delay:%lu "
            "period:%lu\n",
            delay, period);
    if (primary) {
        if (delay == 0) {
            inform("m5 checkpoint called with zero delay => triggering "
                   "collaborative "
                   "checkpoint\n");
            sync->requestCkpt(ReqType::collective);
        } else {
            inform("m5 checkpoint called with non-zero delay => triggering "
                   "immediate "
                   "checkpoint (at the next sync)\n");
            sync->requestCkpt(ReqType::immediate);
        }
        if (period != 0)
            inform("Non-zero period for m5_ckpt is ignored in "
                   "distributed gem5 runs\n");
        ret = false;
    }
    return ret;
}

void
DistIface::SyncNode::requestStopSync(ReqType req)
{
    std::lock_guard<std::mutex> sync_lock(lock);
    needStopSync = req;
}

void
DistIface::toggleSync(ThreadContext *tc)
{
    // Unforunate that we have to populate the system pointer member this way.
    primary->sys = tc->getSystemPtr();

    // The invariant for both syncing and "unsyncing" is that all threads will
    // stop executing intructions until the desired sync state has been reached
    // for all nodes.  This is the easiest way to prevent deadlock (in the case
    // of "unsyncing") and causality errors (in the case of syncing).
    if (primary->syncEvent->scheduled()) {
        inform("Request toggling syncronization off\n");
        primary->sync->requestStopSync(ReqType::collective);

        // At this point, we have no clue when everyone will reach the sync
        // stop point.  Suspend execution of all local thread contexts.
        // Dist-gem5 will reactivate all thread contexts when everyone has
        // reached the sync stop point.
        for (auto *tc : primary->sys->threads) {
            if (tc->status() == ThreadContext::Active)
                tc->quiesce();
        }
    } else {
        inform("Request toggling syncronization on\n");
        primary->syncEvent->start();

        // We need to suspend all CPUs until the sync point is reached by all
        // nodes to prevent causality errors.  We can also schedule CPU
        // activation here, since we know exactly when the next sync will
        // occur.
        for (auto *tc : primary->sys->threads) {
            if (tc->status() == ThreadContext::Active)
                tc->quiesceTick(primary->syncEvent->when() + 1);
        }
    }
}

bool
DistIface::readyToExit(Tick delay)
{
    bool ret = true;
    DPRINTF(DistEthernet, "DistIface::readyToExit() called, delay:%lu\n",
            delay);
    if (primary) {
        // To successfully coordinate an exit, all nodes must be synchronising
        if (!primary->syncEvent->scheduled())
            primary->syncEvent->start();

        if (delay == 0) {
            inform(
                "m5 exit called with zero delay => triggering collaborative "
                "exit\n");
            sync->requestExit(ReqType::collective);
        } else {
            inform(
                "m5 exit called with non-zero delay => triggering immediate "
                "exit (at the next sync)\n");
            sync->requestExit(ReqType::immediate);
        }
        ret = false;
    }
    return ret;
}

uint64_t
DistIface::rankParam()
{
    uint64_t val;
    if (primary) {
        val = primary->rank;
    } else {
        warn("Dist-rank parameter is queried in single gem5 simulation.");
        val = 0;
    }
    return val;
}

uint64_t
DistIface::sizeParam()
{
    uint64_t val;
    if (primary) {
        val = primary->size;
    } else {
        warn("Dist-size parameter is queried in single gem5 simulation.");
        val = 1;
    }
    return val;
}

} // namespace gem5
