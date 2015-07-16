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
 */

#include "dev/multi_iface.hh"

#include <queue>
#include <thread>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/MultiEthernet.hh"
#include "debug/MultiEthernetPkt.hh"
#include "dev/etherpkt.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"


MultiIface::Sync *MultiIface::sync = nullptr;
MultiIface::SyncEvent *MultiIface::syncEvent = nullptr;
unsigned MultiIface::recvThreadsNum = 0;
MultiIface *MultiIface::master = nullptr;

bool
MultiIface::Sync::run(SyncTrigger t, Tick sync_tick)
{
    std::unique_lock<std::mutex> sync_lock(lock);

    trigger = t;
    if (trigger != SyncTrigger::periodic) {
        DPRINTF(MultiEthernet,"MultiIface::Sync::run() trigger:%d\n",
                (unsigned)trigger);
    }

    switch (state) {
      case SyncState::asyncCkpt:
        switch (trigger) {
          case SyncTrigger::ckpt:
            assert(MultiIface::syncEvent->interrupted == false);
            state = SyncState::busy;
            break;
          case SyncTrigger::periodic:
            if (waitNum == 0) {
                // So all recv threads got an async checkpoint request already
                // and a simExit is scheduled at the end of the current tick
                // (i.e. it is a periodic sync scheduled at the same tick as
                // the simExit).
                state = SyncState::idle;
                DPRINTF(MultiEthernet,"MultiIface::Sync::run() interrupted "
                "due to async ckpt scheduled\n");
                return false;
            } else {
                // we still need to wait for some receiver thread to get the
                // aysnc ckpt request. We are going to proceed as 'interrupted'
                // periodic sync.
                state = SyncState::interrupted;
                DPRINTF(MultiEthernet,"MultiIface::Sync::run() interrupted "
                "due to ckpt request is coming in\n");
            }
            break;
          case SyncTrigger::atomic:
            assert(trigger != SyncTrigger::atomic);
        }
        break;
      case SyncState::idle:
        state = SyncState::busy;
        break;
        // Only one sync can be active at any time
      case SyncState::interrupted:
      case SyncState::busy:
        assert(state != SyncState::interrupted);
        assert(state != SyncState::busy);
        break;
    }
    // Kick-off the sync unless we are in the middle of an interrupted
    // periodic sync
    if (state != SyncState::interrupted) {
        assert(waitNum == 0);
        waitNum = MultiIface::recvThreadsNum;
        // initiate the global synchronisation
        assert(MultiIface::master != nullptr);
        MultiIface::master->syncRaw(triggerToMsg[(unsigned)trigger], sync_tick);
    }
    // now wait until all receiver threads complete the synchronisation
    auto lf = [this]{ return waitNum == 0; };
    cv.wait(sync_lock, lf);

    // we are done
    assert(state == SyncState::busy || state == SyncState::interrupted);
    bool ret = (state != SyncState::interrupted);
    state = SyncState::idle;
    return ret;
}

void
MultiIface::Sync::progress(MsgType msg)
{
    std::unique_lock<std::mutex> sync_lock(lock);

    switch (msg) {
      case MsgType::cmdAtomicSyncAck:
        assert(state == SyncState::busy && trigger == SyncTrigger::atomic);
        break;
      case MsgType::cmdPeriodicSyncAck:
        assert(state == SyncState::busy && trigger == SyncTrigger::periodic);
        break;
      case MsgType::cmdCkptSyncAck:
        assert(state == SyncState::busy && trigger == SyncTrigger::ckpt);
        break;
      case MsgType::cmdCkptSyncReq:
        switch (state) {
          case SyncState::busy:
            if (trigger == SyncTrigger::ckpt) {
                // We are already in a checkpoint sync but got another ckpt
                // sync request. This may happen if two (or more) peer gem5
                // processes try to start a ckpt nearly at the same time.
                // Incrementing waitNum here (before decrementing it below)
                // effectively results in ignoring this new ckpt sync request.
                waitNum++;
                break;
            }
            assert (waitNum == recvThreadsNum);
            state = SyncState::interrupted;
            // we need to fall over here to handle "recvThreadsNum == 1" case
          case SyncState::interrupted:
            assert(trigger == SyncTrigger::periodic);
            assert(waitNum >= 1);
            if (waitNum == 1) {
                exitSimLoop("checkpoint");
            }
            break;
          case SyncState::idle:
            // There is no on-going sync so we got an async ckpt request. If we
            // are the only receiver thread then we need to schedule the
            // checkpoint. Otherwise, only change the state to 'asyncCkpt' and
            // let the last receiver thread to schedule the checkpoint at the
            // 'asyncCkpt' case.
            // Note that a periodic or resume sync may start later and that can
            // trigger a state change to 'interrupted' (so the checkpoint may
            // get scheduled at 'interrupted' case finally).
            assert(waitNum == 0);
            state = SyncState::asyncCkpt;
            waitNum = MultiIface::recvThreadsNum;
            // we need to fall over here to handle "recvThreadsNum == 1" case
          case SyncState::asyncCkpt:
            assert(waitNum >= 1);
            if (waitNum == 1)
                exitSimLoop("checkpoint");
            break;
          default:
            panic("Unexpected state for checkpoint request message");
            break;
        }
        break;
      default:
        panic("Unknown msg type");
        break;
    }
    waitNum--;
    assert(state != SyncState::idle);
    // Notify the simultaion thread if there is an on-going sync.
    if (state != SyncState::asyncCkpt) {
        sync_lock.unlock();
        cv.notify_one();
    }
}

void MultiIface::SyncEvent::start(Tick start, Tick interval)
{
    assert(!scheduled());
    if (interval == 0)
        panic("Multi synchronisation period must be greater than zero");
    repeat = interval;
    schedule(start);
}

void
MultiIface::SyncEvent::adjust(Tick start_tick, Tick repeat_tick)
{
    // The new multi interface may require earlier start of the
    // synchronisation.
    assert(scheduled() == true);
    if (start_tick < when())
        reschedule(start_tick);
    // The new multi interface may require more frequent synchronisation.
    if (repeat == 0)
        panic("Multi synchronisation period must be greater than zero");
    if (repeat < repeat_tick)
        repeat = repeat_tick;
}

void
MultiIface::SyncEvent::process()
{
    /*
     * Note that this is a global event so this process method will be called
     * by only exactly one thread.
     */
    // if we are draining the system then we must not start a periodic sync (as
    // it is not sure that all peer gem5 will reach this tick before taking
    // the checkpoint).
    if (isDraining == true) {
        assert(interrupted == false);
        interrupted = true;
        DPRINTF(MultiEthernet,"MultiIface::SyncEvent::process() interrupted "
                "due to draining\n");
        return;
    }
    if (interrupted == false)
        scheduledAt = curTick();
    /*
     * We hold the eventq lock at this point but the receiver thread may
     * need the lock to schedule new recv events while waiting for the
     * multi sync to complete.
     * Note that the other simulation threads also release their eventq
     * locks while waiting for us due to the global event semantics.
     */
    curEventQueue()->unlock();
    // we do a global sync here
    interrupted = !MultiIface::sync->run(SyncTrigger::periodic, scheduledAt);
    // Global sync completed or got interrupted.
    // we are expected to exit with the eventq lock held
    curEventQueue()->lock();
    // schedule the next global sync event if this one completed. Otherwise
    // (i.e. this one was interrupted by a checkpoint request), we will 
    // reschedule this one after the draining is complete.
    if (!interrupted)
        schedule(scheduledAt + repeat);
}

void MultiIface::SyncEvent::resume()
{
    Tick sync_tick;
    assert(!scheduled());
    if (interrupted) {
        assert(curTick() >= scheduledAt);
        // We have to complete the interrupted periodic sync asap.
        // Note that this sync might be interrupted now again with a checkpoint
        // request from a peer gem5...
        sync_tick = curTick();
        schedule(sync_tick);
    } else {
        // So we completed the last periodic sync, let's find  out the tick for
        // next one
        assert(curTick() > scheduledAt);
        sync_tick = scheduledAt + repeat;
        if (sync_tick < curTick())
            panic("Cannot resume periodic synchronisation");
        schedule(sync_tick);
    }
    DPRINTF(MultiEthernet,
            "MultiIface::SyncEvent periodic sync resumed at %lld "
            "(curTick:%lld)\n", sync_tick, curTick());
}

void MultiIface::SyncEvent::serialize(const std::string &base,
                                      CheckpointOut &cp) const
{
    // Save the periodic multi sync schedule information
    paramOut(cp, base + ".periodicSyncRepeat", repeat);
    paramOut(cp, base + ".periodicSyncInterrupted", interrupted);
    paramOut(cp, base + ".periodicSyncAt", scheduledAt);
}

void MultiIface::SyncEvent::unserialize(const std::string &base,
                                        CheckpointIn &cp)
{
    paramIn(cp, base + ".periodicSyncRepeat", repeat);
    paramIn(cp, base + ".periodicSyncInterrupted", interrupted);
    paramIn(cp, base + ".periodicSyncAt", scheduledAt);
}

MultiIface::MultiIface(unsigned multi_rank,
                       Tick sync_start,
                       Tick sync_repeat,
                       EventManager *em) :
    syncStart(sync_start), syncRepeat(sync_repeat),
    recvThread(nullptr), eventManager(em), recvDone(nullptr),
    scheduledRecvPacket(nullptr), linkDelay(0), rank(multi_rank)
{
    DPRINTF(MultiEthernet, "MultiIface() ctor rank:%d\n",multi_rank);
    if (master == nullptr) {
        assert(sync == nullptr);
        assert(syncEvent == nullptr);
        sync = new Sync();
        syncEvent = new SyncEvent();
        master = this;
    }
}

MultiIface::~MultiIface()
{
    assert(recvThread);
    delete recvThread;
    if (this == master) {
        assert(syncEvent);
        delete syncEvent;
        assert(sync);
        delete sync;
    }
}

void
MultiIface::packetOut(EthPacketPtr pkt, Tick send_delay)
{
    MultiHeaderPkt::Header header_pkt;
    unsigned address_length = MultiHeaderPkt::maxAddressLength();

    // Prepare a multi header packet for the Ethernet packet we want to
    // send out.
    header_pkt.msgType = MsgType::dataDescriptor;
    header_pkt.sendTick  = curTick();
    header_pkt.sendDelay = send_delay;

    // Store also the source and destination addresses.
    pkt->packAddress(header_pkt.srcAddress, header_pkt.dstAddress,
                     address_length);

    header_pkt.dataPacketLength = pkt->size();

    // Send out the multi hedare packet followed by the Ethernet packet.
    sendRaw(&header_pkt, sizeof(header_pkt), header_pkt.dstAddress);
    sendRaw(pkt->data, pkt->size(), header_pkt.dstAddress);
    DPRINTF(MultiEthernetPkt,
            "MultiIface::sendDataPacket() done size:%d send_delay:%llu "
            "src:0x%02x%02x%02x%02x%02x%02x "
            "dst:0x%02x%02x%02x%02x%02x%02x\n",
            pkt->size(), send_delay,
            header_pkt.srcAddress[0], header_pkt.srcAddress[1],
            header_pkt.srcAddress[2], header_pkt.srcAddress[3],
            header_pkt.srcAddress[4], header_pkt.srcAddress[5],
            header_pkt.dstAddress[0], header_pkt.dstAddress[1],
            header_pkt.dstAddress[2], header_pkt.dstAddress[3],
            header_pkt.dstAddress[4], header_pkt.dstAddress[5]);
}

bool
MultiIface::recvHeader(MultiHeaderPkt::Header &header_pkt)
{
    // Blocking receive of an incoming multi header packet.
    return recvRaw((void *)&header_pkt, sizeof(header_pkt));
}

void
MultiIface::recvData(const MultiHeaderPkt::Header &header_pkt)
{
    // We are here beacuse a header packet has been received implying
    // that an Ethernet (data) packet is coming in next.
    assert(header_pkt.msgType == MsgType::dataDescriptor);
    // Allocate storage for the incoming Ethernet packet.
    EthPacketPtr new_packet(new EthPacketData(header_pkt.dataPacketLength));
    // Now execute the blocking receive and store the incoming data directly
    // in the new EthPacketData object.
    if (! recvRaw((void *)(new_packet->data), header_pkt.dataPacketLength))
        panic("Missing data packet");

    new_packet->length = header_pkt.dataPacketLength;
    // Grab the event queue lock to schedule a new receive event for the
    // data packet.
    curEventQueue()->lock();
    // Compute the receive tick. It includes the send delay and the
    // simulated link delay.
    Tick recv_tick = header_pkt.sendTick + header_pkt.sendDelay + linkDelay;
    DPRINTF(MultiEthernetPkt, "MultiIface::recvThread() packet receive, "
            "send_tick:%llu send_delay:%llu link_delay:%llu recv_tick:%llu\n",
            header_pkt.sendTick, header_pkt.sendDelay, linkDelay, recv_tick);

    if (recv_tick <= curTick()) {
        panic("Simulators out of sync - missed packet receive by %llu ticks",
              curTick() - recv_tick);
    }
    // Now we are about to schedule a recvDone event for the new data packet.
    // We use the same recvDone object for all incoming data packets. If
    // that is already scheduled - i.e. a receive event for a previous
    // data packet is already pending - then we have to check whether the
    // receive tick for the new packet is earlier than that of the currently
    // pending event. Packets may arrive out-of-order with respect to
    // simulated receive time. If that is the case, we need to re-schedule the
    // recvDone event for the new packet. Otherwise, we save the packet
    // pointer and the recv tick for the new packet in the recvQueue. See
    // the implementation of the packetIn() method for comments on how this
    // information is retrieved from the recvQueue by the simulation thread.
    if (!recvDone->scheduled()) {
        assert(recvQueue.size() == 0);
        assert(scheduledRecvPacket == nullptr);
        scheduledRecvPacket = new_packet;
        eventManager->schedule(recvDone, recv_tick);
    } else if (recvDone->when() > recv_tick) {
        recvQueue.emplace(scheduledRecvPacket, recvDone->when());
        eventManager->reschedule(recvDone, recv_tick);
        scheduledRecvPacket = new_packet;
    } else {
        recvQueue.emplace(new_packet, recv_tick);
    }
    curEventQueue()->unlock();
}

void
MultiIface::recvThreadFunc()
{
    EthPacketPtr new_packet;
    MultiHeaderPkt::Header header;

    // The new receiver thread shares the event queue with the simulation
    // thread (associated with the simulated Ethernet link).
    curEventQueue(eventManager->eventQueue());
    // Main loop to wait for and process any incoming message.
    for (;;) {
        // recvHeader() blocks until the next multi header packet comes in.
        if (!recvHeader(header)) {
            // We lost connection to the peer gem5 processes most likely
            // because one of them called m5 exit. So we stop here.
            exit_message("info", 0, "Message server closed connection, "
                         "simulation is exiting");
        }
        // We got a valid multi header packet, let's process it
        if (header.msgType == MsgType::dataDescriptor) {
            recvData(header);
        } else {
            // everything else must be synchronisation related command
            sync->progress(header.msgType);
        }
    }
}

EthPacketPtr
MultiIface::packetIn()
{
    // We are called within the process() method of the recvDone event. We
    // return the packet that triggered the current receive event.
    // If there is further packets in the recvQueue, we also have to schedule
    // the recvEvent for the next packet with the smallest receive tick.
    // The priority queue container ensures that smallest receive tick is
    // always on the top of the queue.
    assert(scheduledRecvPacket != nullptr);
    EthPacketPtr next_packet = scheduledRecvPacket;

    if (! recvQueue.empty()) {
        eventManager->schedule(recvDone, recvQueue.top().second);
        scheduledRecvPacket = recvQueue.top().first;
        recvQueue.pop();
    } else {
        scheduledRecvPacket = nullptr;
    }

    return next_packet;
}

void
MultiIface::spawnRecvThread(Event *recv_done, Tick link_delay)
{
    assert(recvThread == nullptr);
    // all receive thread must be spawned before simulation starts
    assert(eventManager->eventQueue()->getCurTick() == 0);

    recvDone = recv_done;
    linkDelay = link_delay;

    recvThread = new std::thread(&MultiIface::recvThreadFunc, this);

    recvThreadsNum++;
}

DrainState
MultiIface::drain()
{
    DPRINTF(MultiEthernet,"MultiIFace::drain() called\n");

    // This can be called multiple times in the same drain cycle.
    if (master == this) {
        syncEvent->isDraining = true;
    }

    return DrainState::Drained;
}

void MultiIface::drainDone() {
    if (master == this) {
        assert(syncEvent->isDraining == true);
        syncEvent->isDraining = false;
        // We need to resume the interrupted periodic sync here now that the
        // draining is done. If the last periodic sync completed before the
        // checkpoint then the next one is already scheduled.
        if (syncEvent->interrupted)
            syncEvent->resume();
    }
}

void MultiIface::serialize(const std::string &base, CheckpointOut &cp) const
{
    // Drain the multi interface before the checkpoint is taken. We cannot call
    // this as part of the normal drain cycle because this multi sync has to be
    // called exactly once after the system is fully drained.
    // Note that every peer will take a checkpoint but they may take it at
    // different ticks.
    // This sync request may interrupt an on-going periodic sync in some peers.
    sync->run(SyncTrigger::ckpt, curTick());

    // Save the periodic multi sync status
    syncEvent->serialize(base, cp);

    unsigned n_rx_packets = recvQueue.size();
    if (scheduledRecvPacket != nullptr)
        n_rx_packets++;

    paramOut(cp, base + ".nRxPackets", n_rx_packets);

    if (n_rx_packets > 0) {
        assert(recvDone->scheduled());
        scheduledRecvPacket->serialize(base + ".rxPacket[0]", cp);
    }

    for (unsigned i=1; i < n_rx_packets; i++)  {
        const RecvInfo recv_info = recvQueue.impl().at(i-1);
        recv_info.first->serialize(base + csprintf(".rxPacket[%d]", i), cp);
        Tick rx_tick = recv_info.second;
        paramOut(cp, base + csprintf(".rxTick[%d]", i), rx_tick);
    }
}

void MultiIface::unserialize(const std::string &base, CheckpointIn &cp)
{
    assert(recvQueue.size() == 0);
    assert(scheduledRecvPacket == nullptr);
    assert(recvDone->scheduled() == false);

    // restore periodic sync info
    syncEvent->unserialize(base, cp);

    unsigned n_rx_packets;
    paramIn(cp, base + ".nRxPackets", n_rx_packets);

    if (n_rx_packets > 0) {
        scheduledRecvPacket = std::make_shared<EthPacketData>(16384);
        scheduledRecvPacket->unserialize(base + ".rxPacket[0]", cp);
        // Note: receive event will be scheduled when the link is unserialized
    }

    for (unsigned i=1; i < n_rx_packets; i++) {
        EthPacketPtr rx_packet = std::make_shared<EthPacketData>(16384);
        rx_packet->unserialize(base + csprintf(".rxPacket[%d]", i), cp);
        Tick rx_tick = 0;
        paramIn(cp, base + csprintf(".rxTick[%d]", i), rx_tick);
        assert(rx_tick > 0);
        recvQueue.emplace(rx_packet,rx_tick);
    }
}

void MultiIface::initRandom()
{
    // Initialize the seed for random generator to avoid the same sequence
    // in all gem5 peer processes
    assert(master != nullptr);
    if (this == master)
        random_mt.init(5489 * (rank+1) + 257);
}

void MultiIface::startPeriodicSync()
{
    DPRINTF(MultiEthernet, "MultiIface:::initPeriodicSync started\n");
    // Do a global sync here to ensure that peer gem5 processes are around
    // (actually this may not be needed...)
    sync->run(SyncTrigger::atomic, curTick());

    // Start the periodic sync if it is a fresh simulation from scratch
    if (curTick() == 0) {
        if (this == master) {
        syncEvent->start(syncStart, syncRepeat);
        inform("Multi synchronisation activated: start at %lld, "
               "repeat at every %lld ticks.\n",
               syncStart, syncRepeat);
        } else {
            // In case another multiIface object requires different schedule
            // for periodic sync than the master does.
            syncEvent->adjust(syncStart, syncRepeat);
        }
    } else {
        // Schedule the next periodic sync if resuming from a checkpoint
        if (this == master)
            syncEvent->resume();
    }
    DPRINTF(MultiEthernet, "MultiIface::initPeriodicSync done\n");
}
