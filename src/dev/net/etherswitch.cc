/*
 * Copyright (c) 2014 The Regents of The University of Michigan
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
 *
 * Authors: Anthony Gutierrez
 *          Mohammad Alian
 */

/* @file
 * Device model for an ethernet switch
 */

#include "dev/net/etherswitch.hh"

#include "base/random.hh"
#include "debug/EthernetAll.hh"

using namespace std;

EtherSwitch::EtherSwitch(const Params *p)
    : EtherObject(p), ttl(p->time_to_live)
{
    for (int i = 0; i < p->port_interface_connection_count; ++i) {
        std::string interfaceName = csprintf("%s.interface%d", name(), i);
        Interface *interface = new Interface(interfaceName, this,
                                        p->output_buffer_size, p->delay,
                                        p->delay_var, p->fabric_speed, i);
        interfaces.push_back(interface);
    }
}

EtherSwitch::~EtherSwitch()
{
    for (auto it : interfaces)
        delete it;

    interfaces.clear();
}

EtherInt*
EtherSwitch::getEthPort(const std::string &if_name, int idx)
{
    if (idx < 0 || idx >= interfaces.size())
        return nullptr;

    Interface *interface = interfaces.at(idx);
    panic_if(interface->getPeer(), "interface already connected\n");

    return interface;
}

bool
EtherSwitch::Interface::PortFifo::push(EthPacketPtr ptr, unsigned senderId)
{
    assert(ptr->length);

    _size += ptr->length;
    fifo.emplace_hint(fifo.end(), ptr, curTick(), senderId);

    // Drop the extra pushed packets from end of the fifo
    while (avail() < 0) {
        DPRINTF(Ethernet, "Fifo is full. Drop packet: len=%d\n",
                std::prev(fifo.end())->packet->length);

        _size -= std::prev(fifo.end())->packet->length;
        fifo.erase(std::prev(fifo.end()));
    }

    if (empty()) {
        warn("EtherSwitch: Packet length (%d) exceeds the maximum storage "
             "capacity of port fifo (%d)", ptr->length, _maxsize);
    }

    // Return true if the newly pushed packet gets inserted
    // at the head of the queue, otherwise return false
    // We need this information to deschedule the event that has been
    // scheduled for the old head of queue packet and schedule a new one
    if (!empty() && fifo.begin()->packet == ptr) {
        return true;
    }
    return false;
}

void
EtherSwitch::Interface::PortFifo::pop()
{
    if (empty())
        return;

    assert(_size >= fifo.begin()->packet->length);
    // Erase the packet at the head of the queue
    _size -= fifo.begin()->packet->length;
    fifo.erase(fifo.begin());
}

void
EtherSwitch::Interface::PortFifo::clear()
{
    fifo.clear();
    _size = 0;
}

EtherSwitch::Interface::Interface(const std::string &name,
                                  EtherSwitch *etherSwitch,
                                  uint64_t outputBufferSize, Tick delay,
                                  Tick delay_var, double rate, unsigned id)
    : EtherInt(name), ticksPerByte(rate), switchDelay(delay),
      delayVar(delay_var), interfaceId(id), parent(etherSwitch),
      outputFifo(name + ".outputFifo", outputBufferSize), txEvent(this)
{
}

bool
EtherSwitch::Interface::recvPacket(EthPacketPtr packet)
{
    Net::EthAddr destMacAddr(packet->data);
    Net::EthAddr srcMacAddr(&packet->data[6]);

    learnSenderAddr(srcMacAddr, this);
    Interface *receiver = lookupDestPort(destMacAddr);

    if (!receiver || destMacAddr.multicast() || destMacAddr.broadcast()) {
        for (auto it : parent->interfaces)
            if (it != this)
                it->enqueue(packet, interfaceId);
    } else {
        DPRINTF(Ethernet, "sending packet from MAC %x on port "
                "%s to MAC %x on port %s\n", uint64_t(srcMacAddr),
                this->name(), uint64_t(destMacAddr), receiver->name());

        receiver->enqueue(packet, interfaceId);
    }
    // At the output port, we either have buffer space (no drop) or
    // don't (drop packet); in both cases packet is received on
    // the interface successfully and there is no notion of busy
    // interface here (as we don't have inputFifo)
    return true;
}

void
EtherSwitch::Interface::enqueue(EthPacketPtr packet, unsigned senderId)
{
    // assuming per-interface transmission events,
    // if the newly push packet gets inserted at the head of the queue
    // (either there was nothing in the queue or the priority of the new
    // packet was higher than the packets already in the fifo)
    // then we need to schedule an event at
    // "curTick" + "switchingDelay of the packet at the head of the fifo"
    // to send this packet out the external link
    // otherwise, there is already a txEvent scheduled
    if (outputFifo.push(packet, senderId)) {
        parent->reschedule(txEvent, curTick() + switchingDelay(), true);
    }
}

void
EtherSwitch::Interface::transmit()
{
    // there should be something in the output queue
    assert(!outputFifo.empty());

    if (!sendPacket(outputFifo.front())) {
        DPRINTF(Ethernet, "output port busy...retry later\n");
        if (!txEvent.scheduled())
            parent->schedule(txEvent, curTick() + retryTime);
    } else {
        DPRINTF(Ethernet, "packet sent: len=%d\n", outputFifo.front()->length);
        outputFifo.pop();
        // schedule an event to send the pkt at
        // the head of queue, if there is any
        if (!outputFifo.empty()) {
            parent->schedule(txEvent, curTick() + switchingDelay());
        }
    }
}

Tick
EtherSwitch::Interface::switchingDelay()
{
    Tick delay = (Tick)ceil(((double)outputFifo.front()->simLength
                                     * ticksPerByte) + 1.0);
    if (delayVar != 0)
                delay += random_mt.random<Tick>(0, delayVar);
    delay += switchDelay;
    return delay;
}

EtherSwitch::Interface*
EtherSwitch::Interface::lookupDestPort(Net::EthAddr destMacAddr)
{
    auto it = parent->forwardingTable.find(uint64_t(destMacAddr));

    if (it == parent->forwardingTable.end()) {
        DPRINTF(Ethernet, "no entry in forwaring table for MAC: "
                "%x\n", uint64_t(destMacAddr));
        return nullptr;
    }

    // check if this entry is valid based on TTL and lastUseTime
    if ((curTick() - it->second.lastUseTime) > parent->ttl) {
        // TTL for this mapping has been expired, so this item is not
        // valide anymore, let's remove it from the map
        parent->forwardingTable.erase(it);
        return nullptr;
    }

    DPRINTF(Ethernet, "found entry for MAC address %x on port %s\n",
            uint64_t(destMacAddr), it->second.interface->name());
    return it->second.interface;
}

void
EtherSwitch::Interface::learnSenderAddr(Net::EthAddr srcMacAddr,
                                          Interface *sender)
{
    // learn the port for the sending MAC address
    auto it = parent->forwardingTable.find(uint64_t(srcMacAddr));

    // if the port for sender's MAC address is not cached,
    // cache it now, otherwise just update lastUseTime time
    if (it == parent->forwardingTable.end()) {
        DPRINTF(Ethernet, "adding forwarding table entry for MAC "
                " address %x on port %s\n", uint64_t(srcMacAddr),
                sender->name());
        EtherSwitch::SwitchTableEntry forwardingTableEntry;
        forwardingTableEntry.interface = sender;
        forwardingTableEntry.lastUseTime = curTick();
        parent->forwardingTable.insert(std::make_pair(uint64_t(srcMacAddr),
            forwardingTableEntry));
    } else {
        it->second.lastUseTime = curTick();
    }
}

void
EtherSwitch::serialize(CheckpointOut &cp) const
{
    for (auto it : interfaces)
        it->serializeSection(cp, it->name());

}

void
EtherSwitch::unserialize(CheckpointIn &cp)
{
    for (auto it : interfaces)
        it->unserializeSection(cp, it->name());

}

void
EtherSwitch::Interface::serialize(CheckpointOut &cp) const
{
    bool event_scheduled = txEvent.scheduled();
    SERIALIZE_SCALAR(event_scheduled);

    if (event_scheduled) {
        Tick event_time = txEvent.when();
        SERIALIZE_SCALAR(event_time);
    }
    outputFifo.serializeSection(cp, "outputFifo");
}

void
EtherSwitch::Interface::unserialize(CheckpointIn &cp)
{
    bool event_scheduled;
    UNSERIALIZE_SCALAR(event_scheduled);

    if (event_scheduled) {
        Tick event_time;
        UNSERIALIZE_SCALAR(event_time);
        parent->schedule(txEvent, event_time);
    }
    outputFifo.unserializeSection(cp, "outputFifo");
}

void
EtherSwitch::Interface::PortFifoEntry::serialize(CheckpointOut &cp) const
{
    packet->serialize("packet", cp);
    SERIALIZE_SCALAR(recvTick);
    SERIALIZE_SCALAR(srcId);
}

void
EtherSwitch::Interface::PortFifoEntry::unserialize(CheckpointIn &cp)
{
    packet = make_shared<EthPacketData>(16384);
    packet->unserialize("packet", cp);
    UNSERIALIZE_SCALAR(recvTick);
    UNSERIALIZE_SCALAR(srcId);
}

void
EtherSwitch::Interface::PortFifo::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_size);
    int fifosize = fifo.size();

    SERIALIZE_SCALAR(fifosize);

    int i = 0;
    for (const auto &entry : fifo)
        entry.serializeSection(cp, csprintf("entry%d", i++));
}

void
EtherSwitch::Interface::PortFifo::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_size);
    int fifosize;

    UNSERIALIZE_SCALAR(fifosize);
    fifo.clear();

    for (int i = 0; i < fifosize; ++i) {
        PortFifoEntry entry(nullptr, 0, 0);

        entry.unserializeSection(cp, csprintf("entry%d", i));

        fifo.insert(entry);

    }
}

EtherSwitch *
EtherSwitchParams::create()
{
    return new EtherSwitch(this);
}
