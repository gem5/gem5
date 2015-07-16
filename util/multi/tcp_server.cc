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
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabor Dozsa
 */


/* @file
 *  Message server implementation using TCP stream sockets for parallel gem5
 * runs.
 */
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>

#include "tcp_server.hh"

using namespace std;

// Some basic macros for information and error reporting.
#define PRINTF(...) fprintf(stderr, __VA_ARGS__)

#ifdef DEBUG
static bool debugSetup = true;
static bool debugPeriodic = false;
static bool debugSync = true;
static bool debugPkt = false;
#define DPRINTF(v,...) if (v) PRINTF(__VA_ARGS__)
#else
#define DPRINTF(v,...)
#endif

#define inform(...) do { PRINTF("info: ");      \
        PRINTF(__VA_ARGS__); } while(0)

#define panic(...)  do { PRINTF("panic: ");     \
        PRINTF(__VA_ARGS__);                    \
        PRINTF("\n[%s:%s], line %d\n",          \
               __FUNCTION__, __FILE__, __LINE__); \
        exit(-1); } while(0)

TCPServer *TCPServer::instance = nullptr;

TCPServer::Channel::Channel() : fd(-1), isAlive(false), state(SyncState::idle)
{
    MultiHeaderPkt::clearAddress(address);
}

unsigned
TCPServer::Channel::recvRaw(void *buf, unsigned size) const
{
    ssize_t n;
    // This is a blocking receive.
    n = recv(fd, buf, size, MSG_WAITALL);

    if (n < 0)
        panic("read() failed:%s", strerror(errno));
    else if (n > 0 && n < size)
        // the recv() call should wait for the full message
        panic("read() failed");

    return n;
}

void
TCPServer::Channel::sendRaw(const void *buf, unsigned size) const
{
    ssize_t n;
    n = send(fd, buf, size, MSG_NOSIGNAL);
    if (n < 0)
        panic("write() failed:%s", strerror(errno));
    else if (n != size)
        panic("write() failed");
}

void TCPServer::Channel::updateAddress(const AddressType &new_address)
{
    // check if the known address has changed (e.g. the client reconfigured
    // its Ethernet NIC)
    if (MultiHeaderPkt::isAddressEqual(address, new_address))
        return;

    // So we have to update the address. Note that we always
    // store the same address as key in the map but the ordering
    // may change so we need to erase and re-insert it again.
    auto info = TCPServer::instance->addressMap.find(&address);
    if (info != TCPServer::instance->addressMap.end()) {
        TCPServer::instance->addressMap.erase(info);
    }

    MultiHeaderPkt::copyAddress(address, new_address);
    TCPServer::instance->addressMap[&address] = this;
}

void
TCPServer::Channel::headerPktIn()
{
    ssize_t n;
    Header hdr_pkt;

    n = recvRaw(&hdr_pkt, sizeof(hdr_pkt));

    if (n == 0) {
        // EOF - nothing to do here, we will handle this as a POLLRDHUP event
        // in the main loop.
        return;
    }

    if (hdr_pkt.msgType == MsgType::dataDescriptor) {
        updateAddress(hdr_pkt.srcAddress);
        TCPServer::instance->xferData(hdr_pkt, *this);
    } else {
        processCmd(hdr_pkt.msgType, hdr_pkt.sendTick);
    }
}

void TCPServer::Channel::processCmd(MsgType cmd, Tick send_tick)
{
    switch (cmd) {
      case MsgType::cmdAtomicSyncReq:
        DPRINTF(debugSync,"Atomic sync request (rank:%d)\n",rank);
        assert(state == SyncState::idle);
        state = SyncState::atomic;
        TCPServer::instance->syncTryComplete(SyncState::atomic,
                                             MsgType::cmdAtomicSyncAck);
        break;
      case MsgType::cmdPeriodicSyncReq:
        DPRINTF(debugPeriodic,"PERIODIC sync request (at %ld)\n",send_tick);
        // sanity check
        if (TCPServer::instance->periodicSyncTick() == 0) {
            TCPServer::instance->periodicSyncTick(send_tick);
        } else if ( TCPServer::instance->periodicSyncTick() != send_tick) {
            panic("Out of order periodic sync request - rank:%d "
                  "(send_tick:%ld ongoing:%ld)", rank, send_tick,
                  TCPServer::instance->periodicSyncTick());
        }
        switch (state) {
          case SyncState::idle:
            state = SyncState::periodic;
            TCPServer::instance->syncTryComplete(SyncState::periodic,
                                                 MsgType::cmdPeriodicSyncAck);
            break;
          case SyncState::asyncCkpt:
            // An async ckpt request has already been sent to this client and
            // that will interrupt this periodic sync. We can simply drop this
            // message.
            break;
          default:
            panic("Unexpected state for periodic sync request (rank:%d)",
                rank);
            break;
        }
        break;
      case MsgType::cmdCkptSyncReq:
        DPRINTF(debugSync, "CKPT sync request (rank:%d)\n",rank);
        switch (state) {
          case SyncState::idle:
            TCPServer::instance->ckptPropagate(*this);
            // we fall through here to complete #clients==1 case
          case SyncState::asyncCkpt:
            state = SyncState::ckpt;
            TCPServer::instance->syncTryComplete(SyncState::ckpt,
                                                 MsgType::cmdCkptSyncAck);
            break;
          default:
            panic("Unexpected state for ckpt sync request (rank:%d)", rank);
            break;
        }
        break;
      default:
        panic("Unexpected header packet (rank:%d)",rank);
        break;
    }
}

TCPServer::TCPServer(unsigned clients_num,
                     unsigned listen_port,
                     int timeout_in_sec)
{
    assert(instance == nullptr);
    construct(clients_num, listen_port, timeout_in_sec);
    instance = this;
}

TCPServer::~TCPServer()
{
    for (auto &c : clientsPollFd)
        close(c.fd);
}

void
TCPServer::construct(unsigned clients_num, unsigned port, int timeout_in_sec)
{
    int listen_sock, new_sock, ret;
    unsigned client_len;
    struct sockaddr_in server_addr, client_addr;
    struct pollfd new_pollfd;
    Channel new_channel;

    DPRINTF(debugSetup, "Start listening on port %u ...\n", port);

    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0)
        panic("socket() failed:%s", strerror(errno));

    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);
    if (bind(listen_sock, (struct sockaddr *) &server_addr,
             sizeof(server_addr)) < 0)
        panic("bind() failed:%s", strerror(errno));
    listen(listen_sock, 10);

    clientsPollFd.reserve(clients_num);
    clientsChannel.reserve(clients_num);

    new_pollfd.events = POLLIN | POLLRDHUP;
    new_pollfd.revents = 0;
    while (clientsPollFd.size() < clients_num) {
        new_pollfd.fd = listen_sock;
        ret = poll(&new_pollfd, 1, timeout_in_sec*1000);
        if (ret == 0)
            panic("Timeout while waiting for clients to connect");
        assert(ret == 1 && new_pollfd.revents == POLLIN);
        client_len = sizeof(client_addr);
        new_sock = accept(listen_sock,
                          (struct sockaddr *) &client_addr,
                          &client_len);
        if (new_sock < 0)
            panic("accept() failed:%s", strerror(errno));
        new_pollfd.fd = new_sock;
        new_pollfd.revents = 0;
        clientsPollFd.push_back(new_pollfd);
        new_channel.fd = new_sock;
        new_channel.isAlive = true;
        new_channel.recvRaw(&new_channel.rank, sizeof(new_channel.rank));
        clientsChannel.push_back(new_channel);

        DPRINTF(debugSetup, "New client connection addr:%u port:%hu rank:%d\n",
                client_addr.sin_addr.s_addr, client_addr.sin_port,
                new_channel.rank);
    }
    ret = close(listen_sock);
    assert(ret == 0);

    DPRINTF(debugSetup, "Setup complete\n");
}

void
TCPServer::run()
{
    int nfd;
    unsigned num_active_clients = clientsPollFd.size();

    DPRINTF(debugSetup, "Entering run() loop\n");
    while (num_active_clients ==  clientsPollFd.size()) {
        nfd = poll(&clientsPollFd[0], clientsPollFd.size(), -1);
        if (nfd == -1)
            panic("poll() failed:%s", strerror(errno));

        for (unsigned i = 0, n = 0;
             i < clientsPollFd.size() && (signed)n < nfd;
             i++) {
            struct pollfd &pfd = clientsPollFd[i];
            if (pfd.revents) {
                if (pfd.revents & POLLERR)
                    panic("poll() returned POLLERR");
                if (pfd.revents & POLLIN) {
                    clientsChannel[i].headerPktIn();
                }
                if (pfd.revents & POLLRDHUP) {
                    // One gem5 process exited or aborted. Either way, we
                    // assume the full simulation should stop now (either
                    // because m5 exit was called or a serious error
                    // occurred.) So we quit the run loop here and close all
                    // sockets to notify the remaining peer gem5 processes.
                    pfd.events = 0;
                    clientsChannel[i].isAlive = false;
                    num_active_clients--;
                    DPRINTF(debugSetup, "POLLRDHUP event");
                }
                n++;
                if ((signed)n == nfd)
                    break;
            }
        }
    }
    DPRINTF(debugSetup, "Exiting run() loop\n");
}

void
TCPServer::xferData(const Header &hdr_pkt, const Channel &src)
{
    unsigned n;
    assert(hdr_pkt.dataPacketLength <= sizeof(packetBuffer));
    n = src.recvRaw(packetBuffer, hdr_pkt.dataPacketLength);

    if (n == 0)
        panic("recvRaw() failed");
    DPRINTF(debugPkt, "Incoming data packet (from rank %d) "
            "src:0x%02x%02x%02x%02x%02x%02x "
            "dst:0x%02x%02x%02x%02x%02x%02x\n",
            src.rank,
            hdr_pkt.srcAddress[0],
            hdr_pkt.srcAddress[1],
            hdr_pkt.srcAddress[2],
            hdr_pkt.srcAddress[3],
            hdr_pkt.srcAddress[4],
            hdr_pkt.srcAddress[5],
            hdr_pkt.dstAddress[0],
            hdr_pkt.dstAddress[1],
            hdr_pkt.dstAddress[2],
            hdr_pkt.dstAddress[3],
            hdr_pkt.dstAddress[4],
            hdr_pkt.dstAddress[5]);
    // Now try to figure out the destination client(s).
    auto dst_info = addressMap.find(&hdr_pkt.dstAddress);

    // First handle the multicast/broadcast or unknonw destination case. These
    // all trigger a broadcast of the packet to all clients.
    if (MultiHeaderPkt::isUnicastAddress(hdr_pkt.dstAddress) == false ||
        dst_info == addressMap.end()) {
        unsigned n = 0;
        for (auto const &c: clientsChannel) {
            if (c.isAlive && &c!=&src) {
                c.sendRaw(&hdr_pkt, sizeof(hdr_pkt));
                c.sendRaw(packetBuffer, hdr_pkt.dataPacketLength);
                n++;
            }
        }
        if (n == 0) {
            inform("Broadcast/multicast packet dropped\n");
        }
    } else {
        // It is a unicast address with a known destination
        Channel *dst = dst_info->second;
        if (dst->isAlive) {
            dst->sendRaw(&hdr_pkt, sizeof(hdr_pkt));
            dst->sendRaw(packetBuffer, hdr_pkt.dataPacketLength);
            DPRINTF(debugPkt, "Unicast packet sent (to rank %d)\n",dst->rank);
        } else {
            inform("Unicast packet dropped (destination exited)\n");
        }
    }
}

void
TCPServer::syncTryComplete(SyncState st, MsgType ack)
{
    // Check if the barrieris complete. If so then notify all the clients.
    for (auto &c : clientsChannel) {
        if (c.isAlive && (c.state != st)) {
            // sync not complete yet, stop here
            return;
        }
    }
    // Sync complete, send out the acks
    MultiHeaderPkt::Header hdr_pkt;
    hdr_pkt.msgType = ack;
    for (auto &c : clientsChannel) {
        if (c.isAlive) {
            c.sendRaw(&hdr_pkt, sizeof(hdr_pkt));
            c.state = SyncState::idle;
        }
    }
    // Reset periodic send tick
    _periodicSyncTick = 0;
    DPRINTF(st == SyncState::periodic ? debugPeriodic : debugSync,
            "Sync COMPLETE\n");
}

void
TCPServer::ckptPropagate(Channel &ch)
{
    // Channel ch got a ckpt request that needs to be propagated to the other
    // clients
    MultiHeaderPkt::Header hdr_pkt;
    hdr_pkt.msgType = MsgType::cmdCkptSyncReq;
    for (auto &c : clientsChannel) {
        if (c.isAlive && (&c != &ch)) {
            switch (c.state) {
              case SyncState::idle:
              case SyncState::periodic:
                c.sendRaw(&hdr_pkt, sizeof(hdr_pkt));
                c.state = SyncState::asyncCkpt;
                break;
              default:
                panic("Unexpected state for ckpt sync request propagation "
                      "(rank:%d)\n",c.rank);
                break;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    TCPServer *server;
    int clients_num = -1, listen_port = -1;
    int first_arg = 1, timeout_in_sec = 60;

    if (argc > 1 && string(argv[1]).compare("-debug") == 0) {
        timeout_in_sec = -1;
        first_arg++;
        argc--;
    }

    if (argc != 3)
        panic("We need two command line args (number of clients and tcp listen"
              " port");

    clients_num = atoi(argv[first_arg]);
    listen_port = atoi(argv[first_arg + 1]);

    server = new TCPServer(clients_num, listen_port, timeout_in_sec);

    server->run();

    delete server;

    return 0;
}
