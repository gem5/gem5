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
 */

/* @file
 * TCP stream socket based interface class implementation for dist-gem5 runs.
 */

#include "dev/net/tcp_iface.hh"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

#include "base/trace.hh"
#include "base/types.hh"
#include "debug/DistEthernet.hh"
#include "debug/DistEthernetCmd.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"

#if defined(__FreeBSD__)
#include <netinet/in.h>

#endif

// MSG_NOSIGNAL does not exists on OS X
#if defined(__APPLE__) || defined(__MACH__)
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif
#endif

using namespace std;

std::vector<std::pair<TCPIface::NodeInfo, int> > TCPIface::nodes;
vector<int> TCPIface::sockRegistry;
int TCPIface::fdStatic = -1;
bool TCPIface::anyListening = false;

TCPIface::TCPIface(string server_name, unsigned server_port,
                   unsigned dist_rank, unsigned dist_size,
                   Tick sync_start, Tick sync_repeat,
                   EventManager *em, bool use_pseudo_op, bool is_switch,
                   int num_nodes) :
    DistIface(dist_rank, dist_size, sync_start, sync_repeat, em, use_pseudo_op,
              is_switch, num_nodes), serverName(server_name),
    serverPort(server_port), isSwitch(is_switch), listening(false)
{
    if (is_switch && isPrimary) {
        while (!listen(serverPort)) {
            DPRINTF(DistEthernet, "TCPIface(listen): Can't bind port %d\n",
                    serverPort);
            serverPort++;
        }
        inform("tcp_iface listening on port %d", serverPort);
        // Now accept the first connection requests from each compute node and
        // store the node info. The compute nodes will then wait for ack
        // messages. Ack messages will be sent by initTransport() in the
        // appropriate order to make sure that every compute node is always
        // connected to the same switch port.
        NodeInfo ni;
        for (int i = 0; i < size; i++) {
            accept();
            DPRINTF(DistEthernet, "First connection, waiting for link info\n");
            if (!recvTCP(sock, &ni, sizeof(ni)))
                panic("Failed to receive link info");
            nodes.push_back(make_pair(ni, sock));
        }
    }
}

bool
TCPIface::listen(int port)
{
    if (listening)
        panic("Socket already listening!");

    struct sockaddr_in sockaddr;
    int ret;

    fdStatic = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    panic_if(fdStatic < 0, "socket() failed: %s", strerror(errno));

    sockaddr.sin_family = PF_INET;
    sockaddr.sin_addr.s_addr = INADDR_ANY;
    sockaddr.sin_port = htons(port);
    // finally clear sin_zero
    memset(&sockaddr.sin_zero, 0, sizeof(sockaddr.sin_zero));
    ret = ::bind(fdStatic, (struct sockaddr *)&sockaddr, sizeof (sockaddr));

    if (ret != 0) {
        if (ret == -1 && errno != EADDRINUSE)
            panic("ListenSocket(listen): bind() failed!");
        return false;
    }

    if (::listen(fdStatic, 24) == -1) {
        if (errno != EADDRINUSE)
            panic("ListenSocket(listen): listen() failed!");

        return false;
    }

    listening = true;
    anyListening = true;
    return true;
}

void
TCPIface::establishConnection()
{
    static unsigned cur_rank = 0;
    static unsigned cur_id = 0;
    NodeInfo ni;

    if (isSwitch) {
        if (cur_id == 0) { // first connection accepted in the ctor already
            auto const &iface0 =
                find_if(nodes.begin(), nodes.end(),
                        [](const pair<NodeInfo, int> &cn) -> bool {
                            return cn.first.rank == cur_rank;
                        });
            assert(iface0 != nodes.end());
            assert(iface0->first.distIfaceId == 0);
            sock = iface0->second;
            ni = iface0->first;
        } else { // additional connections from the same compute node
            accept();
            DPRINTF(DistEthernet, "Next connection, waiting for link info\n");
            if (!recvTCP(sock, &ni, sizeof(ni)))
                panic("Failed to receive link info");
            assert(ni.rank == cur_rank);
            assert(ni.distIfaceId == cur_id);
        }
        inform("Link okay  (iface:%d -> (node:%d, iface:%d))",
               distIfaceId, ni.rank, ni.distIfaceId);
        if (ni.distIfaceId < ni.distIfaceNum - 1) {
            cur_id++;
        } else {
            cur_rank++;
            cur_id = 0;
        }
        // send ack
        ni.distIfaceId = distIfaceId;
        ni.distIfaceNum = distIfaceNum;
        sendTCP(sock, &ni, sizeof(ni));
    } else { // this is not a switch
        connect();
        // send link info
        ni.rank = rank;
        ni.distIfaceId = distIfaceId;
        ni.distIfaceNum = distIfaceNum;
        sendTCP(sock, &ni, sizeof(ni));
        DPRINTF(DistEthernet, "Connected, waiting for ack (distIfaceId:%d\n",
                distIfaceId);
        if (!recvTCP(sock, &ni, sizeof(ni)))
            panic("Failed to receive ack");
        assert(ni.rank == rank);
        inform("Link okay  (iface:%d -> switch iface:%d)", distIfaceId,
               ni.distIfaceId);
    }
    sockRegistry.push_back(sock);
}

void
TCPIface::accept()
{
    struct sockaddr_in sockaddr;
    socklen_t slen = sizeof (sockaddr);
    sock = ::accept(fdStatic, (struct sockaddr *)&sockaddr, &slen);
    if (sock != -1) {
        int i = 1;
        if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&i,
                         sizeof(i)) < 0)
            warn("ListenSocket(accept): setsockopt() TCP_NODELAY failed!");
    }
}

void
TCPIface::connect()
{
    struct addrinfo addr_hint, *addr_results;
     int ret;

     string port_str = to_string(serverPort);

     sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
     panic_if(sock < 0, "socket() failed: %s", strerror(errno));

     int fl = 1;
     if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&fl, sizeof(fl)) < 0)
         warn("ConnectSocket(connect): setsockopt() TCP_NODELAY failed!");

     bzero(&addr_hint, sizeof(addr_hint));
     addr_hint.ai_family = AF_INET;
     addr_hint.ai_socktype = SOCK_STREAM;
     addr_hint.ai_protocol = IPPROTO_TCP;

     ret = getaddrinfo(serverName.c_str(), port_str.c_str(),
                       &addr_hint, &addr_results);
     panic_if(ret < 0, "getaddrinf() failed: %s", strerror(errno));

     DPRINTF(DistEthernet, "Connecting to %s:%s\n",
             serverName.c_str(), port_str.c_str());

     ret = ::connect(sock, (struct sockaddr *)(addr_results->ai_addr),
                     addr_results->ai_addrlen);
     panic_if(ret < 0, "connect() failed: %s", strerror(errno));

     freeaddrinfo(addr_results);
}

TCPIface::~TCPIface()
{
    int M5_VAR_USED ret;

    ret = close(sock);
    assert(ret == 0);
}

void
TCPIface::sendTCP(int sock, const void *buf, unsigned length)
{
    ssize_t ret;

    ret = ::send(sock, buf, length, MSG_NOSIGNAL);
    if (ret < 0) {
        if (errno == ECONNRESET || errno == EPIPE) {
            exitSimLoop("Message server closed connection, simulation "
                        "is exiting");
        } else {
            panic("send() failed: %s", strerror(errno));
        }
    }
    panic_if(ret != length, "send() failed");
}

bool
TCPIface::recvTCP(int sock, void *buf, unsigned length)
{
    ssize_t ret;

    ret = ::recv(sock, buf, length,  MSG_WAITALL );
    if (ret < 0) {
        if (errno == ECONNRESET || errno == EPIPE)
            inform("recv(): %s", strerror(errno));
        else if (ret < 0)
            panic("recv() failed: %s", strerror(errno));
    } else if (ret == 0) {
        inform("recv(): Connection closed");
    } else if (ret != length)
        panic("recv() failed");

    return (ret == length);
}

void
TCPIface::sendPacket(const Header &header, const EthPacketPtr &packet)
{
    sendTCP(sock, &header, sizeof(header));
    sendTCP(sock, packet->data, packet->length);
}

void
TCPIface::sendCmd(const Header &header)
{
    DPRINTF(DistEthernetCmd, "TCPIface::sendCmd() type: %d\n",
            static_cast<int>(header.msgType));
    // Global commands (i.e. sync request) are always sent by the primary
    // DistIface. The transfer method is simply implemented as point-to-point
    // messages for now
    for (auto s: sockRegistry)
        sendTCP(s, (void*)&header, sizeof(header));
}

bool
TCPIface::recvHeader(Header &header)
{
    bool ret = recvTCP(sock, &header, sizeof(header));
    DPRINTF(DistEthernetCmd, "TCPIface::recvHeader() type: %d ret: %d\n",
            static_cast<int>(header.msgType), ret);
    return ret;
}

void
TCPIface::recvPacket(const Header &header, EthPacketPtr &packet)
{
    packet = make_shared<EthPacketData>(header.dataPacketLength);
    bool ret = recvTCP(sock, packet->data, header.dataPacketLength);
    panic_if(!ret, "Error while reading socket");
    packet->simLength = header.simLength;
    packet->length = header.dataPacketLength;
}

void
TCPIface::initTransport()
{
    // We cannot setup the conections in the constructor because the number
    // of dist interfaces (per process) is unknown until the (simobject) init
    // phase. That information is necessary for global connection ordering.
    establishConnection();
}
