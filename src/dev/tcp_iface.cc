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
 * TCP stream socket based interface class implementation for multi gem5 runs.
 */

#include "dev/tcp_iface.hh"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "base/types.hh"
#include "debug/MultiEthernet.hh"

// MSG_NOSIGNAL does not exists on OS X
#if defined(__APPLE__) || defined(__MACH__)
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif
#endif

using namespace std;

vector<int> TCPIface::sockRegistry;

TCPIface::TCPIface(string server_name, unsigned server_port,
                   unsigned multi_rank, Tick sync_start, Tick sync_repeat,
                   EventManager *em) :
    MultiIface(multi_rank, sync_start, sync_repeat, em)
{
    struct addrinfo addr_hint, *addr_results;
    int ret;

    string port_str = to_string(server_port);

    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    panic_if(sock < 0, "socket() failed: %s", strerror(errno));

    bzero(&addr_hint, sizeof(addr_hint));
    addr_hint.ai_family = AF_INET;
    addr_hint.ai_socktype = SOCK_STREAM;
    addr_hint.ai_protocol = IPPROTO_TCP;

    ret = getaddrinfo(server_name.c_str(), port_str.c_str(),
                      &addr_hint, &addr_results);
    panic_if(ret < 0, "getaddrinf() failed: %s", strerror(errno));

    DPRINTF(MultiEthernet, "Connecting to %s:%u\n",
            server_name.c_str(), port_str.c_str());

    ret = ::connect(sock, (struct sockaddr *)(addr_results->ai_addr),
                    addr_results->ai_addrlen);
    panic_if(ret < 0, "connect() failed: %s", strerror(errno));

    freeaddrinfo(addr_results);
    // add our socket to the static registry
    sockRegistry.push_back(sock);
    // let the server know who we are
    sendTCP(sock, &multi_rank, sizeof(multi_rank));
}

TCPIface::~TCPIface()
{
    int M5_VAR_USED ret;

    ret = close(sock);
    assert(ret == 0);
}

void
TCPIface::sendTCP(int sock, void *buf, unsigned length)
{
    ssize_t ret;

    ret = ::send(sock, buf, length, MSG_NOSIGNAL);
    panic_if(ret < 0, "send() failed: %s", strerror(errno));
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
TCPIface::syncRaw(MultiHeaderPkt::MsgType sync_req, Tick sync_tick)
{
    /*
     * Barrier is simply implemented by point-to-point messages to the server
     * for now. This method is called by only one TCPIface object.
     * The server will send back an 'ack' message when it gets the
     * sync request from all clients.
     */
    MultiHeaderPkt::Header header_pkt;
    header_pkt.msgType = sync_req;
    header_pkt.sendTick = sync_tick;

    for (auto s : sockRegistry)
        sendTCP(s, (void *)&header_pkt, sizeof(header_pkt));
}

