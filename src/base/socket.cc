/*
 * Copyright (c) 2020 The Regents of the University of California
 * All rights reserved
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "base/socket.hh"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>

#include "base/logging.hh"
#include "base/types.hh"
#include "sim/byteswap.hh"

using namespace std;

bool ListenSocket::listeningDisabled = false;
bool ListenSocket::anyListening = false;

bool ListenSocket::bindToLoopback = false;

void
ListenSocket::cleanup()
{
    listeningDisabled = false;
    anyListening = false;
    bindToLoopback = false;
}

void
ListenSocket::disableAll()
{
    if (anyListening)
        panic("Too late to disable all listeners, already have a listener");
    listeningDisabled = true;
}

bool
ListenSocket::allDisabled()
{
    return listeningDisabled;
}

void
ListenSocket::loopbackOnly()
{
    if (anyListening)
        panic("Too late to bind to loopback, already have a listener");
    bindToLoopback = true;
}

////////////////////////////////////////////////////////////////////////
//
//

ListenSocket::ListenSocket()
    : listening(false), fd(-1)
{}

ListenSocket::~ListenSocket()
{
    if (fd != -1)
        close(fd);
}

// Create a socket and configure it for listening
bool
ListenSocket::listen(int port, bool reuse)
{
    if (listening)
        panic("Socket already listening!");

    fd = ::socket(PF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        panic("Can't create socket:%s !", strerror(errno));

    if (reuse) {
        int i = 1;
        if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *)&i,
                         sizeof(i)) < 0)
            panic("ListenSocket(listen): setsockopt() SO_REUSEADDR failed!");
    }

    struct sockaddr_in sockaddr;
    sockaddr.sin_family = PF_INET;
    sockaddr.sin_addr.s_addr =
        htobe<in_addr_t>(bindToLoopback ? INADDR_LOOPBACK : INADDR_ANY);
    sockaddr.sin_port = htons(port);
    // finally clear sin_zero
    memset(&sockaddr.sin_zero, 0, sizeof(sockaddr.sin_zero));
    int ret = ::bind(fd, (struct sockaddr *)&sockaddr, sizeof (sockaddr));
    if (ret != 0) {
        if (ret == -1 && errno != EADDRINUSE)
            panic("ListenSocket(listen): bind() failed!");
        return false;
    }

    if (::listen(fd, 1) == -1) {
        if (errno != EADDRINUSE)
            panic("ListenSocket(listen): listen() failed!");

        return false;
    }

    listening = true;
    anyListening = true;
    return true;
}


// Open a connection.  Accept will block, so if you don't want it to,
// make sure a connection is ready before you call accept.
int
ListenSocket::accept(bool nodelay)
{
    struct sockaddr_in sockaddr;
    socklen_t slen = sizeof (sockaddr);
    int sfd = ::accept(fd, (struct sockaddr *)&sockaddr, &slen);
    if (sfd != -1 && nodelay) {
        int i = 1;
        if (::setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, (char *)&i,
                         sizeof(i)) < 0)
            warn("ListenSocket(accept): setsockopt() TCP_NODELAY failed!");
    }

    return sfd;
}
