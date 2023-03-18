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
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>

#include "base/logging.hh"
#include "base/output.hh"
#include "base/str.hh"
#include "base/types.hh"
#include "sim/byteswap.hh"

namespace gem5
{
namespace
{

bool
isSocketNameAbstract(const std::string &path)
{
    if (path.empty()) {
        return false;
    }
    // No null byte should be present in the path
    return path.front() == '@';
}

std::string
resolve(const std::string &path)
{
    if (path.empty()) {
        return path;
    }
    if (isSocketNameAbstract(path)) {
        return '\0' + path.substr(1);
    }
    return simout.resolve(path);
}

}  // namespace

bool ListenSocket::listeningDisabled = false;
bool ListenSocket::anyListening = false;

bool ListenSocket::bindToLoopback = false;

UnixSocketAddr
UnixSocketAddr::build(const std::string &path)
{
    sockaddr_un addr = {.sun_family = AF_UNIX, .sun_path = {}};

    const bool is_abstract = isSocketNameAbstract(path);
    size_t max_len = sizeof(addr.sun_path);
    if (!is_abstract) {
        // File based socket names need to be null terminated
        max_len -= 1;
    }

    std::string resolved_path = resolve(path);
    std::string fmt_path = replace(resolved_path, '\0', '@');
    if (resolved_path.size() > max_len) {
        resolved_path = resolved_path.substr(0, max_len);
        const std::string untruncated_path = std::move(fmt_path);
        fmt_path = replace(resolved_path, '\0', '@');
        warn("SocketPath: unix socket path truncated from '%s' to '%s'",
             untruncated_path, fmt_path);
    }

    // We can't use strncpy here, since abstract sockets start with \0 which
    // will make strncpy think that the string is empty.
    memcpy(addr.sun_path, resolved_path.c_str(), resolved_path.size());
    // We can't use sizeof(sockaddr_un) for abstract sockets, since all
    // sizeof(sun_path) bytes are used in representing the path.
    const size_t path_size =
        is_abstract ? resolved_path.size() : sizeof(addr.sun_path);
    const size_t addr_size = offsetof(sockaddr_un, sun_path) + path_size;

    return UnixSocketAddr{.addr = std::move(addr),
                          .addrSize = addr_size,
                          .isAbstract = is_abstract,
                          .formattedPath = std::move(fmt_path)};
}

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

// Wrappers to stub out SOCK_CLOEXEC/accept4 availability

int
ListenSocket::socketCloexec(int domain, int type, int protocol)
{
#ifdef SOCK_CLOEXEC
    type |= SOCK_CLOEXEC;
#endif
    return ::socket(domain, type, protocol);
}

int
ListenSocket::acceptCloexec(int sockfd, struct sockaddr *addr,
                             socklen_t *addrlen)
{
#if defined(_GNU_SOURCE) && defined(SOCK_CLOEXEC)
    return ::accept4(sockfd, addr, addrlen, SOCK_CLOEXEC);
#else
    return ::accept(sockfd, addr, addrlen);
#endif
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
ListenSocket::listen(int port)
{
    panic_if(listening, "Socket already listening!");

    // only create socket if not already created by a previous call
    if (fd == -1) {
        fd = socketCloexec(PF_INET, SOCK_STREAM, 0);
        panic_if(fd < 0, "Can't create socket:%s !", strerror(errno));
    }

    int i = 1;
    int ret = ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i));
    panic_if(ret < 0,
            "ListenSocket(listen): setsockopt() SO_REUSEADDR failed!");

    struct sockaddr_in sockaddr;
    sockaddr.sin_family = PF_INET;
    sockaddr.sin_addr.s_addr =
        htobe<in_addr_t>(bindToLoopback ? INADDR_LOOPBACK : INADDR_ANY);
    sockaddr.sin_port = htons(port);
    // finally clear sin_zero
    std::memset(&sockaddr.sin_zero, 0, sizeof(sockaddr.sin_zero));
    ret = ::bind(fd, (struct sockaddr *)&sockaddr, sizeof (sockaddr));
    if (ret != 0) {
        panic_if(ret == -1 && errno != EADDRINUSE,
                "ListenSocket(listen): bind() failed!");
        return false;
    }

    if (::listen(fd, 1) == -1) {
        panic_if(errno != EADDRINUSE,
                "ListenSocket(listen): listen() failed!");
        // User may decide to retry with a different port later; however, the
        // socket is already bound to a port and the next bind will surely
        // fail. We'll close the socket and reset fd to -1 so our user can
        // retry with a cleaner state.
        close(fd);
        fd = -1;
        return false;
    }

    listening = true;
    anyListening = true;
    return true;
}


// Open a connection.  Accept will block, so if you don't want it to,
// make sure a connection is ready before you call accept.
int
ListenSocket::accept()
{
    struct sockaddr_in sockaddr;
    socklen_t slen = sizeof (sockaddr);
    int sfd = acceptCloexec(fd, (struct sockaddr *)&sockaddr, &slen);
    if (sfd == -1)
        return -1;

    int i = 1;
    int ret = ::setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, &i, sizeof(i));
    warn_if(ret < 0, "ListenSocket(accept): setsockopt() TCP_NODELAY failed!");

    return sfd;
}

} // namespace gem5
