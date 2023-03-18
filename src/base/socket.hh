/*
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

#ifndef __SOCKET_HH__
#define __SOCKET_HH__

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>

#include <string>

#include "base/named.hh"

namespace gem5
{

/**
 * @brief Wrapper around sockaddr_un, so that it can be used for both file
 * based unix sockets as well as abstract unix sockets.
 */
struct UnixSocketAddr
{
    /**
     * @brief Builds UnixSocketAddr from the given path.
     * @pre: `path` either represents a file based unix socket, or an abstract
     *       unix socket. If `path` represents an abstract socket, it should
     *       start with the character '@', and it should not have any null
     *       bytes in the name.
     * @param path: Pathname, where the socket should be instantiated.
     * @return UnixSocketAddr
     */
    static UnixSocketAddr build(const std::string &path);

    sockaddr_un addr;
    // Size of `sockaddr_un addr`. This is equal to sizeof(sockaddr_un) if
    // `addr` represents a normal file based unix socket. For abstract sockets
    // however, the size could be different. Because all sizeof(sun_path) is
    // used to represent the name of an abstract socket, addrSize for abstract
    // sockets only count the number of characters actually used by sun_path,
    // excluding any trailing null bytes.
    size_t addrSize;
    bool isAbstract;
    // Formatted string for file based sockets look the same as addr.sun_path.
    // For abstract sockets however, all null bytes are replaced with @
    std::string formattedPath;
};

class ListenSocket : public Named
{
  protected:
    /**
     * The following variables are only used by socket unit tests:
     * listeningDisabled, anyListening, bindToLoopback.
     */
    static bool listeningDisabled;
    static bool anyListening;

    static bool bindToLoopback;

  public:
    static void disableAll();
    static bool allDisabled();

    static void loopbackOnly();

  protected:
    bool listening;
    int fd;
    int _port;

    /*
     * cleanup resets the static variables back to their default values.
     */
    static void cleanup();

  public:
    /**
     * @ingroup api_socket
     * @{
     */
    ListenSocket(const std::string &_name, int port);
    ListenSocket();
    virtual ~ListenSocket();

    virtual int accept();

    virtual bool listen(int port);
    virtual void listen();

    virtual void output(std::ostream &os) const;

    int getfd() const { return fd; }
    bool islistening() const { return listening; }

    /* Create a socket, adding SOCK_CLOEXEC if available. */
    static int socketCloexec(int domain, int type, int protocol);
    /* Accept a connection, adding SOCK_CLOEXEC if available. */
    static int acceptCloexec(int sockfd, struct sockaddr *addr,
                              socklen_t *addrlen);
    /** @} */ // end of api_socket
};

inline static std::ostream &
operator << (std::ostream &os, const ListenSocket &socket)
{
    socket.output(os);
    return os;
}

} // namespace gem5

#endif //__SOCKET_HH__
