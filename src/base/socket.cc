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

// check if filesystem library is available
#if defined(__cpp_lib_filesystem) || __has_include(<filesystem>)
#include <filesystem>
#else
// This is only reachable if we're using GCC 7 or clang versions 6
// through 10 (note: gem5 does not support GCC versions older than
// GCC 7 or clang versions older than clang 6.0 as they do not
// support the C++17 standard).
// If we're using GCC 7 or clang versions 6 through 10, we need to use
// <experimental/filesystem>.
#include <experimental/filesystem>

namespace std
{
namespace filesystem = experimental::filesystem;
}
#endif

#include "base/logging.hh"
#include "base/output.hh"
#include "base/str.hh"
#include "base/types.hh"
#include "sim/byteswap.hh"

namespace gem5
{

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

ListenSocket::ListenSocket(const std::string &_name) : Named(_name) {}

ListenSocket::~ListenSocket()
{
    if (fd != -1)
        close(fd);
}

// Open a connection.  Accept will block, so if you don't want it to,
// make sure a connection is ready before you call accept.
int
ListenSocket::accept()
{
    struct sockaddr_in sockaddr;
    socklen_t slen = sizeof(sockaddr);
    int sfd = acceptCloexec(fd, (struct sockaddr *)&sockaddr, &slen);
    panic_if(sfd == -1, "%s: Failed to accept connection: %s", name(),
             strerror(errno));

    return sfd;
}

bool
ListenSocketConfig::parseIni(const std::string &value,
                             ListenSocketConfig &retval)
{
    if (value.size() == 0) {
        retval = listenSocketEmptyConfig();
        return true;
    } else if (value[0] == '@') {
        retval = listenSocketUnixAbstractConfig(value.substr(1));
        return true;
    } else if (value[0] == 'P') {
        std::filesystem::path p(value.substr(1));
        retval = listenSocketUnixFileConfig(p.parent_path(), p.filename());
        return true;
    } else if (value[0] == '#') {
        uint64_t port;
        bool ret = to_number(value.substr(1), port);
        if (!ret)
            return false;
        retval = listenSocketInetConfig(port);
        return true;
    } else {
        panic("Can't interpret %s as a host socket.", value);
    }
}

ListenSocketInet::ListenSocketInet(const std::string &_name, int port)
    : ListenSocket(_name), _port(port)
{}

int
ListenSocketInet::accept()
{
    int sfd = ListenSocket::accept();
    if (sfd == -1)
        return -1;

    int i = 1;
    int ret = ::setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, &i, sizeof(i));
    warn_if(ret < 0, "ListenSocket(accept): setsockopt() TCP_NODELAY failed!");

    return sfd;
}

// Create a socket and configure it for listening
bool
ListenSocketInet::listen(int port)
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
    ret = ::bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
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

    setListening();
    return true;
}

void
ListenSocketInet::listen()
{
    while (!listen(_port)) {
        _port++;
        fatal_if(_port > 65536, "%s: cannot find an available port.", name());
    }
    ccprintf(std::cerr, "%s: Listening for connections on %s\n", name(),
             *this);
}

void
ListenSocketInet::output(std::ostream &os) const
{
    os << "port " << _port;
}

ListenSocketConfig
listenSocketInetConfig(int port)
{
    return ListenSocketConfig([port](const std::string &name) {
        return std::make_unique<ListenSocketInet>(name, port);
    });
}

void
ListenSocketUnix::checkPathLength(const std::string &original, size_t max_len)
{
    fatal_if(original.size() > max_len,
             "Length of socket path '%s' is %d, greater than max %d.",
             original, original.size(), max_len);
}

void
ListenSocketUnix::listen()
{
    panic_if(listening, "%s: Socket already listening!", name());

    // only create socket if not already created by previous call
    if (fd == -1) {
        fd = socketCloexec(PF_UNIX, SOCK_STREAM, 0);
        panic_if(fd < 0, "%s: Can't create unix socket:%s !", name(),
                 strerror(errno));
    }

    sockaddr_un serv_addr;
    std::memset(&serv_addr, 0, sizeof(serv_addr));
    size_t addr_size = prepSockaddrUn(serv_addr);

    fatal_if(bind(fd, (struct sockaddr *)&(serv_addr), addr_size) != 0,
             "%s: Cannot bind unix socket %s: %s", name(), *this,
             strerror(errno));

    fatal_if(::listen(fd, 1) == -1, "%s: Failed to listen on %s: %s\n", name(),
             *this, strerror(errno));

    ccprintf(std::cerr, "%s: Listening for connections on %s\n", name(),
             *this);

    setListening();
}

ListenSocketUnixFile::ListenSocketUnixFile(const std::string &_name,
                                           const std::string &_dir,
                                           const std::string &_fname)
    : ListenSocketUnix(_name), dir(_dir), fname(_fname)
{
    checkPathLength(fname, sizeof(sockaddr_un::sun_path) - 1);
}

ListenSocketUnixFile::~ListenSocketUnixFile()
{
    if (fd != -1) {
        close(fd);
        fd = -1;
        unlink();
    }
}

bool
ListenSocketUnixFile::unlink() const
{
    auto path = resolvedDir + "/" + fname;
    return ::unlink(path.c_str()) == 0;
}

size_t
ListenSocketUnixFile::prepSockaddrUn(sockaddr_un &addr) const
{
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, fname.c_str(), fname.size());
    return sizeof(addr.sun_path);
}

void
ListenSocketUnixFile::listen()
{
    resolvedDir = simout.resolve(dir);
    warn_if(unlink(),
            "%s: server path %s was occupied and will be replaced. Please "
            "make sure there is no other server using the same path.",
            name(), resolvedDir + "/" + fname);

    // Make sure "dir" exists.
    std::error_code ec;
    std::filesystem::create_directory(resolvedDir, ec);
    fatal_if(ec, "Failed to create directory %s", ec.message());

    // Change the working directory to the directory containing the socket so
    // that we maximize the limited space in sockaddr_un.sun_path.
    auto cwd = std::filesystem::current_path(ec);
    panic_if(ec, "Failed to get current working directory %s", ec.message());
    std::filesystem::current_path(resolvedDir, ec);
    fatal_if(ec, "Failed to change to directory %s: %s", resolvedDir,
             ec.message());

    ListenSocketUnix::listen();

    std::filesystem::current_path(cwd, ec);
    panic_if(ec, "Failed to change back working directory %s", ec.message());
}

void
ListenSocketUnixFile::output(std::ostream &os) const
{
    os << "socket \"" << dir << "/" << fname << "\"";
}

ListenSocketConfig
listenSocketUnixFileConfig(std::string dir, std::string fname)
{
    return ListenSocketConfig([dir, fname](const std::string &name) {
        return std::make_unique<ListenSocketUnixFile>(name, dir, fname);
    });
}

size_t
ListenSocketUnixAbstract::prepSockaddrUn(sockaddr_un &addr) const
{
    addr.sun_family = AF_UNIX;
    addr.sun_path[0] = '\0';
    std::memcpy(&addr.sun_path[1], path.c_str(), path.size());
    return offsetof(sockaddr_un, sun_path) + path.size() + 1;
}

ListenSocketUnixAbstract::ListenSocketUnixAbstract(const std::string &_name,
                                                   const std::string &_path)
    : ListenSocketUnix(_name), path(_path)
{
    checkPathLength(path, sizeof(sockaddr_un::sun_path) - 1);
}

void
ListenSocketUnixAbstract::output(std::ostream &os) const
{
    os << "abstract socket \"" << path << "\"";
}

ListenSocketConfig
listenSocketUnixAbstractConfig(std::string path)
{
    return ListenSocketConfig([path](const std::string &name) {
        return std::make_unique<ListenSocketUnixAbstract>(name, path);
    });
}

} // namespace gem5
