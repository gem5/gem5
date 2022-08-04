/*
 * Copyright 2022 Google, Inc.
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

#include "mem/shared_memory_server.hh"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

#include "base/logging.hh"
#include "base/output.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"

namespace gem5
{
namespace memory
{

SharedMemoryServer::SharedMemoryServer(const SharedMemoryServerParams& params)
    : SimObject(params), unixSocketPath(simout.resolve(params.server_path)),
      system(params.system), serverFd(-1)
{
    fatal_if(system == nullptr, "Requires a system to share memory from!");
    // Ensure the unix socket path to use is not occupied. Also, if there's
    // actually anything to be removed, warn the user something might be off.
    if (unlink(unixSocketPath.c_str()) == 0) {
        warn(
            "The server path %s was occupied and will be replaced. Please "
            "make sure there is no other server using the same path.",
            unixSocketPath.c_str());
    }
    // Create a new unix socket.
    serverFd = ListenSocket::socketCloexec(AF_UNIX, SOCK_STREAM, 0);
    panic_if(serverFd < 0, "%s: cannot create unix socket: %s", name().c_str(),
             strerror(errno));
    // Bind to the specified path.
    sockaddr_un serv_addr = {};
    serv_addr.sun_family = AF_UNIX;
    strncpy(serv_addr.sun_path, unixSocketPath.c_str(),
            sizeof(serv_addr.sun_path) - 1);
    warn_if(strlen(serv_addr.sun_path) != unixSocketPath.size(),
            "%s: unix socket path truncated, expect '%s' but get '%s'",
            name().c_str(), unixSocketPath.c_str(), serv_addr.sun_path);
    int bind_retv = bind(serverFd, reinterpret_cast<sockaddr*>(&serv_addr),
                         sizeof(serv_addr));
    fatal_if(bind_retv != 0, "%s: cannot bind unix socket: %s", name().c_str(),
             strerror(errno));
    // Start listening.
    int listen_retv = listen(serverFd, 1);
    fatal_if(listen_retv != 0, "%s: listen failed: %s", name().c_str(),
             strerror(errno));
    listenSocketEvent.reset(new ListenSocketEvent(serverFd, this));
    pollQueue.schedule(listenSocketEvent.get());
    inform("%s: listening at %s", name().c_str(), unixSocketPath.c_str());
}

SharedMemoryServer::~SharedMemoryServer()
{
    int unlink_retv = unlink(unixSocketPath.c_str());
    warn_if(unlink_retv != 0, "%s: cannot unlink unix socket: %s",
            name().c_str(), strerror(errno));
    int close_retv = close(serverFd);
    warn_if(close_retv != 0, "%s: cannot close unix socket: %s",
            name().c_str(), strerror(errno));
}

SharedMemoryServer::BaseShmPollEvent::BaseShmPollEvent(
    int fd, SharedMemoryServer* shm_server)
    : PollEvent(fd, POLLIN), shmServer(shm_server),
      eventName(shmServer->name() + ".fd" + std::to_string(fd))
{
}

const std::string&
SharedMemoryServer::BaseShmPollEvent::name() const
{
    return eventName;
}

bool
SharedMemoryServer::BaseShmPollEvent::tryReadAll(void* buffer, size_t size)
{
    char* char_buffer = reinterpret_cast<char*>(buffer);
    for (size_t offset = 0; offset < size;) {
        ssize_t retv = recv(pfd.fd, char_buffer + offset, size - offset, 0);
        if (retv >= 0) {
            offset += retv;
        } else if (errno != EINTR) {
            warn("%s: recv failed: %s", name().c_str(), strerror(errno));
            return false;
        }
    }
    return true;
}

void
SharedMemoryServer::ListenSocketEvent::process(int revents)
{
    panic_if(revents & (POLLERR | POLLNVAL), "%s: listen socket is broken",
             name().c_str());
    int cli_fd = ListenSocket::acceptCloexec(pfd.fd, nullptr, nullptr);
    panic_if(cli_fd < 0, "%s: accept failed: %s", name().c_str(),
             strerror(errno));
    panic_if(shmServer->clientSocketEvent.get(),
             "%s: cannot serve two clients at once", name().c_str());
    inform("%s: accept new connection %d", name().c_str(), cli_fd);
    shmServer->clientSocketEvent.reset(
        new ClientSocketEvent(cli_fd, shmServer));
    pollQueue.schedule(shmServer->clientSocketEvent.get());
}

void
SharedMemoryServer::ClientSocketEvent::process(int revents)
{
    do {
        // Ensure the connection is not closed nor broken.
        if (revents & (POLLHUP | POLLERR | POLLNVAL)) {
            break;
        }

        // Receive a request packet. We ignore the endianness as unix socket
        // only allows communication on the same system anyway.
        RequestType req_type;
        struct
        {
            uint64_t start;
            uint64_t end;
        } request;
        if (!tryReadAll(&req_type, sizeof(req_type))) {
            break;
        }
        if (req_type != RequestType::kGetPhysRange) {
            warn("%s: receive unknown request: %d", name().c_str(),
                 static_cast<int>(req_type));
            break;
        }
        if (!tryReadAll(&request, sizeof(request))) {
            break;
        }
        AddrRange range(request.start, request.end);
        inform("%s: receive request: %s", name().c_str(),
               range.to_string().c_str());

        // Identify the backing store.
        const auto& stores = shmServer->system->getPhysMem().getBackingStore();
        auto it = std::find_if(
            stores.begin(), stores.end(), [&](const BackingStoreEntry& entry) {
                return entry.shmFd >= 0 && range.isSubset(entry.range);
            });
        if (it == stores.end()) {
            warn("%s: cannot find backing store for %s", name().c_str(),
                 range.to_string().c_str());
            break;
        }
        inform("%s: find shared backing store for %s at %s, shm=%d:%lld",
               name().c_str(), range.to_string().c_str(),
               it->range.to_string().c_str(), it->shmFd,
               (unsigned long long)it->shmOffset);

        // Populate response message.
        // mmap fd @ offset <===> [start, end] in simulated phys mem.
        msghdr msg = {};
        // Setup iovec for fields other than fd. We ignore the endianness as
        // unix socket only allows communication on the same system anyway.
        struct
        {
            off_t offset;
        } response;
        // (offset of the request range in shared memory) =
        //     (offset of the full range in shared memory) +
        //     (offset of the request range in the full range)
        response.offset = it->shmOffset + (range.start() - it->range.start());
        iovec ios = {.iov_base = &response, .iov_len = sizeof(response)};
        msg.msg_iov = &ios;
        msg.msg_iovlen = 1;
        // Setup fd as an ancillary data.
        union
        {
            char buf[CMSG_SPACE(sizeof(it->shmFd))];
            struct cmsghdr align;
        } cmsgs;
        msg.msg_control = cmsgs.buf;
        msg.msg_controllen = sizeof(cmsgs.buf);
        cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
        cmsg->cmsg_level = SOL_SOCKET;
        cmsg->cmsg_type = SCM_RIGHTS;
        cmsg->cmsg_len = CMSG_LEN(sizeof(it->shmFd));
        memcpy(CMSG_DATA(cmsg), &it->shmFd, sizeof(it->shmFd));
        // Send the response.
        int retv = sendmsg(pfd.fd, &msg, 0);
        if (retv < 0) {
            warn("%s: sendmsg failed: %s", name().c_str(), strerror(errno));
            break;
        }
        if (retv != sizeof(response)) {
            warn("%s: failed to send all response at once", name().c_str());
            break;
        }

        // Request done.
        inform("%s: request done", name().c_str());
        return;
    } while (false);

    // If we ever reach here, our client either close the connection or is
    // somehow broken. We'll just close the connection and move on.
    inform("%s: closing connection", name().c_str());
    close(pfd.fd);
    shmServer->clientSocketEvent.reset();
}

} // namespace memory
} // namespace gem5
