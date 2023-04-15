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
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#if (defined(__GNUC__) && (__GNUC__ >= 8)) || defined(__clang__)
    #include <filesystem>
#else
    // This is only reachable if we're using GCC 7 (note: gem5 does not support
    // GCC versions older than GCC 7 as they do not support the C++17
    // standard).
    // If we're using GCC 7, we need to use <experimental/filesystem>.
    #include <experimental/filesystem>
    namespace std {
        namespace filesystem = experimental::filesystem;
    }
#endif

#include "base/logging.hh"
#include "base/output.hh"
#include "base/pollevent.hh"

namespace gem5
{
namespace memory
{

namespace
{

ListenSocketPtr
buildListenSocket(const std::string &path, const std::string &name)
{
    fatal_if(path.empty(), "%s: Empty socket path", name);
    if (path[0] == '@')
        return listenSocketUnixAbstractConfig(path.substr(1)).build(name);

    std::filesystem::path p(path);
    return listenSocketUnixFileConfig(
            p.parent_path(), p.filename()).build(name);
}

} // anonymous namespace

SharedMemoryServer::SharedMemoryServer(const SharedMemoryServerParams& params)
    : SimObject(params),
      system(params.system),
      listener(buildListenSocket(params.server_path, name()))
{
    fatal_if(system == nullptr, "Requires a system to share memory from!");
    listener->listen();

    listenSocketEvent.reset(new ListenSocketEvent(listener->getfd(), this));
    pollQueue.schedule(listenSocketEvent.get());
    inform("%s: listening at %s", name(), *listener);
}

SharedMemoryServer::~SharedMemoryServer() {}

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
            warn("%s: recv failed: %s", name(), strerror(errno));
            return false;
        }
    }
    return true;
}

void
SharedMemoryServer::ListenSocketEvent::process(int revents)
{
    int cli_fd = shmServer->listener->accept();
    inform("%s: accept new connection %d", name(), cli_fd);
    shmServer->clientSocketEvents[cli_fd].reset(
        new ClientSocketEvent(cli_fd, shmServer));
    pollQueue.schedule(shmServer->clientSocketEvents[cli_fd].get());
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
            warn("%s: receive unknown request: %d", name(),
                 static_cast<int>(req_type));
            break;
        }
        if (!tryReadAll(&request, sizeof(request))) {
            break;
        }
        AddrRange range(request.start, request.end);
        inform("%s: receive request: %s", name(), range.to_string());

        // Identify the backing store.
        const auto& stores = shmServer->system->getPhysMem().getBackingStore();
        auto it = std::find_if(
            stores.begin(), stores.end(), [&](const BackingStoreEntry& entry) {
                return entry.shmFd >= 0 && range.isSubset(entry.range);
            });
        if (it == stores.end()) {
            warn("%s: cannot find backing store for %s", name(),
                 range.to_string());
            break;
        }
        inform("%s: find shared backing store for %s at %s, shm=%d:%lld",
               name(), range.to_string(), it->range.to_string(), it->shmFd,
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
            warn("%s: sendmsg failed: %s", name(), strerror(errno));
            break;
        }
        if (retv != sizeof(response)) {
            warn("%s: failed to send all response at once", name());
            break;
        }

        // Request done.
        inform("%s: request done", name());
        return;
    } while (false);

    // If we ever reach here, our client either close the connection or is
    // somehow broken. We'll just close the connection and move on.
    inform("%s: closing connection", name());
    close(pfd.fd);
    shmServer->clientSocketEvents.erase(pfd.fd);
}

} // namespace memory
} // namespace gem5
