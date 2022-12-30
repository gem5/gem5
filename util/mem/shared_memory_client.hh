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

#ifndef __UTIL_MEM_SHARED_MEMORY_CLIENT_HH__
#define __UTIL_MEM_SHARED_MEMORY_CLIENT_HH__

#include <cinttypes>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <utility>

#include <err.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

namespace gem5
{
namespace util
{
namespace memory
{

class SharedMemoryClient
{
  public:
    enum RequestType : int
    {
        kGetPhysRange = 0
    };

    explicit SharedMemoryClient(const std::string& server_path);

    // Request to access the range [start, end] of physical memory from the
    // viewpoint of the gem5 system providing the shared memory service.
    // There is no guarantee that the physical memory range is not used by
    // others. It is the user's responsibility to make sure not to break other
    // services or IPs accessing the same range. For example, you might want to
    // configure the kernel running in gem5 simulator to reserve such range.
    void* MapMemory(uint64_t start, uint64_t end);

    // Unmap previous mapped region, no client is needed here.
    static bool UnmapMemory(void* mem);

  private:
    using AllocRecordStorage = std::unordered_map<void*, size_t>;

    int GetConnection();
    bool SendGetPhysRangeRequest(int sock_fd, uint64_t start, uint64_t end);
    bool RecvGetPhysRangeResponse(int sock_fd, int* ptr_fd, off_t* ptr_offset);
    void* DoMap(int shm_fd, off_t shm_offset, size_t size);

    bool SendAll(int sock_fd, const void* buffer, size_t size);

    static AllocRecordStorage& GetAllocRecordStorage();

    std::string server_path_;
};

inline SharedMemoryClient::SharedMemoryClient(const std::string& server_path)
    : server_path_(server_path)
{
}


inline void*
SharedMemoryClient::MapMemory(uint64_t start, uint64_t end)
{
    void* mem = nullptr;
    int sock_fd = -1;
    int shm_fd = -1;
    off_t shm_offset;

    do {
        if (start > end) {
            warnx("invalid range %" PRIu64 "-%" PRIu64, start, end);
            break;
        }
        sock_fd = GetConnection();
        if (sock_fd < 0) {
            warnx("cannot connect to shared memory server");
            break;
        }
        if (!SendGetPhysRangeRequest(sock_fd, start, end)) {
            warnx("cannot send request to shared memory server");
            break;
        }
        if (!RecvGetPhysRangeResponse(sock_fd, &shm_fd, &shm_offset)) {
            warnx("failed to read shared memory server response");
            break;
        }
        mem = DoMap(shm_fd, shm_offset, end - start + 1);
        if (mem == nullptr) {
            warnx("failed to create memory mapping");
            break;
        }
    } while (false);

    if (sock_fd >= 0) {
        close(sock_fd);
    }
    if (shm_fd >= 0) {
        close(shm_fd);
    }

    return mem;
}

inline bool
SharedMemoryClient::UnmapMemory(void* mem)
{
    auto& storage = GetAllocRecordStorage();
    auto it = storage.find(mem);
    if (it == storage.end()) {
        return false;
    }
    if (munmap(mem, it->second) < 0) {
        warn("munmap failed");
        return false;
    }
    storage.erase(it);
    return true;
}

inline int
SharedMemoryClient::GetConnection()
{
    int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        warn("create unix socket failed");
        return -1;
    }

    sockaddr_un serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sun_family = AF_UNIX;
    strncpy(serv_addr.sun_path, server_path_.c_str(),
            sizeof(serv_addr.sun_path) - 1);
    if (strlen(serv_addr.sun_path) != server_path_.size()) {
        warnx("server address truncated");
        close(sock_fd);
        return -1;
    }
    if (connect(sock_fd, reinterpret_cast<sockaddr*>(&serv_addr),
                sizeof(serv_addr)) < 0) {
        warn("connect failed");
        close(sock_fd);
        return -1;
    }
    return sock_fd;
}

inline bool
SharedMemoryClient::SendGetPhysRangeRequest(int sock_fd, uint64_t start,
                                            uint64_t end)
{
    int req_type = RequestType::kGetPhysRange;
    struct
    {
        uint64_t start;
        uint64_t end;
    } request = {start, end};
    return SendAll(sock_fd, &req_type, sizeof(req_type)) &&
           SendAll(sock_fd, &request, sizeof(request));
}

inline bool
SharedMemoryClient::RecvGetPhysRangeResponse(int sock_fd, int* ptr_fd,
                                             off_t* ptr_offset)
{
    if (!ptr_fd || !ptr_offset) {
        return false;
    }

    msghdr msg = {};
    // Setup ptr_offset as buffer.
    iovec io = {.iov_base = ptr_offset, .iov_len = sizeof(*ptr_offset)};
    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    // Setup buffer for fd.
    union
    {
        char buffer[CMSG_SPACE(sizeof(*ptr_fd))];
        struct cmsghdr align;
    } cmsgs;
    msg.msg_control = cmsgs.buffer;
    msg.msg_controllen = sizeof(cmsgs.buffer);
    cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(*ptr_fd));
    // Try receive the message.
    ssize_t retv = recvmsg(sock_fd, &msg, 0);
    if (retv < 0) {
        warn("recvmsg failed");
        return false;
    }
    if (retv != sizeof(*ptr_offset)) {
        warnx("cannot receive all response");
        return false;
    }
    memcpy(ptr_fd, CMSG_DATA(cmsg), sizeof(*ptr_fd));
    return true;
}

inline void*
SharedMemoryClient::DoMap(int shm_fd, off_t shm_offset, size_t size)
{
    void* mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd,
                     shm_offset);
    if (mem == MAP_FAILED) {
        warn("mmap failed");
        return nullptr;
    }
    // If we cannot record a new mapping, our mapping are probably corrupted.
    if (!GetAllocRecordStorage().emplace(mem, size).second) {
        errx(EXIT_FAILURE, "cannot register memory mapping!");
    }
    return mem;
}

inline bool
SharedMemoryClient::SendAll(int sock_fd, const void* buffer, size_t size)
{
    const char* char_buffer = reinterpret_cast<const char*>(buffer);
    for (size_t offset = 0; offset < size;) {
        ssize_t retv = send(sock_fd, char_buffer + offset, size - offset, 0);
        if (retv >= 0) {
            offset += retv;
        } else if (errno != EINTR) {
            warn("send failed");
            return false;
        }
    }
    return true;
}

inline SharedMemoryClient::AllocRecordStorage&
SharedMemoryClient::GetAllocRecordStorage()
{
    static auto storage = new SharedMemoryClient::AllocRecordStorage();
    return *storage;
}

} // namespace memory
} // namespace util
} // namespace gem5

#endif  // __UTIL_MEM_SHARED_MEMORY_CLIENT_HH__
