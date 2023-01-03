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

#ifndef __MEM_SHARED_MEMORY_SERVER_HH__
#define __MEM_SHARED_MEMORY_SERVER_HH__

#include <memory>
#include <string>
#include <unordered_map>

#include "base/pollevent.hh"
#include "base/socket.hh"
#include "params/SharedMemoryServer.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{
namespace memory
{

class SharedMemoryServer : public SimObject
{
  public:
    enum class RequestType : int
    {
        kGetPhysRange = 0,
    };

    explicit SharedMemoryServer(const SharedMemoryServerParams& params);
    ~SharedMemoryServer();

  private:
    class BaseShmPollEvent : public PollEvent
    {
      public:
        BaseShmPollEvent(int fd, SharedMemoryServer* shm_server);

        const std::string& name() const;

      protected:
        bool tryReadAll(void* buffer, size_t size);

        SharedMemoryServer* shmServer;
        std::string eventName;
    };

    class ListenSocketEvent : public BaseShmPollEvent
    {
      public:
        using BaseShmPollEvent::BaseShmPollEvent;
        void process(int revent) override;
    };

    class ClientSocketEvent : public BaseShmPollEvent
    {
      public:
        using BaseShmPollEvent::BaseShmPollEvent;
        void process(int revent) override;
    };

    UnixSocketAddr sockAddr;
    System* system;

    int serverFd;
    std::unique_ptr<ListenSocketEvent> listenSocketEvent;
    std::unordered_map<int, std::unique_ptr<ClientSocketEvent>>
        clientSocketEvents;
};

} // namespace memory
} // namespace gem5

#endif  // __MEM_SHARED_MEMORY_SERVER_HH__
