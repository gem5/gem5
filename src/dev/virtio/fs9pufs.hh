/*
 * Copyright (c) 2014-2025 ARM Limited
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

#ifndef __DEV_VIRTIO_FSUFS_HH__
#define __DEV_VIRTIO_FSUFS_HH__

#include <map>
#include <memory>
#include <string>

extern "C" {
#include <npfs.h>
#include <ufs.h>
}

#include "dev/virtio/fs9p.hh"

namespace gem5
{

struct VirtIO9PUfsParams;

/**
 * VirtIO 9p proxy that communicates with the libufs 9p server using
 * pipes.
 */
class VirtIO9PUfs : public VirtIO9PProxy
{
  public:
    typedef VirtIO9PUfsParams Params;
    VirtIO9PUfs(const Params &params);
    virtual ~VirtIO9PUfs();

    void startup();
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    /**
     * Start the embedded ufs server and setup the communication pipes.
     */
    void start();

    ssize_t read(uint8_t *data, size_t len);
    ssize_t write(const uint8_t *data, size_t len);
    void terminate();

  private:
    class UfsDataEvent : public PollEvent
    {
      public:
        UfsDataEvent(VirtIO9PUfs &_parent, int fd, int event)
            : PollEvent(fd, event), parent(_parent)
        {}

        virtual ~UfsDataEvent() {}

        void process(int revent);

      private:
        VirtIO9PUfs &parent;
    };

    /** fd for data pipe going to ufs (write end) */
    int fd_to_ufs;
    /** fd for data pipe coming from ufs (read end) */
    int fd_from_ufs;

    std::unique_ptr<UfsDataEvent> dataEvent;
    Npsrv *srv;
};

} // namespace gem5

#endif // __DEV_VIRTIO_FSUFS_HH__
