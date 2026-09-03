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

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>

#include "base/callback.hh"
#include "base/output.hh"
#include "debug/VIO9P.hh"
#include "debug/VIO9PData.hh"
#include "debug/VIO9PUfs.hh"
#include "params/VirtIO9PBase.hh"
#include "params/VirtIO9PProxy.hh"
#include "params/VirtIO9PSocket.hh"
#include "params/VirtIO9PUfs.hh"
#include "sim/core.hh"
#include "sim/system.hh"

#include "dev/virtio/fs9pufs.hh"

namespace gem5
{

VirtIO9PUfs::VirtIO9PUfs(const Params &params)
    : VirtIO9PProxy(params), fd_to_ufs(-1), fd_from_ufs(-1), srv(nullptr)
{}

VirtIO9PUfs::~VirtIO9PUfs()
{}

void
VirtIO9PUfs::startup()
{
    DPRINTF(VIO9P, "VirtIO9PUfs: startup\n");
    start();
    dataEvent.reset(new UfsDataEvent(*this, fd_from_ufs, POLLIN));
    pollQueue.schedule(dataEvent.get());
}

void
VirtIO9PUfs::start()
{
    const Params &p = dynamic_cast<const Params &>(params());

    if (srv != nullptr) {
        DPRINTF(VIO9P, "VirtIO9PUfs: already started\n");
        return;
    }

    srv = ufs_start(const_cast<char *>(p.root.c_str()),
                    debug::VIO9PUfs ? 1 : 0, 2, 1, 0);
    panic_if(srv == nullptr, "Failed to start the 9P server");
    ufs_get_fds(srv, &fd_from_ufs, &fd_to_ufs);
}

ssize_t
VirtIO9PUfs::read(uint8_t *data, size_t len)
{
    assert(fd_from_ufs != -1);
    const int ret(::read(fd_from_ufs, static_cast<void *>(data), len));
    return ret < 0 ? -errno : ret;
}

ssize_t
VirtIO9PUfs::write(const uint8_t *data, size_t len)
{
    assert(fd_to_ufs != -1);
    const int ret(::write(fd_to_ufs, static_cast<const void *>(data), len));
    return ret < 0 ? -errno : ret;
}

void
VirtIO9PUfs::UfsDataEvent::process(int revent)
{
    parent.serverDataReady();
}

void
VirtIO9PUfs::terminate()
{
    if (fd_to_ufs != -1) {
        close(fd_to_ufs);
        fd_to_ufs = -1;
    }
    if (fd_from_ufs != -1) {
        close(fd_from_ufs);
        fd_from_ufs = -1;
    }
}

void
VirtIO9PUfs::serialize(CheckpointOut &cp) const
{
    int datalen;
    char *data;

    datalen = ufs_checkpoint(srv, reinterpret_cast<void **>(&data));
    panic_if(datalen < 0, "VirtIO9PUfs: serialize failed");

    SERIALIZE_SCALAR(datalen);
    SERIALIZE_ARRAY(data, datalen);
    free(data);
    VirtIO9PBase::serialize(cp);
}

void
VirtIO9PUfs::unserialize(CheckpointIn &cp)
{
    int datalen;
    char *data;
    char err[256];

    start();
    UNSERIALIZE_SCALAR(datalen);
    data = static_cast<char *>(malloc(datalen));
    UNSERIALIZE_ARRAY(data, datalen);
    if (ufs_restore(srv, data, datalen, err, sizeof(err)) < 0) {
        warn("VirtIO9PUfs: restore failed: %s", err);
    }
    free(data);

    VirtIO9PBase::unserialize(cp);
}

} // namespace gem5
