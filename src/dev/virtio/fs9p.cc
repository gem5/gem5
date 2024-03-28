/*
 * Copyright (c) 2014-2017 ARM Limited
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

#include "dev/virtio/fs9p.hh"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstring>
#include <fstream>

#include "base/callback.hh"
#include "base/output.hh"
#include "debug/VIO9P.hh"
#include "debug/VIO9PData.hh"
#include "params/VirtIO9PBase.hh"
#include "params/VirtIO9PDiod.hh"
#include "params/VirtIO9PProxy.hh"
#include "params/VirtIO9PSocket.hh"
#include "sim/core.hh"
#include "sim/system.hh"

namespace gem5
{

struct P9MsgInfo
{
    P9MsgInfo(P9MsgType _type, std::string _name)
        : type(_type), name(_name) {}

    P9MsgType type;
    std::string name;
};

typedef std::map<P9MsgType, P9MsgInfo> P9MsgInfoMap;

/* clang-format off */
#define P9MSG(type, name)                               \
    { (type), P9MsgInfo((type), "T" # name ) },         \
    { (type + 1), P9MsgInfo((type + 1), "R" # name ) }

static const P9MsgInfoMap p9_msg_info {
    P9MSG(6, LERROR),
    P9MSG(8, STATFS),
    P9MSG(12, LOPEN),
    P9MSG(14, LCREATE),
    P9MSG(16, SYMLINK),
    P9MSG(18, MKNOD),
    P9MSG(20, RENAME),
    P9MSG(22, READLINK),
    P9MSG(24, GETATTR),
    P9MSG(26, SETATTR),
    P9MSG(30, XATTRWALK),
    P9MSG(32, XATTRCREATE),
    P9MSG(40, READDIR),
    P9MSG(50, FSYNC),
    P9MSG(52, LOCK),
    P9MSG(54, GETLOCK),
    P9MSG(70, LINK),
    P9MSG(72, MKDIR),
    P9MSG(74, RENAMEAT),
    P9MSG(76, UNLINKAT),
    P9MSG(100, VERSION),
    P9MSG(102, AUTH),
    P9MSG(104, ATTACH),
    P9MSG(106, ERROR),
    P9MSG(108, FLUSH),
    P9MSG(110, WALK),
    P9MSG(112, OPEN),
    P9MSG(114, CREATE),
    P9MSG(116, READ),
    P9MSG(118, WRITE),
    P9MSG(120, CLUNK),
    P9MSG(122, REMOVE),
    P9MSG(124, STAT),
    P9MSG(126, WSTAT),
};

#undef P9MSG
/* clang-format on */

VirtIO9PBase::VirtIO9PBase(const Params &params)
    : VirtIODeviceBase(params, ID_9P,
                       sizeof(Config) + params.tag.size(),
                       F_MOUNT_TAG),
      queue(params.system->physProxy, byteOrder, params.queueSize, *this)
{
    config.reset((Config *)
                 operator new(configSize));
    config->len = htog(params.tag.size(), byteOrder);
    memcpy(config->tag, params.tag.c_str(), params.tag.size());

    registerQueue(queue);
}


VirtIO9PBase::~VirtIO9PBase()
{
}

void
VirtIO9PBase::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    readConfigBlob(pkt, cfgOffset, (uint8_t *)config.get());
}

void
VirtIO9PBase::FSQueue::onNotifyDescriptor(VirtDescriptor *desc)
{
    DPRINTF(VIO9P, "Got input data descriptor (len: %i)\n", desc->size());
    DPRINTF(VIO9P, "\tPending transactions: %i\n", parent.pendingTransactions.size());

    P9MsgHeader header;
    desc->chainRead(0, (uint8_t *)&header, sizeof(header));
    header = p9toh(header);

    uint8_t data[header.len - sizeof(header)];
    desc->chainRead(sizeof(header), data, sizeof(data));

    // Keep track of pending transactions
    parent.pendingTransactions[header.tag] = desc;

    DPRINTF(VIO9P, "recvTMsg\n");
    parent.dumpMsg(header, data, sizeof(data));

    // Notify device of message
    parent.recvTMsg(header, data, sizeof(data));
}

void
VirtIO9PBase::sendRMsg(const P9MsgHeader &header, const uint8_t *data, size_t size)
{
    DPRINTF(VIO9P, "Sending RMsg\n");
    dumpMsg(header, data, size);
    DPRINTF(VIO9P, "\tPending transactions: %i\n", pendingTransactions.size());
    assert(header.len >= sizeof(header));

    VirtDescriptor *main_desc(pendingTransactions[header.tag]);
    pendingTransactions.erase(header.tag);

    // Find the first output descriptor
    VirtDescriptor *out_desc(main_desc);
    while (out_desc && !out_desc->isOutgoing())
        out_desc = out_desc->next();
    if (!out_desc)
        panic("sendRMsg: Framing error, no output descriptor.\n");

    P9MsgHeader header_out(htop9(header));
    header_out.len = htop9(sizeof(P9MsgHeader) + size);

    out_desc->chainWrite(0, (uint8_t *)&header_out, sizeof(header_out));
    out_desc->chainWrite(sizeof(header_out), data, size);

    queue.produceDescriptor(main_desc, sizeof(P9MsgHeader) + size);
    kick();
}

void
VirtIO9PBase::dumpMsg(const P9MsgHeader &header, const uint8_t *data, size_t size)
{
#ifndef NDEBUG
    if (!debug::VIO9P)
        return;

    const P9MsgInfoMap::const_iterator it_msg(p9_msg_info.find(header.type));
    if (it_msg != p9_msg_info.cend()) {
        const P9MsgInfo &info(it_msg->second);
        DPRINTF(VIO9P, "P9Msg[len = %i, type = %s (%i), tag = %i]\n",
                header.len, info.name, header.type, header.tag);
    } else {
        DPRINTF(VIO9P, "P9Msg[len = %i, type = Unknown (%i), tag = %i]\n",
                header.len, header.type, header.tag);
    }
    DDUMP(VIO9PData, data, size);
#endif
}


VirtIO9PProxy::VirtIO9PProxy(const Params &params)
  : VirtIO9PBase(params), deviceUsed(false)
{
}

VirtIO9PProxy::~VirtIO9PProxy()
{
}


void
VirtIO9PProxy::serialize(CheckpointOut &cp) const
{
    if (deviceUsed) {
        warn("Serializing VirtIO9Base device after device has been used. It is "
             "likely that state will be lost, and that the device will cease "
             "to work!");
    }
    SERIALIZE_SCALAR(deviceUsed);

    VirtIO9PBase::serialize(cp);
}

void
VirtIO9PProxy::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(deviceUsed);

    if (deviceUsed) {
        warn("Unserializing VirtIO9Base device after device has been used. It is "
             "likely that state has been lost, and that the device will cease "
             "to work!");
    }
    VirtIO9PBase::unserialize(cp);
}


void
VirtIO9PProxy::recvTMsg(const P9MsgHeader &header,
                        const uint8_t *data, size_t size)
{
    deviceUsed = true;
    assert(header.len == sizeof(header) + size);
    // While technically not needed, we send the packet as one
    // contiguous segment to make some packet dissectors happy.
    uint8_t out[header.len];
    P9MsgHeader header_out(htop9(header));
    memcpy(out, (uint8_t *)&header_out, sizeof(header_out));
    memcpy(out + sizeof(header_out), data, size);
    writeAll(out, sizeof(header_out) + size);
}

void
VirtIO9PProxy::serverDataReady()
{
    P9MsgHeader header;
    readAll((uint8_t *)&header, sizeof(header));
    header = p9toh(header);

    const ssize_t payload_len(header.len - sizeof(header));
    if (payload_len < 0)
        panic("Payload length is negative!\n");
    uint8_t data[payload_len];
    readAll(data, payload_len);

    sendRMsg(header, data, payload_len);
}


void
VirtIO9PProxy::readAll(uint8_t *data, size_t len)
{
    while (len) {
        ssize_t ret;
        while ((ret = read(data, len)) == -EAGAIN)
            ;
        if (ret < 0)
            panic("readAll: Read failed: %i\n", -ret);

        len -= ret;
        data += ret;
    }
}

void
VirtIO9PProxy::writeAll(const uint8_t *data, size_t len)
{
    while (len) {
        ssize_t ret;
        while ((ret = write(data, len)) == -EAGAIN)
            ;
        if (ret < 0)
            panic("writeAll: write failed: %i\n", -ret);

        len -= ret;
        data += ret;
    }
}



VirtIO9PDiod::VirtIO9PDiod(const Params &params)
    : VirtIO9PProxy(params),
      fd_to_diod(-1), fd_from_diod(-1), diod_pid(-1)
{
    // Register an exit callback so we can kill the diod process
    registerExitCallback([this]() { terminateDiod(); });
}

VirtIO9PDiod::~VirtIO9PDiod()
{
}

void
VirtIO9PDiod::startup()
{
    startDiod();
    dataEvent.reset(new DiodDataEvent(*this, fd_from_diod, POLLIN));
    pollQueue.schedule(dataEvent.get());
}

void
VirtIO9PDiod::startDiod()
{
    const Params &p = dynamic_cast<const Params &>(params());
    int pipe_rfd[2];
    int pipe_wfd[2];

    DPRINTF(VIO9P, "Using diod at %s.\n", p.diod);

    panic_if(pipe(pipe_rfd) == -1, "Failed to create DIOD read pipe: %s",
            strerror(errno));
    panic_if(pipe(pipe_wfd) == -1, "Failed to create DIOD write pipe: %s",
            strerror(errno));

    fd_to_diod = pipe_rfd[1];
    fd_from_diod = pipe_wfd[0];

    // Create Unix domain socket
    int socket_id = socket(AF_UNIX, SOCK_STREAM, 0);
    panic_if(socket_id == -1, "Socket creation failed %i", errno);

    // Bind the socket to a path which will not be read
    struct sockaddr_un socket_address;
    memset(&socket_address, 0, sizeof(socket_address));
    socket_address.sun_family = AF_UNIX;

    const std::string socket_path = simout.resolve(p.socketPath);
    fatal_if(!OutputDirectory::isAbsolute(socket_path), "Please make the"
             " output directory an absolute path, else diod will fail!");

    // Prevent overflow in strcpy
    fatal_if(sizeof(socket_address.sun_path) <= socket_path.length(),
             "Incorrect length of socket path");
    strncpy(socket_address.sun_path, socket_path.c_str(),
            sizeof(socket_address.sun_path) - 1);
    panic_if(bind(socket_id, (struct sockaddr*)&socket_address,
                sizeof(socket_address)) == -1,
            "Socket binding to %i failed - most likely the output dir "
            "and hence unused socket already exists.", socket_id);

    diod_pid = fork();
    panic_if(diod_pid == -1, "Fork failed: %s", strerror(errno));

    if (diod_pid == 0) {
        // Create the socket which will later by used by the diod process
        close(STDIN_FILENO);
        close(pipe_rfd[1]);
        close(pipe_wfd[0]);

        auto diod_rfd_s = std::to_string(pipe_rfd[0]);
        auto diod_wfd_s = std::to_string(pipe_wfd[1]);

        // Start diod
        execlp(p.diod.c_str(), p.diod.c_str(),
               "-d", debug::VIO9P ? "1" : "0", // show debug output
               "-f", // start in foreground
               "-r", diod_rfd_s.c_str(), // setup read FD
               "-w", diod_wfd_s.c_str(), // setup write FD
               "-e", p.root.c_str(), // path to export
               "-n", // disable security
               "-S", // squash all users
               "-l", socket_path.c_str(), // pass the socket
               nullptr);
        panic("Failed to execute diod to %s: %s", socket_path,
                strerror(errno));
    } else {
        close(pipe_rfd[0]);
        close(pipe_wfd[1]);
        inform("Started diod with PID %u, you might need to manually kill "
                "diod if gem5 crashes", diod_pid);
    }
}

ssize_t
VirtIO9PDiod::read(uint8_t *data, size_t len)
{
    assert(fd_from_diod != -1);
    const int ret(::read(fd_from_diod, (void *)data, len));
    return ret < 0 ? -errno : ret;
}

ssize_t
VirtIO9PDiod::write(const uint8_t *data, size_t len)
{
    assert(fd_to_diod != -1);
    const int ret(::write(fd_to_diod, (const void *)data, len));
    return ret < 0 ? -errno : ret;
}

void
VirtIO9PDiod::DiodDataEvent::process(int revent)
{
    parent.serverDataReady();
}

void
VirtIO9PDiod::terminateDiod()
{
    assert(diod_pid != -1);

    DPRINTF(VIO9P, "Trying to kill diod at pid %u \n", diod_pid);

    if (kill(diod_pid, SIGTERM) != 0) {
        perror("Killing diod process");
        warn("Failed to kill diod using SIGTERM");
        return;
    }

    // Check if kill worked
    for (unsigned i = 0; i < 5; i++) {
        int wait_return = waitpid(diod_pid, NULL, WNOHANG);
        if (wait_return == diod_pid) {
            // Managed to kill diod
            return;
        } else if (wait_return == 0) {
            // Diod is not killed so sleep and try again
            usleep(500);
        } else {
            // Failed in waitpid
            perror("Waitpid");
            warn("Failed in waitpid");
        }
    }

    // Try again to kill diod with sigkill
    inform("Trying to kill diod with SIGKILL as SIGTERM failed \n");
    if (kill(diod_pid, SIGKILL) != 0) {
        perror("Killing diod process");
        warn("Failed to kill diod using SIGKILL");
    } else {
        // Managed to kill diod
        return;
    }
}


VirtIO9PSocket::VirtIO9PSocket(const Params &params)
    : VirtIO9PProxy(params), fdSocket(-1)
{
}

VirtIO9PSocket::~VirtIO9PSocket()
{
}

void
VirtIO9PSocket::startup()
{
    connectSocket();
    dataEvent.reset(new SocketDataEvent(*this, fdSocket, POLLIN));
    pollQueue.schedule(dataEvent.get());
}

void
VirtIO9PSocket::connectSocket()
{
    const Params &p = dynamic_cast<const Params &>(params());

    int ret;
    struct addrinfo hints, *result;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = 0;

    if ((ret = getaddrinfo(p.server.c_str(), p.port.c_str(),
                           &hints, &result)) != 0)
        panic("getaddrinfo: %s\n", gai_strerror(ret));

    DPRINTF(VIO9P, "Connecting to 9p server '%s'.\n", p.server);
    for (struct addrinfo *rp = result; rp; rp = rp->ai_next) {
        fdSocket = socket(rp->ai_family, rp->ai_socktype,
                     rp->ai_protocol);
        if (fdSocket == -1) {
            continue;
        } else if (connect(fdSocket, rp->ai_addr, rp->ai_addrlen) != -1) {
            break;
        } else {
            close(fdSocket);
            fdSocket = -1;
        }
    }

    freeaddrinfo(result);

    if (fdSocket == -1)
        panic("Failed to connect to 9p server (%s:%s)", p.server, p.port);
}

void
VirtIO9PSocket::socketDisconnect()
{
    panic("9P Socket disconnected!\n");
}

ssize_t
VirtIO9PSocket::read(uint8_t *data, size_t len)
{
    assert(fdSocket != -1);
    int ret;

    ret = ::recv(fdSocket, (void *)data, len, 0);
    if (ret == 0)
        socketDisconnect();

    return ret < 0 ? -errno : ret;
}

ssize_t
VirtIO9PSocket::write(const uint8_t *data, size_t len)
{
    assert(fdSocket != -1);
    int ret(::send(fdSocket, (const void *)data, len, 0));
    return ret < 0 ? -errno : ret;
}

void
VirtIO9PSocket::SocketDataEvent::process(int revent)
{
    parent.serverDataReady();
}

} // namespace gem5
