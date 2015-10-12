/*
 * Copyright (c) 2014 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __DEV_VIRTIO_FS9P_HH__
#define __DEV_VIRTIO_FS9P_HH__

#include <map>
#include <memory>
#include <string>

#include "base/pollevent.hh"
#include "dev/virtio/base.hh"

struct VirtIO9PBaseParams;

typedef uint8_t P9MsgType;
typedef uint16_t P9Tag;

struct P9MsgHeader {
    /** Length including header */
    uint32_t len;
    /** Message type */
    P9MsgType type;
    /** Message tag */
    P9Tag tag;
} M5_ATTR_PACKED;

/** Convert p9 byte order (LE) to host byte order */
template <typename T> inline T
p9toh(T v) { return letoh(v); }

/** Convert host byte order to p9 byte order (LE) */
template <typename T> inline T
htop9(T v) { return htole(v); }

template <> inline P9MsgHeader
p9toh(P9MsgHeader v)
{
    v.len = p9toh(v.len);
    v.type = p9toh(v.type);
    v.tag = p9toh(v.tag);
    return v;
}

template <> inline P9MsgHeader
htop9(P9MsgHeader v)
{
    v.len = htop9(v.len);
    v.type = htop9(v.type);
    v.tag = htop9(v.tag);
    return v;
}

/**
 * This class implements a VirtIO transport layer for the 9p network
 * file system.
 *
 * The 9p VirtIO transport uses the following queues:
 *  -# 9p requests and replies
 *
 * Each 9p request and response is sent in its own descriptor
 * chain. The guest initiates a transaction by packing a T message
 * (see the 9p spec) into the first part of a descriptor chain. After
 * the T message, the guest reserves space for the reply (R message)
 * by including one or more writable descriptors. The server replies
 * by writing an R message into the writable descriptors and putting
 * the chain in the used ring (VirtQueue::produceDescriptor()).
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see https://github.com/ericvh/9p-rfc
 * @see https://code.google.com/p/diod/wiki/protocol
 */
class VirtIO9PBase : public VirtIODeviceBase
{
  public:
    typedef VirtIO9PBaseParams Params;
    VirtIO9PBase(Params *params);
    virtual ~VirtIO9PBase();

    void readConfig(PacketPtr pkt, Addr cfgOffset);

  protected:
    /**
     * VirtIO 9p configuration structure
     *
     * @note The fields in this structure depend on the features
     * exposed to the guest.
     */
    struct Config {
        uint16_t len;
        char tag[];
    } M5_ATTR_PACKED;

    /** Currently active configuration (host byte order) */
    std::unique_ptr<Config> config;

    /** VirtIO device ID */
    static const DeviceId ID_9P = 0x09;

    /** @{
     * @name Feature bits
     */
    /** Device provides a name of the resource in its configuration */
    static const FeatureBits F_MOUNT_TAG = 0x01;
    /** @} */

  protected:
    /**
     * Virtqueue for 9p requests
     */
    class FSQueue : public VirtQueue
    {
      public:
        FSQueue(PortProxy &proxy, uint16_t size, VirtIO9PBase &_parent)
            : VirtQueue(proxy, size), parent(_parent) {}
        virtual ~FSQueue() {}

        void onNotifyDescriptor(VirtDescriptor *desc);

        std::string name() const { return parent.name() + ".queue"; }

      protected:
        VirtIO9PBase &parent;
    };

    FSQueue queue;

  protected:
    /**
     * Handle incoming 9p RPC message.
     *
     * @param header 9p message header.
     * @param data Pointer to data in message.
     * @param size Size of data (excluding header)
     */
    virtual void recvTMsg(const P9MsgHeader &header, const uint8_t *data, size_t size) = 0;
    /**
     * Send a 9p RPC message reply.
     *
     * @param header 9p message header.
     * @param data Pointer to data in message.
     * @param size Size of data (excluding header)
     */
    void sendRMsg(const P9MsgHeader &header, const uint8_t *data, size_t size);

    /**
     * Dump a 9p RPC message on the debug output
     *
     * @param header 9p message header.
     * @param data Pointer to data in message.
     * @param size Size of data (excluding header)
     */
    void dumpMsg(const P9MsgHeader &header, const uint8_t *data, size_t size);

  private:
    /**
     * Map between 9p transaction tags and descriptors where they
     * appeared.
     *
     * When handling asynchronous requests, we need to ensure that
     * replies are posted in the same descriptor as queries. The 9p
     * RPC protocol uses the tag field in the header to match requests
     * and replies, which we use here to find the relevant descriptor.
     */
    std::map<P9Tag, VirtDescriptor *> pendingTransactions;
};

struct VirtIO9PProxyParams;

/**
 * VirtIO 9p proxy base class.
 *
 * This base class provides basic functionality shared by different 9p
 * proxy implementations.
 */
class VirtIO9PProxy : public VirtIO9PBase
{
  public:
    typedef VirtIO9PProxyParams Params;
    VirtIO9PProxy(Params *params);
    virtual ~VirtIO9PProxy();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    void recvTMsg(const P9MsgHeader &header, const uint8_t *data,
                  size_t size) override;

    /** Notification of pending data from server */
    void serverDataReady();

    /**
     * Read data from the server behind the proxy.
     *
     * @note This method may return read fewer than len bytes.
     *
     * @param data Memory location to store results in.
     * @param len Maximum length to read.
     * @return Number of bytes read, -errno on failure.
     */
    virtual ssize_t read(uint8_t *data, size_t len) = 0;
    /**
     * Write data to the server behind the proxy.
     *
     * @note This method may return write fewer than len bytes.
     *
     * @param data Pointer to data to write.
     * @param len Maximum length to write.
     * @return Number of bytes written, -errno on failure.
     */
    virtual ssize_t write(const uint8_t *data, size_t len) = 0;

    /**
     * Convenience function that reads exactly len bytes.
     *
     * This method calls read until exactly len number of bytes has
     * been read. A read() call is retried if the underlying syscall
     * was interrupted.
     *
     * @param data Memory location to store results in.
     * @param len Number of bytes to read.
     */
    void readAll(uint8_t *data, size_t len);
    /**
     * Convenience function that writes exactly len bytes.
     *
     * This method calls write until exactly len number of bytes has
     * been written. A write() call is retried if the underlying
     * syscall was interrupted.
     *
     * @param data Data to write.
     * @param len Number of bytes to write.
     */
    void writeAll(const uint8_t *data, size_t len);
};

struct VirtIO9PDiodParams;

/**
 * VirtIO 9p proxy that communicates with the diod 9p server using
 * pipes.
 */
class VirtIO9PDiod : public VirtIO9PProxy
{
  public:
    typedef VirtIO9PDiodParams Params;
    VirtIO9PDiod(Params *params);
    virtual ~VirtIO9PDiod();

    void startup();

  protected:
    /**
     * Start diod and setup the communication pipes.
     */
    void startDiod();

    ssize_t read(uint8_t *data, size_t len);
    ssize_t write(const uint8_t *data, size_t len);

  private:
    class DiodDataEvent : public PollEvent
    {
      public:
        DiodDataEvent(VirtIO9PDiod &_parent, int fd, int event)
            : PollEvent(fd, event), parent(_parent) {}

        virtual ~DiodDataEvent() {};

        void process(int revent);

      private:
        VirtIO9PDiod &parent;
    };

    /** fd for data pipe going to diod (write end) */
    int fd_to_diod;
    /** fd for data pipe coming from diod (read end) */
    int fd_from_diod;

    std::unique_ptr<DiodDataEvent> dataEvent;

    /** PID of diod process */
    int diod_pid;
};

struct VirtIO9PSocketParams;

/**
 * VirtIO 9p proxy that communicates with a 9p server over tcp
 * sockets.
 */
class VirtIO9PSocket : public VirtIO9PProxy
{
  public:
    typedef VirtIO9PSocketParams Params;
    VirtIO9PSocket(Params *params);
    virtual ~VirtIO9PSocket();

    void startup();

  protected:
    /**
     * Try to resolve the server name and connect to the 9p server.
     */
    void connectSocket();

    /** 9p server disconnect notification */
    void socketDisconnect();

    ssize_t read(uint8_t *data, size_t len);
    ssize_t write(const uint8_t *data, size_t len);

  private:
    class SocketDataEvent : public PollEvent
    {
      public:
        SocketDataEvent(VirtIO9PSocket &_parent, int fd, int event)
            : PollEvent(fd, event), parent(_parent) {}

        virtual ~SocketDataEvent() {};

        void process(int revent);

      private:
        VirtIO9PSocket &parent;
    };

    /** Socket connected to the 9p server */
    int fdSocket;

    std::unique_ptr<SocketDataEvent> dataEvent;
};

#endif // __DEV_VIRTIO_FS9P_HH__
