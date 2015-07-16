/*
 * Copyright (c) 2015 ARM Limited
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
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 *
 * Authors: Gabor Dozsa
 */

/* @file
 *  Message server using TCP stream sockets for parallel gem5 runs.
 *
 * For a high level description about multi gem5 see comments in
 * header files src/dev/multi_iface.hh and src/dev/tcp_iface.hh.
 *
 * This file implements the central message server process for multi gem5.
 * The server is responsible the following tasks.
 * 1. Establishing a TCP socket connection for each gem5 process (clients).
 *
 * 2. Process data messages coming in from clients. The server checks
 * the MAC addresses in the header message and transfers the message
 * to the target(s) client(s).
 *
 * 3. Processing synchronisation related control messages. Synchronisation
 * is performed as follows. The server waits for a 'barrier enter' message
 * from all the clients. When the last such control message arrives, the
 * server sends out a 'barrier leave' control message to all the clients.
 *
 * 4. Triggers complete termination in case a client exits. A client may
 * exit either by calling 'm5 exit' pseudo instruction or due to a fatal
 * error. In either case, we assume that the entire multi simulation needs to
 * terminate. The server triggers full termination by tearing down the
 * open TCP sockets.
 *
 * The TCPServer class is instantiated as a singleton object.
 *
 * The server can be built independently from the rest of gem5 (and it is
 * architecture agnostic). See the Makefile in the same directory.
 *
 */

#include <poll.h>

#include <map>
#include <vector>

#include "dev/etherpkt.hh"
#include "dev/multi_packet.hh"

/**
 * The maximum length of an Ethernet packet (allowing Jumbo frames).
 */
#define MAX_ETH_PACKET_LENGTH 9014

class TCPServer
{
  public:
    typedef MultiHeaderPkt::AddressType AddressType;
    typedef MultiHeaderPkt::Header Header;
    typedef MultiHeaderPkt::MsgType MsgType;

  private:

    enum
    class SyncState { periodic, ckpt, asyncCkpt, atomic, idle };
    /**
     * The Channel class encapsulates all the information about a client
     * and its current status.
     */
    class Channel
    {
      private:
        /**
         * The MAC address of the client.
         */
        AddressType address;

        /**
         * Update the client MAC address. It is called every time a new data
         * packet is to come in.
         */
        void updateAddress(const AddressType &new_addr);
        /**
         * Process an incoming command message.
         */
        void processCmd(MultiHeaderPkt::MsgType cmd, Tick send_tick);


      public:
        /**
         * TCP stream socket.
         */
        int fd;
        /**
         * Is client connected?
         */
        bool isAlive;
        /**
         * Current state of the channel wrt. multi synchronisation.
         */
        SyncState state;
        /**
         * Multi rank of the client
         */
        unsigned rank;

      public:
        Channel();
        ~Channel () {}


        /**
         * Receive and process the next incoming header packet.
         */
        void headerPktIn();
        /**
         * Send raw data to the connected client.
         *
         * @param data The data to send.
         * @param size Size of the data (in bytes).
         */
        void sendRaw(const void *data, unsigned size) const;
        /**
         * Receive raw data from the connected client.
         *
         * @param buf The buffer to store the incoming data into.
         * @param size Size of data to receive (in bytes).
         * @return In case of success, it returns size. Zero is returned
         * if the socket is already closed by the client.
         */
        unsigned recvRaw(void *buf, unsigned size) const;
    };

    /**
     * The array of socket descriptors needed by the poll() system call.
     */
    std::vector<struct pollfd> clientsPollFd;
    /**
     * Array holding all clients info.
     */
    std::vector<Channel> clientsChannel;


    /**
     * We use a map to select the target client based on the destination
     * MAC address.
     */
    struct AddressCompare
    {
        bool operator()(const AddressType *a1, const AddressType *a2)
        {
            return MultiHeaderPkt::isAddressLess(*a1, *a2);
        }
    };
    std::map<const AddressType *, Channel *, AddressCompare> addressMap;

    /**
     * As we dealt with only one message at a time, we can allocate and re-use
     * a single packet buffer (to hold any incoming data packet).
     */
    uint8_t packetBuffer[MAX_ETH_PACKET_LENGTH];
    /**
     * Send tick of the current periodic sync. It is used for sanity check.
     */
    Tick _periodicSyncTick;
    /**
     * The singleton server object.
     */
    static TCPServer *instance;

    /**
     * Set up the socket connections to all the clients.
     *
     * @param listen_port The port we are listening on for new client
     * connection requests.
     * @param nclients The number of clients to connect to.
     * @param timeout Timeout in sec to complete the setup phase
     * (i.e. all gem5 establish socket connections)
     */
    void construct(unsigned listen_port, unsigned nclients, int timeout);
    /**
     * Transfer the header and the follow up data packet to the target(s)
     * clients.
     *
     * @param hdr The header message structure.
     * @param ch The source channel for the message.
     */
    void xferData(const Header &hdr, const Channel &ch);
    /**
     * Check if the current round of a synchronisation is completed and notify
     * the clients if it is so.
     *
     * @param st The state all channels should have if sync is complete.
     * @param ack The type of ack message to send out if the sync is compete.
     */
    void syncTryComplete(SyncState st, MultiHeaderPkt::MsgType ack);
    /**
     * Broadcast a request for checkpoint sync.
     *
     * @param ch The source channel of the checkpoint sync request.
     */
    void ckptPropagate(Channel &ch);
    /**
     * Setter for current periodic send tick.
     */
    void periodicSyncTick(Tick t) { _periodicSyncTick = t; }
    /**
     * Getter for current periodic send tick.
     */
    Tick periodicSyncTick() { return _periodicSyncTick; }

  public:

    TCPServer(unsigned clients_num, unsigned listen_port, int timeout_in_sec);
    ~TCPServer();

    /**
     * The main server loop that waits for and processes incoming messages.
     */
    void run();
};
