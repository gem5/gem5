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
 * Header packet class for multi gem5 runs.
 *
 * For a high level description about multi gem5 see comments in
 * header file multi_iface.hh.
 *
 * The MultiHeaderPkt class defines the format of message headers
 * sent among gem5 processes during a multi gem5 simulation. A header packet
 * can either carry the description of data packet (i.e. a simulated Ethernet
 * packet) or a synchronisation related control command. In case of
 * data packet description, the corresponding data packet always follows
 * the header packet back-to-back.
 */
#ifndef __DEV_MULTI_PACKET_HH__
#define __DEV_MULTI_PACKET_HH__

#include <cstring>

#include "base/types.hh"

class MultiHeaderPkt
{
  private:
    MultiHeaderPkt() {}
    ~MultiHeaderPkt() {}

  public:
    /**
     * Simply type to help with calculating space requirements for
     * the corresponding header field.
     */
    typedef uint8_t AddressType[6];

    /**
     *  The msg type defines what informarion a multi header packet carries.
     */
    enum class MsgType
    {
        dataDescriptor,
        cmdPeriodicSyncReq,
        cmdPeriodicSyncAck,
        cmdCkptSyncReq,
        cmdCkptSyncAck,
        cmdAtomicSyncReq,
        cmdAtomicSyncAck,
        unknown
    };

    struct Header
    {
        /**
         * The msg type field is valid for all header packets. In case of
         * a synchronisation control command this is the only valid field.
         */
        MsgType msgType;
        Tick sendTick;
        Tick sendDelay;
        /**
         * Actual length of the simulated Ethernet packet.
         */
        unsigned dataPacketLength;
        /**
         * Source MAC address.
         */
        AddressType srcAddress;
        /**
         * Destination MAC address.
         */
        AddressType dstAddress;
    };

    static unsigned maxAddressLength();

    /**
     * Static functions for manipulating and comparing MAC addresses.
     */
    static void clearAddress(AddressType &addr);
    static bool isAddressEqual(const AddressType &addr1,
                               const AddressType &addr2);
    static bool isAddressLess(const AddressType &addr1,
                              const AddressType &addr2);

    static void copyAddress(AddressType &dest,
                            const AddressType &src);

    static bool isUnicastAddress(const AddressType &addr);
    static bool isMulticastAddress(const AddressType &addr);
    static bool isBroadcastAddress(const AddressType &addr);
};

#endif
