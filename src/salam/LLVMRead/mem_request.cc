/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include <iomanip>
#include <sstream>

#include "mem_request.hh"

MemoryRequest::MemoryRequest(Addr add, size_t len)
{
    address = add;
    length = len;

    needToRead = true;
    needToWrite = false;

    currentReadAddr = address;

    beginAddr = address;
    readLeft = length;
    writeLeft = 0;
    totalLength = length;
    readDone = 0;

    buffer = new uint8_t[length];
    readsDone = new bool[length];
    std::memset(buffer, 0, length);

    for (int i = 0; i < length; i++) {
        readsDone[i] = false;
    }
    pkt = NULL;
}

MemoryRequest::MemoryRequest(Addr add, const void *data, size_t len)
{
    address = add;
    length = len;

    needToWrite = true;
    needToRead = false;

    currentWriteAddr = address;

    readLeft = 0;
    writeLeft = length;

    totalLength = length;
    writeLeft = totalLength;

    readDone = length;
    writeDone = 0;

    buffer = new uint8_t[length];
    readsDone = new bool[length];
    std::memcpy(buffer, data, length);
    pkt = NULL;
}

std::string
MemoryRequest::printBuffer()
{
    std::stringstream ss;
    ss << "0x";
    for (int i = totalLength - 1; i >= 0; i--) {
        unsigned tmp = buffer[i];
        ss << std::setfill('0') << std::setw(2) << std::hex << tmp;
    }
    return ss.str();
}
