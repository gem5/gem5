/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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
 */

#include <unistd.h>

#include <cstdlib>

#include "arch/x86/pin/message.h"
#include "base/logging.hh"
#include "debug/PinCPU.hh"

namespace gem5
{

namespace X86ISA
{

void
Message::send(int fd) const
{
    const uint8_t *data = reinterpret_cast<const uint8_t *>(this);
    size_t size = sizeof *this;
    while (size > 0) {
        ssize_t bytes_written;
        if ((bytes_written = write(fd, data, size)) < 0) {
            panic("Failed to write message!\n");
        }
        data += bytes_written;
        size -= bytes_written;
    }
}

void
Message::recv(int fd)
{
    // inform("receiving message\n");
    type = (Type)-1;
    uint8_t *data = reinterpret_cast<uint8_t *>(this);
    size_t size = sizeof *this;
    while (size > 0) {
        ssize_t bytes_read = read(fd, data, size);
        if (bytes_read < 0) {
            panic("read failed: %s\n", std::strerror(errno));
        } else if (bytes_read == 0) {
            panic("Pin closed the pipe!\n");
        }
        data += bytes_read;
        size -= bytes_read;
    }
    // inform("received message (type=%i)\n", type);
}

std::ostream &
operator<<(std::ostream &os, const Message &msg)
{
    os << "pinmsg{.type=";
    switch (msg.type) {
        case Message::Ack:
            os << "ACK";
            break;
        default:
            panic("unhandled message type!\n");
    }
    os << "}";
    return os;
}

} // namespace X86ISA
} // namespace gem5
