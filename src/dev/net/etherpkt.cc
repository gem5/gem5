/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

#include "dev/net/etherpkt.hh"

#include <iostream>

#include "base/inet.hh"
#include "base/misc.hh"
#include "sim/serialize.hh"

using namespace std;

void
EthPacketData::serialize(const string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".simLength", simLength);
    paramOut(cp, base + ".bufLength", bufLength);
    paramOut(cp, base + ".length", length);
    arrayParamOut(cp, base + ".data", data, length);
}

void
EthPacketData::unserialize(const string &base, CheckpointIn &cp)
{
    paramIn(cp, base + ".length", length);
    unsigned chkpt_buf_length;
    if (optParamIn(cp, base + ".bufLength", chkpt_buf_length)) {
        // If bufLength is in the checkpoint, make sure that the current buffer
        // is unallocated or that the checkpoint requested size is smaller than
        // the current buffer.
        assert(!data || chkpt_buf_length <= bufLength);
        bufLength = chkpt_buf_length;
    } else {
        // If bufLength is not in the checkpoint, try to use the existing
        // buffer or use length to size the buffer
        if (!data)
            bufLength = length;
    }
    assert(length <= bufLength);
    if (!data)
        data = new uint8_t[bufLength];
    arrayParamIn(cp, base + ".data", data, length);
    if (!optParamIn(cp, base + ".simLength", simLength))
        simLength = length;
}

