/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sim/fd_entry.hh"

#include "sim/serialize.hh"

namespace gem5
{

void
FDEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_closeOnExec);
}

void
FDEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_closeOnExec);
}

void
FileFDEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_closeOnExec);
    SERIALIZE_SCALAR(_flags);
    SERIALIZE_SCALAR(_fileName);
    SERIALIZE_SCALAR(_fileOffset);
    SERIALIZE_SCALAR(_mode);
}

void
FileFDEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_closeOnExec);
    UNSERIALIZE_SCALAR(_flags);
    UNSERIALIZE_SCALAR(_fileName);
    UNSERIALIZE_SCALAR(_fileOffset);
    UNSERIALIZE_SCALAR(_mode);
}

void
PipeFDEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_closeOnExec);
    SERIALIZE_SCALAR(_flags);
    // SERIALIZE_SCALAR(_pipeEndType);
}

void
PipeFDEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_closeOnExec);
    UNSERIALIZE_SCALAR(_flags);
    // UNSERIALIZE_SCALAR(_pipeEndType);
}

void
DeviceFDEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_closeOnExec);
    // SERIALIZE_SCALAR(_driver);
    SERIALIZE_SCALAR(_fileName);
}

void
DeviceFDEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_closeOnExec);
    // UNSERIALIZE_SCALAR(_driver);
    UNSERIALIZE_SCALAR(_fileName);
}

} // namespace gem5
