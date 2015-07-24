/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 *          Steve Reinhardt
 */

#include "base/misc.hh"
#include "fd_entry.hh"

using namespace std;

void
FDEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(fd);
    if (fd != -1) {
        SERIALIZE_SCALAR(mode);
        SERIALIZE_SCALAR(flags);
        SERIALIZE_SCALAR(isPipe);
        SERIALIZE_SCALAR(readPipeSource);
        SERIALIZE_SCALAR(fileOffset);
        SERIALIZE_SCALAR(filename);
    }
    if (driver)
        warn("EmulatedDriver objects do not currently support checkpoints");
}

void
FDEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(fd);
    if (fd != -1) {
        UNSERIALIZE_SCALAR(mode);
        UNSERIALIZE_SCALAR(flags);
        UNSERIALIZE_SCALAR(isPipe);
        UNSERIALIZE_SCALAR(readPipeSource);
        UNSERIALIZE_SCALAR(fileOffset);
        UNSERIALIZE_SCALAR(filename);
    }
    driver = NULL;
}

bool
FDEntry::isFree()
{
    return (fd == -1 && driver == NULL);
}

void
FDEntry::set(int sim_fd, const string name, int flags, int mode, bool pipe)
{
    fd = sim_fd;
    filename = name;
    this->flags = flags;
    this->mode = mode;
    isPipe = pipe;
    fileOffset = 0;
    readPipeSource = 0;
    driver = NULL;
}

void
FDEntry::reset()
{
    set(-1, "", 0, 0, false);
}
