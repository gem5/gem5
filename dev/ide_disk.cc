/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/** @file
 * Device model implementation for an IDE disk
 */

#include <cerrno>
#include <cstring>
#include <deque>
#include <string>

#include "base/cprintf.hh" // csprintf
#include "base/trace.hh"
#include "dev/disk_image.hh"
#include "dev/ide_disk.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"
#include "sim/universe.hh"

using namespace std;

IdeDisk::IdeDisk(const string &name, DiskImage *img, int delay)
    : SimObject(name), ctrl(NULL), image(img)
{
    diskDelay = delay * ticksPerSecond / 1000;
}

IdeDisk::~IdeDisk()
{
}

void
IdeDisk::serialize(ostream &os)
{
}

void
IdeDisk::unserialize(Checkpoint *cp, const string &section)
{
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IdeDisk)

    SimObjectParam<DiskImage *> image;
    Param<int> disk_delay;

END_DECLARE_SIM_OBJECT_PARAMS(IdeDisk)

BEGIN_INIT_SIM_OBJECT_PARAMS(IdeDisk)

    INIT_PARAM(image, "Disk image"),
    INIT_PARAM_DFLT(disk_delay, "Fixed disk delay in milliseconds", 0)

END_INIT_SIM_OBJECT_PARAMS(IdeDisk)


CREATE_SIM_OBJECT(IdeDisk)
{
    return new IdeDisk(getInstanceName(), image, disk_delay);
}

REGISTER_SIM_OBJECT("IdeDisk", IdeDisk)

#endif //DOXYGEN_SHOULD_SKIP_THIS
