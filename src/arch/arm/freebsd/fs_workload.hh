/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#ifndef __ARCH_ARM_FREEBSD_FS_WORKLOAD_HH__
#define __ARCH_ARM_FREEBSD_FS_WORKLOAD_HH__

#include <map>

#include "arch/arm/fs_workload.hh"
#include "kern/freebsd/events.hh"
#include "params/ArmFsFreebsd.hh"

namespace gem5
{

namespace ArmISA
{

class FsFreebsd : public ArmISA::FsWorkload
{
  public:
    /** Boilerplate params code */
    PARAMS(ArmFsFreebsd);

    /** When enabled, dump stats/task info on context switches for
     *  Streamline and per-thread cache occupancy studies, etc. */
    bool enableContextSwitchStatsDump;

    /** This map stores a mapping of OS process IDs to internal Task IDs. The
     * mapping is done because the stats system doesn't tend to like vectors
     * that are much greater than 1000 items and the entire process space is
     * 65K. */
    std::map<uint32_t, uint32_t> taskMap;

    /** This is a file that is placed in the run directory that prints out
     * mappings between taskIds and OS process IDs */
    std::ostream* taskFile;

    FsFreebsd(const Params &p);
    ~FsFreebsd();

    void initState() override;

    /** This function creates a new task Id for the given pid.
     * @param tc thread context that is currentyl executing  */
    void mapPid(ThreadContext* tc, uint32_t pid);

  private:
    /** Event to halt the simulator if the kernel calls panic()  */
    PCEvent *kernelPanic = nullptr;

    /** Event to halt the simulator if the kernel calls oopses  */
    PCEvent *kernelOops = nullptr;

    /**
     * PC based event to skip udelay(<time>) calls and quiesce the
     * processor for the appropriate amount of time. This is not functionally
     * required but does speed up simulation.
     */
    PCEvent *skipUDelay = nullptr;

    /** These variables store addresses of important data structures
     * that are normaly kept coherent at boot with cache mainetence operations.
     * Since these operations aren't supported in gem5, we keep them coherent
     * by making them uncacheable until all processors in the system boot.
     */
    Addr secDataPtrAddr;
    Addr secDataAddr;
    Addr penReleaseAddr;
    Addr pen64ReleaseAddr;
    Addr bootReleaseAddr;
};

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_FREEBSD_FS_WORKLOAD_HH__
