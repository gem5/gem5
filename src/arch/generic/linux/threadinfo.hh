/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#ifndef __ARCH_GENERIC_LINUX_THREADINFO_HH__
#define __ARCH_GENERIC_LINUX_THREADINFO_HH__

#include "cpu/thread_context.hh"
#include "mem/translating_port_proxy.hh"
#include "sim/system.hh"

namespace gem5
{

namespace linux
{

class ThreadInfo
{
  private:
    ThreadContext *tc;
    System *sys;

    ByteOrder byteOrder;

    template <typename T>
    bool
    get_data(const char *symbol, T &data)
    {
        auto &symtab = sys->workload->symtab(tc);
        auto it = symtab.find(symbol);
        if (it == symtab.end()) {
            warn_once("Unable to find kernel symbol %s\n", symbol);
            warn_once("Kernel not compiled with task_struct info; can't get "
                      "currently executing task/process/thread name/ids!\n");
            return false;
        }

        data = TranslatingPortProxy(tc).read<T>(it->address(), byteOrder);

        return true;
    }

  public:
    ThreadInfo(ThreadContext *_tc)
        : tc(_tc), sys(tc->getSystemPtr()),
        byteOrder(tc->getSystemPtr()->getGuestByteOrder())
    {

    }
    ~ThreadInfo()
    {}

    virtual Addr
    curThreadInfo()
    {
        panic("curThreadInfo() not implemented.");
    }

    Addr
    curTaskInfo(Addr thread_info = 0)
    {
        // Note that in Linux 4.10 the thread_info struct will no longer have a
        // pointer to the task_struct for arm64. See:
        // https://patchwork.kernel.org/patch/9333699/
        int32_t offset = 0;
        if (!get_data("thread_info_task", offset))
            return 0;

        if (!thread_info)
            thread_info = curThreadInfo();

        return TranslatingPortProxy(tc).read<Addr>(thread_info + offset);
    }

    int32_t
    curTaskPIDFromTaskStruct(Addr task_struct)
    {
        int32_t offset = 0;
        if (!get_data("task_struct_pid", offset))
            return -1;

        return TranslatingPortProxy(tc).read<int32_t>(task_struct + offset);
    }

    int32_t
    curTaskPID(Addr thread_info = 0)
    {
        return curTaskPIDFromTaskStruct(curTaskInfo(thread_info));
    }

    int32_t
    curTaskTGIDFromTaskStruct(Addr task_struct)
    {
        int32_t offset = 0;
        if (!get_data("task_struct_tgid", offset))
            return -1;

        return TranslatingPortProxy(tc).read<int32_t>(task_struct + offset);
    }

    int32_t
    curTaskTGID(Addr thread_info = 0)
    {
        return curTaskTGIDFromTaskStruct(curTaskInfo(thread_info));
    }

    int64_t
    curTaskStartFromTaskStruct(Addr task_struct)
    {
        int32_t offset = 0;
        if (!get_data("task_struct_start_time", offset))
            return -1;

        // start_time is actually of type timespec, but if we just
        // grab the first long, we'll get the seconds out of it
        return TranslatingPortProxy(tc).read<int64_t>(task_struct + offset);
    }

    int64_t
    curTaskStart(Addr thread_info = 0)
    {
        return curTaskStartFromTaskStruct(curTaskInfo(thread_info));
    }

    std::string
    curTaskNameFromTaskStruct(Addr task_struct)
    {
        int32_t offset = 0;
        int32_t size = 0;

        if (!get_data("task_struct_comm", offset))
            return "FailureIn_curTaskName";

        if (!get_data("task_struct_comm_size", size))
            return "FailureIn_curTaskName";

        char buffer[size + 1];
        TranslatingPortProxy(tc).readString(
                buffer, task_struct + offset, size);

        return buffer;
    }

    std::string
    curTaskName(Addr thread_info = 0)
    {
        return curTaskNameFromTaskStruct(curTaskInfo(thread_info));
    }

    int32_t
    curTaskMmFromTaskStruct(Addr task_struct)
    {
        int32_t offset;
        if (!get_data("task_struct_mm", offset))
            return -1;

        return TranslatingPortProxy(tc).read<int32_t>(task_struct + offset);
    }

    int32_t
    curTaskMm(Addr thread_info = 0)
    {
        return curTaskMmFromTaskStruct(curTaskInfo(thread_info));
    }
};

} // namespace linux
} // namespace gem5

#endif // __ARCH_GENERIC_LINUX_THREADINFO_HH__
