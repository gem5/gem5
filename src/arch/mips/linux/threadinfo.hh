/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Ali G. Saidi
 *          Nathan L. Binkert
 */


#ifndef __ARCH_MIPS_LINUX_LINUX_TREADNIFO_HH__
#define __ARCH_MIPS_LINUX_LINUX_TREADNIFO_HH__

#include "cpu/thread_context.hh"
#include "sim/system.hh"
#include "sim/vptr.hh"

namespace Linux {

class ThreadInfo
{
  private:
    ThreadContext *tc;
    System *sys;
    Addr pcbb;

    template <typename T>
    bool
    get_data(const char *symbol, T &data)
    {
        Addr addr = 0;
        if (!sys->kernelSymtab->findAddress(symbol, addr))
            return false;

        CopyOut(tc, &data, addr, sizeof(T));

        data = TheISA::gtoh(data);

        return true;
    }

  public:
    ThreadInfo(ThreadContext *_tc, Addr _pcbb = 0)
        : tc(_tc), sys(tc->getSystemPtr()), pcbb(_pcbb)
    {

    }
    ~ThreadInfo()
    {}

    inline Addr
    curThreadInfo()
    {
        panic("curThreadInfo not implemented for MIPS");
        Addr addr = pcbb;
        Addr sp;

        if (!addr)
            addr = tc->readMiscRegNoEffect(0/*TheISA::IPR_PALtemp23*/);

        FunctionalPort *p = tc->getPhysPort();
        p->readBlob(addr, (uint8_t *)&sp, sizeof(Addr));

        return sp & ~ULL(0x3fff);
    }

    inline Addr
    curTaskInfo(Addr thread_info = 0)
    {
        int32_t offset;
        if (!get_data("thread_info_task", offset))
            return 0;

        if (!thread_info)
            thread_info = curThreadInfo();

        Addr addr;
        CopyOut(tc, &addr, thread_info + offset, sizeof(addr));

        return addr;
    }

    int32_t
    curTaskPID(Addr thread_info = 0)
    {
        Addr offset;
        if (!get_data("task_struct_pid", offset))
            return -1;

        int32_t pid;
        CopyOut(tc, &pid, curTaskInfo(thread_info) + offset, sizeof(pid));

        return pid;
    }

    int64_t
    curTaskStart(Addr thread_info = 0)
    {
        Addr offset;
        if (!get_data("task_struct_start_time", offset))
            return -1;

        int64_t data;
        // start_time is actually of type timespec, but if we just
        // grab the first long, we'll get the seconds out of it
        CopyOut(tc, &data, curTaskInfo(thread_info) + offset, sizeof(data));

        return data;
    }

    std::string
    curTaskName(Addr thread_info = 0)
    {
        int32_t offset;
        int32_t size;

        if (!get_data("task_struct_comm", offset))
            return "FailureIn_curTaskName";

        if (!get_data("task_struct_comm_size", size))
            return "FailureIn_curTaskName";

        char buffer[size + 1];
        CopyStringOut(tc, buffer, curTaskInfo(thread_info) + offset, size);

        return buffer;
    }
};

/* namespace Linux */ }

#endif // __ARCH_MIPS_LINUX_LINUX_THREADINFO_HH__
