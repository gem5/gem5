/*
 * Copyright (c) 2018-2019 ARM Limited
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
 */

#include "arch/arm/tlbi_op.hh"

#include "arch/arm/tlb.hh"
#include "cpu/checker/cpu.hh"

namespace ArmISA {

void
TLBIALL::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushAllSecurity(secureLookup, targetEL);
    getDTBPtr(tc)->flushAllSecurity(secureLookup, targetEL);

    // If CheckerCPU is connected, need to notify it of a flush
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushAllSecurity(secureLookup,
                                               targetEL);
        getDTBPtr(checker)->flushAllSecurity(secureLookup,
                                               targetEL);
    }
}

void
ITLBIALL::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushAllSecurity(secureLookup, targetEL);
}

void
DTLBIALL::operator()(ThreadContext* tc)
{
    getDTBPtr(tc)->flushAllSecurity(secureLookup, targetEL);
}

void
TLBIASID::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushAsid(asid, secureLookup, targetEL);
    getDTBPtr(tc)->flushAsid(asid, secureLookup, targetEL);
    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushAsid(asid, secureLookup, targetEL);
        getDTBPtr(checker)->flushAsid(asid, secureLookup, targetEL);
    }
}

void
ITLBIASID::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushAsid(asid, secureLookup, targetEL);
}

void
DTLBIASID::operator()(ThreadContext* tc)
{
    getDTBPtr(tc)->flushAsid(asid, secureLookup, targetEL);
}

void
TLBIALLN::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushAllNs(targetEL);
    getDTBPtr(tc)->flushAllNs(targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushAllNs(targetEL);
        getDTBPtr(checker)->flushAllNs(targetEL);
    }
}

void
TLBIMVAA::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushMva(addr, secureLookup, targetEL);
    getDTBPtr(tc)->flushMva(addr, secureLookup, targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushMva(addr, secureLookup, targetEL);
        getDTBPtr(checker)->flushMva(addr, secureLookup, targetEL);
    }
}

void
TLBIMVA::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushMvaAsid(addr, asid,
                                  secureLookup, targetEL);
    getDTBPtr(tc)->flushMvaAsid(addr, asid,
                                  secureLookup, targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushMvaAsid(
            addr, asid, secureLookup, targetEL);
        getDTBPtr(checker)->flushMvaAsid(
            addr, asid, secureLookup, targetEL);
    }
}

void
ITLBIMVA::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushMvaAsid(
        addr, asid, secureLookup, targetEL);
}

void
DTLBIMVA::operator()(ThreadContext* tc)
{
    getDTBPtr(tc)->flushMvaAsid(
        addr, asid, secureLookup, targetEL);
}

void
TLBIIPA::operator()(ThreadContext* tc)
{
    getITBPtr(tc)->flushIpaVmid(addr,
        secureLookup, targetEL);
    getDTBPtr(tc)->flushIpaVmid(addr,
        secureLookup, targetEL);

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        getITBPtr(checker)->flushIpaVmid(addr,
            secureLookup, targetEL);
        getDTBPtr(checker)->flushIpaVmid(addr,
            secureLookup, targetEL);
    }
}

} // namespace ArmISA
