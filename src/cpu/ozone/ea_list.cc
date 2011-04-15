/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Nathan Binkert
 */

#include "arch/isa_traits.hh"
#include "cpu/ooo_cpu/ea_list.hh"
#include "cpu/inst_seq.hh"

void
EAList::addAddr(const InstSeqNum &new_sn, const Addr &new_ea)
{
    instEA newEA(new_sn, new_ea);

    eaList.push_back(newEA);
}

void
EAList::clearAddr(const InstSeqNum &sn_to_clear, const Addr &ea_to_clear)
{
    eaListIt list_it = eaList.begin();

    while (list_it != eaList.end() && (*list_it).first != sn_to_clear) {
        assert((*list_it).second == ea_to_clear);
    }
}

bool
EAList::checkConflict(const InstSeqNum &check_sn, const Addr &check_ea) const
{
    const constEAListIt list_it = eaList.begin();

    while (list_it != eaList.end() && (*list_it).first < check_sn) {
        if ((*list_it).second == check_ea) {
            return true;
        }
    }

    return false;
}

void
EAList::clear()
{
    eaList.clear();
}

void
EAList::commit(const InstSeqNum &commit_sn)
{
    while (!eaList.empty() && eaList.front().first <= commit_sn) {
        eaList.pop_front();
    }
}
