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

#ifndef __CPU_EA_LIST_HH__
#define __CPU_EA_LIST_HH__

#include <list>
#include <utility>

#include "base/types.hh"
#include "cpu/inst_seq.hh"

/**
 * Simple class to hold onto a list of pairs, each pair having a memory
 * instruction's sequence number and effective addr.  This list can be used
 * for memory disambiguation.  However, if I ever want to forward results, I
 * may have to use a list that holds DynInstPtrs.  Hence this may change in
 * the future.
 */
class EAList {
  private:
    typedef std::pair<InstSeqNum, Addr> instEA;
    typedef std::list<instEA>::iterator eaListIt;
    typedef std::list<instEA>::const_iterator constEAListIt;

    std::list<instEA> eaList;

  public:
    EAList() { }
    ~EAList() { }

    void addAddr(const InstSeqNum &new_sn, const Addr &new_ea);

    void clearAddr(const InstSeqNum &sn_to_clear, const Addr &ea_to_clear);

    /** Checks if any instructions older than check_sn have a conflicting
     *  address with check_ea.  Note that this function does not handle the
     *  sequence number rolling over.
     */
    bool checkConflict(const InstSeqNum &check_sn, const Addr &check_ea) const;

    void clear();

    void commit(const InstSeqNum &commit_sn);
};

#endif // __CPU_EA_LIST_HH__
