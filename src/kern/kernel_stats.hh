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
 * Authors: Lisa Hsu
 *          Nathan Binkert
 */

#ifndef __KERNEL_STATS_HH__
#define __KERNEL_STATS_HH__

#include <string>

#include "config/the_isa.hh"
#include "sim/serialize.hh"
#include "sim/stats.hh"

// What does kernel stats expect is included?
class System;

namespace Kernel {

class Statistics : public Serializable
{
  protected:
    std::string myname;

  protected:
    Stats::Scalar _arm;
    Stats::Scalar _quiesce;
    Stats::Scalar _hwrei;

    Stats::Vector _iplCount;
    Stats::Vector _iplGood;
    Stats::Vector _iplTicks;
    Stats::Formula _iplUsed;

#if THE_ISA == ALPHA_ISA
    Stats::Vector _syscall;
#endif
//    Stats::Vector _faults;

  private:
    int iplLast;
    Tick iplLastTick;

  public:
    Statistics(System *system);
    virtual ~Statistics() {}

    const std::string name() const { return myname; }
    void regStats(const std::string &name);

  public:
    void arm() { _arm++; }
    void quiesce() { _quiesce++; }
    void swpipl(int ipl);

  public:
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace Kernel

#endif // __KERNEL_STATS_HH__
