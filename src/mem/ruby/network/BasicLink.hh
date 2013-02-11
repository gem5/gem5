/*
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_NETWORK_BASIC_LINK_HH__
#define __MEM_RUBY_NETWORK_BASIC_LINK_HH__

#include <iostream>
#include <string>
#include <vector>

#include "params/BasicExtLink.hh"
#include "params/BasicIntLink.hh"
#include "params/BasicLink.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "sim/sim_object.hh"

class BasicLink : public SimObject
{
  public:
    typedef BasicLinkParams Params;
    BasicLink(const Params *p);
    const Params *params() const { return (const Params *)_params; }

    void init();

    void print(std::ostream& out) const;

    Cycles m_latency;
    int m_bandwidth_factor;
    int m_weight;
};

inline std::ostream&
operator<<(std::ostream& out, const BasicLink& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

class BasicExtLink : public BasicLink
{
  public:
    typedef BasicExtLinkParams Params;
    BasicExtLink(const Params *p);
    const Params *params() const { return (const Params *)_params; }

    friend class Topology;

  protected:
    BasicRouter* m_int_node;
    AbstractController* m_ext_node;
};

class BasicIntLink : public BasicLink
{
  public:
    typedef BasicIntLinkParams Params;
    BasicIntLink(const Params *p);
    const Params *params() const { return (const Params *)_params; }

    friend class Topology;

  protected:
    BasicRouter* m_node_a;
    BasicRouter* m_node_b;
};

#endif // __MEM_RUBY_NETWORK_BASIC_LINK_HH__
