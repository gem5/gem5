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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_LINK_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_LINK_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "params/GarnetIntLink_d.hh"
#include "params/GarnetExtLink_d.hh"

class GarnetIntLink_d : public BasicLink
{
  public:
    typedef GarnetIntLink_dParams Params;
    GarnetIntLink_d(const Params *p);

    void init();

    void print(std::ostream& out) const;

    friend class GarnetNetwork_d;

  protected:
    NetworkLink_d* m_network_links[2];
    CreditLink_d* m_credit_links[2];
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetIntLink_d& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

class GarnetExtLink_d : public BasicLink
{
  public:
    typedef GarnetExtLink_dParams Params;
    GarnetExtLink_d(const Params *p);

    void init();

    void print(std::ostream& out) const;

    friend class GarnetNetwork_d;

  protected:
    NetworkLink_d* m_network_links[2];
    CreditLink_d* m_credit_links[2];
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetExtLink_d& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_LINK_HH__
