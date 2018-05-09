/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#include "base/logging.hh"
#include "systemc/ext/core/sc_attr.hh"

namespace sc_core
{

sc_attr_base::sc_attr_base(const std::string &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_attr_base::sc_attr_base(const sc_attr_base &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_attr_base::~sc_attr_base()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const std::string &
sc_attr_base::name() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const std::string *)nullptr;
}

void
sc_attr_base::warn_unimpl(const char *func)
{
    warn("%s not implemented.\n", func);
}

sc_attr_cltn::iterator
sc_attr_cltn::begin()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (iterator)nullptr;
}

sc_attr_cltn::const_iterator
sc_attr_cltn::begin() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (const_iterator)nullptr;
}

sc_attr_cltn::iterator
sc_attr_cltn::end()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (iterator)nullptr;
}

sc_attr_cltn::const_iterator
sc_attr_cltn::end() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return (const_iterator)nullptr;
}

} // namespace sc_core
