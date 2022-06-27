/*
 * Copyright 2019 Google, Inc.
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

#include "dev/intpin.hh"

#include "base/logging.hh"

namespace gem5
{

void
IntSinkPinBase::bind(Port &peer)
{
    source = dynamic_cast<IntSourcePinBase *>(&peer);
    fatal_if(!source, "Attempt to bind interrupt sink pin %s to "
            "incompatible port %s.", name(), peer.name());
    Port::bind(peer);
}

void
IntSinkPinBase::unbind()
{
    source = nullptr;
    Port::unbind();
}

void
IntSourcePinBase::bind(Port &peer)
{
    sink = dynamic_cast<IntSinkPinBase *>(&peer);
    fatal_if(!sink, "Attempt to bind interrupt source pin %s to "
            "incompatible port %s.", name(), peer.name());
    Port::bind(peer);
}

void
IntSourcePinBase::unbind()
{
    sink = nullptr;
    Port::unbind();
}

} // namespace gem5
