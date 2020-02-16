/*
 * Copyright (c) 2015, University of Kaiserslautern
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "systemc/tlm_bridge/sc_ext.hh"

#include "systemc/ext/utils/sc_report_handler.hh"

namespace Gem5SystemC
{

Gem5Extension::Gem5Extension(PacketPtr _packet)
{
    packet = _packet;
    pipeThrough = false;
}

Gem5Extension &
Gem5Extension::getExtension(const tlm::tlm_generic_payload *payload)
{
    Gem5Extension *result = NULL;
    payload->get_extension(result);
    sc_assert(result != NULL);
    return *result;
}

Gem5Extension &
Gem5Extension::getExtension(const tlm::tlm_generic_payload &payload)
{
    return Gem5Extension::getExtension(&payload);
}

PacketPtr
Gem5Extension::getPacket()
{
    return packet;
}

tlm::tlm_extension_base *
Gem5Extension::clone() const
{
    return new Gem5Extension(packet);
}

void
Gem5Extension::copy_from(const tlm::tlm_extension_base &ext)
{
    const Gem5Extension &cpyFrom = static_cast<const Gem5Extension &>(ext);
    packet = cpyFrom.packet;
}

} // namespace Gem5SystemC
