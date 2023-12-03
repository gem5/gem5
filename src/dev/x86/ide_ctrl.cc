/*
 * Copyright 2022 Google, Inc.
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

#include "dev/x86/ide_ctrl.hh"

namespace gem5
{

X86IdeController::X86IdeController(const Params &p) : IdeController(p)
{
    for (int i = 0; i < p.port_int_primary_connection_count; i++) {
        intPrimary.push_back(new IntSourcePin<X86IdeController>(
            csprintf("%s.int_primary[%d]", name(), i), i, this));
    }
    for (int i = 0; i < p.port_int_secondary_connection_count; i++) {
        intSecondary.push_back(new IntSourcePin<X86IdeController>(
            csprintf("%s.int_secondary[%d]", name(), i), i, this));
    }
}

void
X86IdeController::postInterrupt(bool is_primary)
{
    auto &pin = is_primary ? intPrimary : intSecondary;
    for (auto *wire : pin)
        wire->raise();
}

void
X86IdeController::clearInterrupt(bool is_primary)
{
    auto &pin = is_primary ? intPrimary : intSecondary;
    for (auto *wire : pin)
        wire->lower();
}

Port &
X86IdeController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "int_primary")
        return *intPrimary.at(idx);
    else if (if_name == "int_secondary")
        return *intSecondary.at(idx);
    else
        return IdeController::getPort(if_name, idx);
}

} // namespace gem5
