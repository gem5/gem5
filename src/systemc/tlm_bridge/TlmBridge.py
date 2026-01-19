# Copyright 2019 Google, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.SystemC import SystemC_ScModule
from m5.objects.Tlm import (
    TlmInitiatorSocket,
    TlmTargetSocket,
)
from m5.params import *
from m5.proxy import *


class Gem5ToTlmBridgeBase(SystemC_ScModule):
    type = "Gem5ToTlmBridgeBase"
    abstract = True
    cxx_class = "sc_gem5::Gem5ToTlmBridgeBase"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"

    system = Param.System(Parent.any, "system")

    gem5 = ResponsePort("gem5 response port")
    addr_ranges = VectorParam.AddrRange(
        [], "Addresses served by this port's TLM side"
    )


class TlmToGem5BridgeBase(SystemC_ScModule):
    type = "TlmToGem5BridgeBase"
    abstract = True
    cxx_class = "sc_gem5::TlmToGem5BridgeBase"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"

    system = Param.System(Parent.any, "system")

    gem5 = RequestPort("gem5 request port")


class Gem5ToTlmBridge32(Gem5ToTlmBridgeBase):
    type = "Gem5ToTlmBridge32"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::Gem5ToTlmBridge<32>"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"
    override_create = True

    tlm = TlmInitiatorSocket(32, "TLM initiator socket")


class Gem5ToTlmBridge64(Gem5ToTlmBridgeBase):
    type = "Gem5ToTlmBridge64"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::Gem5ToTlmBridge<64>"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"
    override_create = True

    tlm = TlmInitiatorSocket(64, "TLM initiator socket")


class Gem5ToTlmBridge128(Gem5ToTlmBridgeBase):
    type = "Gem5ToTlmBridge128"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::Gem5ToTlmBridge<128>"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"
    override_create = True

    tlm = TlmInitiatorSocket(128, "TLM initiator socket")


class Gem5ToTlmBridge256(Gem5ToTlmBridgeBase):
    type = "Gem5ToTlmBridge256"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::Gem5ToTlmBridge<256>"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"
    override_create = True

    tlm = TlmInitiatorSocket(256, "TLM initiator socket")


class Gem5ToTlmBridge512(Gem5ToTlmBridgeBase):
    type = "Gem5ToTlmBridge512"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::Gem5ToTlmBridge<512>"
    cxx_header = "systemc/tlm_bridge/gem5_to_tlm.hh"
    override_create = True

    tlm = TlmInitiatorSocket(512, "TLM initiator socket")


class TlmToGem5Bridge32(TlmToGem5BridgeBase):
    type = "TlmToGem5Bridge32"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::TlmToGem5Bridge<32>"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"
    override_create = True

    tlm = TlmTargetSocket(32, "TLM target socket")


class TlmToGem5Bridge64(TlmToGem5BridgeBase):
    type = "TlmToGem5Bridge64"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::TlmToGem5Bridge<64>"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"
    override_create = True

    tlm = TlmTargetSocket(64, "TLM target socket")


class TlmToGem5Bridge128(TlmToGem5BridgeBase):
    type = "TlmToGem5Bridge128"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::TlmToGem5Bridge<128>"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"
    override_create = True

    tlm = TlmTargetSocket(128, "TLM target socket")


class TlmToGem5Bridge256(TlmToGem5BridgeBase):
    type = "TlmToGem5Bridge256"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::TlmToGem5Bridge<256>"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"
    override_create = True

    tlm = TlmTargetSocket(256, "TLM target socket")


class TlmToGem5Bridge512(TlmToGem5BridgeBase):
    type = "TlmToGem5Bridge512"
    cxx_template_params = ["unsigned int BITWIDTH"]
    cxx_class = "sc_gem5::TlmToGem5Bridge<512>"
    cxx_header = "systemc/tlm_bridge/tlm_to_gem5.hh"
    override_create = True

    tlm = TlmTargetSocket(512, "TLM target socket")
