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

from m5.params import *
from m5.proxy import *

from m5.objects.SystemC import SystemC_ScModule
from m5.objects.Tlm import TlmInitiatorSocket, TlmTargetSocket


def AMBA_TARGET_ROLE(width):
    return "AMBA TARGET %d" % width


def AMBA_INITIATOR_ROLE(width):
    return "AMBA INITIATOR %d" % width


def SC_REQUEST_PORT_ROLE(port_type):
    return "SC REQUEST PORT for %s" % port_type


def SC_RESPONSE_PORT_ROLE(port_type):
    return "SC RESPONSE PORT for %s" % port_type


class AmbaTargetSocket(Port):
    def __init__(self, width, desc):
        my_role = AMBA_INITIATOR_ROLE(width)
        peer_role = AMBA_TARGET_ROLE(width)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc)


class VectorAmbaTargetSocket(VectorPort):
    def __init__(self, width, desc):
        my_role = AMBA_INITIATOR_ROLE(width)
        peer_role = AMBA_TARGET_ROLE(width)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc)


class AmbaInitiatorSocket(Port):
    def __init__(self, width, desc):
        my_role = AMBA_TARGET_ROLE(width)
        peer_role = AMBA_INITIATOR_ROLE(width)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc, is_source=True)


class VectorAmbaInitiatorSocket(VectorPort):
    def __init__(self, width, desc):
        my_role = AMBA_TARGET_ROLE(width)
        peer_role = AMBA_INITIATOR_ROLE(width)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc, is_source=True)


class ScRequestPort(Port):
    def __init__(self, desc, port_type):
        my_role = SC_REQUEST_PORT_ROLE(port_type)
        peer_role = SC_RESPONSE_PORT_ROLE(port_type)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc)


class ScResponsePort(Port):
    def __init__(self, desc, port_type):
        my_role = SC_RESPONSE_PORT_ROLE(port_type)
        peer_role = SC_REQUEST_PORT_ROLE(port_type)
        Port.compat(my_role, peer_role)

        super().__init__(my_role, desc)


class AmbaToTlmBridge64(SystemC_ScModule):
    type = "AmbaToTlmBridge64"
    cxx_class = "gem5::fastmodel::AmbaToTlmBridge64"
    cxx_header = "arch/arm/fastmodel/amba_to_tlm_bridge.hh"

    amba = AmbaTargetSocket(64, "AMBA PV target socket")
    tlm = TlmInitiatorSocket(64, "TLM initiator socket")

    set_stream_id = Param.Bool(
        False, "Set this true to forward stream ID to gem5 world"
    )


class AmbaFromTlmBridge64(SystemC_ScModule):
    type = "AmbaFromTlmBridge64"
    cxx_class = "gem5::fastmodel::AmbaFromTlmBridge64"
    cxx_header = "arch/arm/fastmodel/amba_from_tlm_bridge.hh"

    tlm = TlmTargetSocket(64, "TLM target socket")
    amba = AmbaInitiatorSocket(64, "AMBA PV initiator socket")
