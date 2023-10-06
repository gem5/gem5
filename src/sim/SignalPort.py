# Copyright 2023 Google, Inc.
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
from m5.params import Port
from m5.params import VectorPort

SIGNAL_SOURCE_ROLE_TEMPLATE = "Signal source <%s>"
SIGNAL_SINK_ROLE_TEMPLATE = "Signal sink <%s>"


def SignalSourcePort(type_signature):
    source_role = SIGNAL_SOURCE_ROLE_TEMPLATE % type_signature
    sink_role = SIGNAL_SINK_ROLE_TEMPLATE % type_signature
    Port.compat(source_role, sink_role)

    class SignalSourcePort(Port):
        def __init__(self, desc):
            super().__init__(source_role, desc, is_source=True)

    return SignalSourcePort


def VectorSignalSourcePort(type_signature):
    source_role = SIGNAL_SOURCE_ROLE_TEMPLATE % type_signature
    sink_role = SIGNAL_SINK_ROLE_TEMPLATE % type_signature
    Port.compat(source_role, sink_role)

    class VectorSignalSourcePort(VectorPort):
        def __init__(self, desc):
            super().__init__(source_role, desc, is_source=True)

    return VectorSignalSourcePort


def SignalSinkPort(type_signature):
    source_role = SIGNAL_SOURCE_ROLE_TEMPLATE % type_signature
    sink_role = SIGNAL_SINK_ROLE_TEMPLATE % type_signature
    Port.compat(source_role, sink_role)

    class SignalSinkPort(Port):
        def __init__(self, desc):
            super().__init__(sink_role, desc)

    return SignalSinkPort


def VectorSignalSinkPort(type_signature):
    source_role = SIGNAL_SOURCE_ROLE_TEMPLATE % type_signature
    sink_role = SIGNAL_SINK_ROLE_TEMPLATE % type_signature
    Port.compat(source_role, sink_role)

    class VectorSignalSinkPort(VectorPort):
        def __init__(self, desc):
            super().__init__(sink_role, desc)

    return VectorSignalSinkPort
