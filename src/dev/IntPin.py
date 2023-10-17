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

from m5.params import Port, VectorPort

INT_SOURCE_ROLE = "Int Source Pin"
INT_SINK_ROLE = "Int Sink Pin"
Port.compat(INT_SOURCE_ROLE, INT_SINK_ROLE)


# A source pin generally represents a single pin which might connect to
# multiple sinks.
class IntSourcePin(VectorPort):
    def __init__(self, desc):
        super().__init__(INT_SOURCE_ROLE, desc, is_source=True)


# A vector of source pins which might represent a bank of physical pins. Unlike
# IntSourcePin, each source pin in VectorIntSourcePin can only connect to a
# single sink pin. VectorIntSourcePin has the same definition as IntSourcePin
# right now, but will likely be implemented differently in the future.
# VectorIntSourcePin is defined as its own separate type to differentiate it
# from IntSourcePin and make it clear to the user how it should be interpreted
# and used.
class VectorIntSourcePin(VectorPort):
    def __init__(self, desc):
        super().__init__(INT_SOURCE_ROLE, desc, is_source=True)


# Each "physical" pin can be driven by a single source pin since there are no
# provisions for resolving competing signals running to the same pin.
class IntSinkPin(Port):
    def __init__(self, desc):
        super().__init__(INT_SINK_ROLE, desc)


# A vector of sink pins represents a bank of physical pins. For instance, an
# interrupt controller with many numbered input interrupts could represent them
# as a VectorIntSinkPin.
class VectorIntSinkPin(VectorPort):
    def __init__(self, desc):
        super().__init__(INT_SINK_ROLE, desc)
