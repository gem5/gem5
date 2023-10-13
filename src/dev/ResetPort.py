# Copyright 2022 Google, Inc.
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

RESET_REQUEST_ROLE = "Reset Request"
RESET_RESPONSE_ROLE = "Reset Response"
Port.compat(RESET_REQUEST_ROLE, RESET_RESPONSE_ROLE)


# ResetRequestPort is an artifact request port for reset purpose.
class ResetRequestPort(Port):
    def __init__(self, desc):
        super().__init__(RESET_REQUEST_ROLE, desc, is_source=True)


# ResetResponsePort is an artifact response port for reset purpose.
# The owner should perform whole reset when receiving a request.
class ResetResponsePort(Port):
    def __init__(self, desc):
        super().__init__(RESET_RESPONSE_ROLE, desc)


# VectorResetRequestPort represents a bank of artifact reset request
# ports.
class VectorResetRequestPort(VectorPort):
    def __init__(self, desc):
        super().__init__(RESET_REQUEST_ROLE, desc, is_source=True)


# VectorResetResponsePort represents a bank of artifact reset request
# ports.
class VectorResetResponsePort(VectorPort):
    def __init__(self, desc):
        super().__init__(RESET_RESPONSE_ROLE, desc)
