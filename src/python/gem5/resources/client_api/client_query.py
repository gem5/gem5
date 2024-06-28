# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
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

from typing import Optional

from _m5 import core

"""
This class is a data class that represents a query to the client.
It encapsulates the fields required to query resources from the client.
Right now, it only contains the resource_id, resource_version, and gem5_version
fields, but it can be expanded to include more fields in the future, if needed.
"""


class ClientQuery:
    def __init__(
        self,
        resource_id: str,
        resource_version: Optional[str] = None,
        gem5_version: Optional[str] = core.gem5Version,
    ):
        self.resource_id = resource_id
        self.resource_version = resource_version
        # We only need the major and minor version numbers.
        # As on database side, we only store the major and minor
        # version numbers.
        self.gem5_version = ".".join(gem5_version.split(".")[:2])

    def get_resource_id(self) -> str:
        return self.resource_id

    def get_resource_version(self) -> Optional[str]:
        return self.resource_version

    def get_gem5_version(self) -> Optional[str]:
        return self.gem5_version
