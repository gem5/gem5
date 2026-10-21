# Copyright (c) 2021-2024, 2026 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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


import m5
from m5.objects import *

from gem5.components.cachehierarchies.chi.nodes.abstract_node import (
    CHI_Node,
    CHI_NodeType,
)


class CHI_SNF_Base(CHI_Node):
    """
    Creates CHI node controllers for the memory controllers
    """

    def __init__(
        self,
        ruby_system,
        node_type,
        cntrl,
    ):
        super().__init__(ruby_system, node_type)
        self.cntrl = cntrl

    def getNetworkSideControllers(self):
        """
        Returns all ruby controllers that need to be connected to the
        network
        """
        return [self.cntrl]

    def getAllControllers(self):
        """
        Returns all ruby controllers associated with this node
        """
        return [self.cntrl]


class CHI_SNF_BootMem(CHI_SNF_Base):
    """
    Create the SNF for the boot memory
    """

    def __init__(self, ruby_system, cntrl):
        super().__init__(ruby_system, CHI_NodeType.CHI_SNF_BootMem, cntrl)


class CHI_SNF_MainMem(CHI_SNF_Base):
    """
    Create the SNF for a list main memory controllers
    """

    def __init__(self, ruby_system, cntrl):
        super().__init__(ruby_system, CHI_NodeType.CHI_SNF_MainMem, cntrl)
