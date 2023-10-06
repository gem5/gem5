# Copyright (c) 2021 ARM Limited
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
from ruby import CHI_config


# CustomMesh parameters for a 2x4 mesh. Routers will have the following layout:
#
# 0 --- 1 --- 2 --- 3
# |     |     |     |
# 4 --- 5 --- 6 --- 7
#
# Default parameter are configs/ruby/CHI_config.py
#
class NoC_Params(CHI_config.NoC_Params):
    num_rows = 2
    num_cols = 4


# Specialization of nodes to define bindings for each CHI node type
# needed by CustomMesh.
# The default types are defined in CHI_Node and their derivatives in
# configs/ruby/CHI_config.py


class CHI_RNF(CHI_config.CHI_RNF):
    class NoC_Params(CHI_config.CHI_RNF.NoC_Params):
        router_list = [1, 2, 5, 6]


class CHI_HNF(CHI_config.CHI_HNF):
    class NoC_Params(CHI_config.CHI_HNF.NoC_Params):
        router_list = [1, 2, 5, 6]


class CHI_MN(CHI_config.CHI_MN):
    class NoC_Params(CHI_config.CHI_MN.NoC_Params):
        router_list = [4]


class CHI_SNF_MainMem(CHI_config.CHI_SNF_MainMem):
    class NoC_Params(CHI_config.CHI_SNF_MainMem.NoC_Params):
        router_list = [0, 4]


class CHI_SNF_BootMem(CHI_config.CHI_SNF_BootMem):
    class NoC_Params(CHI_config.CHI_SNF_BootMem.NoC_Params):
        router_list = [3]


class CHI_RNI_DMA(CHI_config.CHI_RNI_DMA):
    class NoC_Params(CHI_config.CHI_RNI_DMA.NoC_Params):
        router_list = [7]


class CHI_RNI_IO(CHI_config.CHI_RNI_IO):
    class NoC_Params(CHI_config.CHI_RNI_IO.NoC_Params):
        router_list = [7]
