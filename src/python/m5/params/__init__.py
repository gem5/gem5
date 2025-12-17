# Copyright (c) 2012-2014, 2017-2019, 2021, 2024-2025 Arm Limited
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
# Copyright (c) 2004-2006 The Regents of The University of Michigan
# Copyright (c) 2010-2011 Advanced Micro Devices, Inc.
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

"""
This package contains all the parameter types for SimObjects.
"""

from .base_params import *
from .deprecated_params import *
from .enum_params import *
from .network_params import *
from .null_params import *
from .param_types import *
from .pin_params import *
from .port_params import *
from .power_params import *
from .time_params import *

__all__ = [
    # base_params
    "ParamValue",
    "ParamDesc",
    "VectorParamValue",
    "SimObjectVector",
    "VectorParamDesc",
    "Param",
    "VectorParam",
    "isSimObject",
    "isSimObjectSequence",
    "isSimObjectClass",
    "OptionalParam",
    "OptionalParamDesc",
    "DictParam",
    "DictParamDesc",
    "DictParamValue",
    # null_params
    "NULL",
    "isNullPointer",
    "NullOpt",
    "isNullOpt",
    # deprecated_params
    "DeprecatedParam",
    # enum_params
    "Enum",
    "ScopedEnum",
    "ByteOrder",
    # network_params
    "HostSocket",
    "IncEthernetAddr",
    "NextEthernetAddr",
    "EthernetAddr",
    "IpAddress",
    "IpNetmask",
    "IpWithPort",
    "NetworkBandwidth",
    # param_types
    "String",
    "Int",
    "Unsigned",
    "Int8",
    "UInt8",
    "Int16",
    "UInt16",
    "Int32",
    "UInt32",
    "Int64",
    "UInt64",
    "Counter",
    "Tick",
    "TcpPort",
    "UdpPort",
    "Percent",
    "Cycles",
    "Float",
    "MemorySize",
    "MemorySize32",
    "Addr",
    "AddrRange",
    "Bool",
    "Time",
    "MemoryBandwidth",
    "PcCountPair",
    "MaxAddr",
    "MaxTick",
    "AllMemory",
    # port_params
    "Port",
    "RequestPort",
    "ResponsePort",
    "VectorPort",
    "VectorRequestPort",
    "VectorResponsePort",
    "MasterPort",
    "SlavePort",
    "VectorMasterPort",
    "VectorSlavePort",
    # power_params
    "Voltage",
    "Current",
    "Energy",
    "Temperature",
    # time_params
    "Latency",
    "Frequency",
    "Clock",
    # pin_params
    "IntSourcePin",
    "VectorIntSourcePin",
    "IntSinkPin",
    "VectorIntSinkPin",
    "ResetRequestPort",
    "ResetResponsePort",
    "VectorResetRequestPort",
    "VectorResetResponsePort",
]
