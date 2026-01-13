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

import os

from .. import ticks
from ..util import convert
from .base_params import ParamValue


class HostSocket(ParamValue):
    cxx_type = "ListenSocketConfig"

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/socket.hh"', add_once=True)

    def __init__(self, value):
        if isinstance(value, HostSocket):
            self.value = value.value
        else:
            self.value = value

    def getValue(self):
        from _m5.socket import (
            listenSocketEmptyConfig,
            listenSocketInetConfig,
            listenSocketUnixAbstractConfig,
            listenSocketUnixFileConfig,
        )

        if isinstance(self.value, str):
            if self.value[0] == "@":
                return listenSocketUnixAbstractConfig(self.value[1:])
            else:
                d, f = os.path.split(self.value)
                return listenSocketUnixFileConfig(d, f)
        else:
            if self.value == 0:
                return listenSocketEmptyConfig()
            else:
                return listenSocketInetConfig(self.value)

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        if isinstance(self.value, str):
            return self.value
        else:
            return "#" + str(self.value)

    def ini_str(self):
        if isinstance(self.value, str):
            if self.value[0] == "@":
                return self.value
            else:
                return "P" + self.value
        else:
            return "#" + str(self.value)

    @classmethod
    def cxx_ini_predecls(cls, code):
        code('#include "base/socket.hh"', add_once=True)

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{ret} ListenSocketConfig::parseIni({src}, {dest});")


def IncEthernetAddr(addr, val=1):
    bytes = [int(x, 16) for x in addr.split(":")]
    bytes[5] += val
    for i in (5, 4, 3, 2, 1):
        val, rem = divmod(bytes[i], 256)
        bytes[i] = rem
        if val == 0:
            break
        bytes[i - 1] += val
    assert bytes[0] <= 255
    return ":".join(map(lambda x: f"{x:02x}", bytes))


_NextEthernetAddr = "00:90:00:00:00:01"


def NextEthernetAddr():
    global _NextEthernetAddr

    value = _NextEthernetAddr
    _NextEthernetAddr = IncEthernetAddr(_NextEthernetAddr, 1)
    return value


class EthernetAddr(ParamValue):
    cxx_type = "networking::EthAddr"
    ex_str = "00:90:00:00:00:01"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"', add_once=True)

    def __init__(self, value):
        if value == NextEthernetAddr:
            self.value = value
            return

        if not isinstance(value, str):
            raise TypeError("expected an ethernet address and didn't get one")

        bytes = value.split(":")
        if len(bytes) != 6:
            raise TypeError(f"invalid ethernet address {value}")

        for byte in bytes:
            if not 0 <= int(byte, base=16) <= 0xFF:
                raise TypeError(f"invalid ethernet address {value}")

        self.value = value

    def __call__(self, value):
        self.__init__(value)
        return value

    def unproxy(self, base):
        if self.value == NextEthernetAddr:
            return EthernetAddr(self.value())
        return self

    def getValue(self):
        from _m5.net import EthAddr

        return EthAddr(self.value)

    def __str__(self):
        return self.value

    def ini_str(self):
        return self.value

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{dest} = networking::EthAddr({src});")
        code(f"{ret} true;")


# When initializing an IpAddress, pass in an existing IpAddress, a string of
# the form "a.b.c.d", or an integer representing an IP.
class IpAddress(ParamValue):
    cxx_type = "networking::IpAddress"
    ex_str = "127.0.0.1"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"', add_once=True)

    def __init__(self, value):
        if isinstance(value, IpAddress):
            self.ip = value.ip
        else:
            try:
                self.ip = convert.toIpAddress(value)
            except TypeError:
                self.ip = int(value)
        self.verifyIp()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        tup = [(self.ip >> i) & 0xFF for i in (24, 16, 8, 0)]
        return "%d.%d.%d.%d" % tuple(tup)

    def __eq__(self, other):
        if isinstance(other, IpAddress):
            return self.ip == other.ip
        elif isinstance(other, str):
            try:
                return self.ip == convert.toIpAddress(other)
            except:
                return False
        else:
            return self.ip == other

    def __ne__(self, other):
        return not (self == other)

    def verifyIp(self):
        if self.ip < 0 or self.ip >= (1 << 32):
            raise TypeError("invalid ip address %#08x" % self.ip)

    def getValue(self):
        from _m5.net import IpAddress

        return IpAddress(self.ip)


# When initializing an IpNetmask, pass in an existing IpNetmask, a string of
# the form "a.b.c.d/n" or "a.b.c.d/e.f.g.h", or an ip and netmask as
# positional or keyword arguments.
class IpNetmask(IpAddress):
    cxx_type = "networking::IpNetmask"
    ex_str = "127.0.0.0/24"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"', add_once=True)

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal=None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError(f"No value set for {key}")

        if len(args) == 0:
            handle_kwarg(self, kwargs, "ip")
            handle_kwarg(self, kwargs, "netmask")

        elif len(args) == 1:
            if kwargs:
                if not "ip" in kwargs and not "netmask" in kwargs:
                    raise TypeError("Invalid arguments")
                handle_kwarg(self, kwargs, "ip", args[0])
                handle_kwarg(self, kwargs, "netmask", args[0])
            elif isinstance(args[0], IpNetmask):
                self.ip = args[0].ip
                self.netmask = args[0].netmask
            else:
                (self.ip, self.netmask) = convert.toIpNetmask(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.netmask = args[1]
        else:
            raise TypeError("Too many arguments specified")

        if kwargs:
            raise TypeError(f"Too many keywords: {list(kwargs.keys())}")

        self.verify()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return "%s/%d" % (super().__str__(), self.netmask)

    def __eq__(self, other):
        if isinstance(other, IpNetmask):
            return self.ip == other.ip and self.netmask == other.netmask
        elif isinstance(other, str):
            try:
                return (self.ip, self.netmask) == convert.toIpNetmask(other)
            except:
                return False
        else:
            return False

    def verify(self):
        self.verifyIp()
        if self.netmask < 0 or self.netmask > 32:
            raise TypeError("invalid netmask %d" % self.netmask)

    def getValue(self):
        from _m5.net import IpNetmask

        return IpNetmask(self.ip, self.netmask)


# When initializing an IpWithPort, pass in an existing IpWithPort, a string of
# the form "a.b.c.d:p", or an ip and port as positional or keyword arguments.
class IpWithPort(IpAddress):
    cxx_type = "networking::IpWithPort"
    ex_str = "127.0.0.1:80"
    cmd_line_settable = True

    @classmethod
    def cxx_predecls(cls, code):
        code('#include "base/inet.hh"', add_once=True)

    def __init__(self, *args, **kwargs):
        def handle_kwarg(self, kwargs, key, elseVal=None):
            if key in kwargs:
                setattr(self, key, kwargs.pop(key))
            elif elseVal:
                setattr(self, key, elseVal)
            else:
                raise TypeError(f"No value set for {key}")

        if len(args) == 0:
            handle_kwarg(self, kwargs, "ip")
            handle_kwarg(self, kwargs, "port")

        elif len(args) == 1:
            if kwargs:
                if not "ip" in kwargs and not "port" in kwargs:
                    raise TypeError("Invalid arguments")
                handle_kwarg(self, kwargs, "ip", args[0])
                handle_kwarg(self, kwargs, "port", args[0])
            elif isinstance(args[0], IpWithPort):
                self.ip = args[0].ip
                self.port = args[0].port
            else:
                (self.ip, self.port) = convert.toIpWithPort(args[0])

        elif len(args) == 2:
            self.ip = args[0]
            self.port = args[1]
        else:
            raise TypeError("Too many arguments specified")

        if kwargs:
            raise TypeError(f"Too many keywords: {list(kwargs.keys())}")

        self.verify()

    def __call__(self, value):
        self.__init__(value)
        return value

    def __str__(self):
        return "%s:%d" % (super().__str__(), self.port)

    def __eq__(self, other):
        if isinstance(other, IpWithPort):
            return self.ip == other.ip and self.port == other.port
        elif isinstance(other, str):
            try:
                return (self.ip, self.port) == convert.toIpWithPort(other)
            except:
                return False
        else:
            return False

    def verify(self):
        self.verifyIp()
        if self.port < 0 or self.port > 0xFFFF:
            raise TypeError("invalid port %d" % self.port)

    def getValue(self):
        from _m5.net import IpWithPort

        return IpWithPort(self.ip, self.port)


class NetworkBandwidth(float, ParamValue):
    cxx_type = "float"
    ex_str = "1Gbps"
    cmd_line_settable = True

    def __new__(cls, value):
        # convert to bits per second
        val = convert.toNetworkBandwidth(value)
        return super().__new__(cls, val)

    def __str__(self):
        return str(self.val)

    def __call__(self, value):
        val = convert.toNetworkBandwidth(value)
        self.__init__(val)
        return value

    def getValue(self):
        # convert to seconds per byte
        value = 8.0 / float(self)
        # convert to ticks per byte
        value = ticks.fromSeconds(value)
        return float(value)

    def ini_str(self):
        return f"{self.getValue():f}"

    def config_value(self):
        return f"{self.getValue():f}"

    @classmethod
    def cxx_ini_predecls(cls, code):
        code("#include <sstream>", add_once=True)

    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"{ret} (std::istringstream({src}) >> {dest}).eof();")
