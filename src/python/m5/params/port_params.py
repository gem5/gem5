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

import copy

from .. import proxy
from ..util import (
    Singleton,
    fatal,
)
from .base_params import (
    isSimObject,
    isSimObjectClass,
)

#####################################################################
#
# Port objects
#
# Ports are used to interconnect objects in the memory system.
#
#####################################################################


# Port reference: encapsulates a reference to a particular port on a
# particular SimObject.
class PortRef:
    def __init__(self, simobj, name, role, is_source):
        assert isSimObject(simobj) or isSimObjectClass(simobj)
        self.simobj = simobj
        self.name = name
        self.role = role
        self.is_source = is_source
        self.peer = None  # not associated with another port yet
        self.ccConnected = False  # C++ port connection done?
        self.index = -1  # always -1 for non-vector ports

    def __str__(self):
        return f"{self.simobj}.{self.name}"

    def __len__(self):
        # Return the number of connected ports, i.e. 0 is we have no
        # peer and 1 if we do.
        return int(self.peer != None)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return str(self.peer)

    # for config.json
    def get_config_as_dict(self):
        return {
            "role": self.role,
            "peer": str(self.peer),
            "is_source": str(self.is_source),
        }

    def __getattr__(self, attr):
        if attr == "peerObj":
            # shorthand for proxies
            return self.peer.simobj
        raise AttributeError(
            f"'{self.__class__.__name__}' object has no attribute '{attr}'"
        )

    # Full connection is symmetric (both ways).  Called via
    # SimObject.__setattr__ as a result of a port assignment, e.g.,
    # "obj1.portA = obj2.portB", or via VectorPortElementRef.__setitem__,
    # e.g., "obj1.portA[3] = obj2.portB".
    def connect(self, other):
        if isinstance(other, VectorPortRef):
            # reference to plain VectorPort is implicit append
            other = other._get_next()
        if self.peer and not proxy.isproxy(self.peer):
            fatal(
                "Port %s is already connected to %s, cannot connect %s\n",
                self,
                self.peer,
                other,
            )
        self.peer = other

        if proxy.isproxy(other):
            other.set_param_desc(PortParamDesc())
            return
        elif not isinstance(other, PortRef):
            raise TypeError(
                f"assigning non-port reference '{other}' to port '{self}'"
            )

        if not Port.is_compat(self, other):
            fatal(
                "Ports %s and %s with roles '%s' and '%s' "
                "are not compatible",
                self,
                other,
                self.role,
                other.role,
            )

        if other.peer is not self:
            other.connect(self)

    # Allow a compatible port pair to be spliced between a port and its
    # connected peer. Useful operation for connecting instrumentation
    # structures into a system when it is necessary to connect the
    # instrumentation after the full system has been constructed.
    def splice(self, new_1, new_2):
        if not self.peer or proxy.isproxy(self.peer):
            fatal("Port %s not connected, cannot splice in new peers\n", self)

        if not isinstance(new_1, PortRef) or not isinstance(new_2, PortRef):
            raise TypeError(
                f"Splicing non-port references '{new_1}','{new_2}' to port '{self}'"
            )

        old_peer = self.peer

        if Port.is_compat(old_peer, new_1) and Port.is_compat(self, new_2):
            old_peer.peer = new_1
            new_1.peer = old_peer
            self.peer = new_2
            new_2.peer = self
        elif Port.is_compat(old_peer, new_2) and Port.is_compat(self, new_1):
            old_peer.peer = new_2
            new_2.peer = old_peer
            self.peer = new_1
            new_1.peer = self
        else:
            fatal(
                "Ports %s(%s) and %s(%s) can't be compatibly spliced with "
                "%s(%s) and %s(%s)",
                self,
                self.role,
                old_peer,
                old_peer.role,
                new_1,
                new_1.role,
                new_2,
                new_2.role,
            )

    def clone(self, simobj, memo):
        if self in memo:
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert isSimObject(newRef.simobj)
        if self.peer and not proxy.isproxy(self.peer):
            peerObj = self.peer.simobj(_memo=memo)
            newRef.peer = self.peer.clone(peerObj, memo)
            assert not isinstance(newRef.peer, VectorPortRef)
        return newRef

    def unproxy(self, simobj):
        assert simobj is self.simobj
        if proxy.isproxy(self.peer):
            try:
                realPeer = self.peer.unproxy(self.simobj)
            except:
                print(
                    f"Error in unproxying port '{self.name}' of {self.simobj.path()}"
                )
                raise
            self.connect(realPeer)

    # Call C++ to create corresponding port connection between C++ objects
    def ccConnect(self):
        if self.ccConnected:  # already done this
            return

        peer = self.peer
        if not self.peer:  # nothing to connect to
            return

        port = self.simobj.getPort(self.name, self.index)
        peer_port = peer.simobj.getPort(peer.name, peer.index)
        port.bind(peer_port)

        self.ccConnected = True


# A reference to an individual element of a VectorPort... much like a
# PortRef, but has an index.
class VectorPortElementRef(PortRef):
    def __init__(self, simobj, name, role, is_source, index):
        PortRef.__init__(self, simobj, name, role, is_source)
        self.index = index

    def __str__(self):
        return "%s.%s[%d]" % (self.simobj, self.name, self.index)


# A reference to a complete vector-valued port (not just a single element).
# Can be indexed to retrieve individual VectorPortElementRef instances.
class VectorPortRef:
    def __init__(self, simobj, name, role, is_source):
        assert isSimObject(simobj) or isSimObjectClass(simobj)
        self.simobj = simobj
        self.name = name
        self.role = role
        self.is_source = is_source
        self.elements = []

    def __str__(self):
        return f"{self.simobj}.{self.name}[:]"

    def __len__(self):
        # Return the number of connected peers, corresponding the the
        # length of the elements.
        return len(self.elements)

    # for config.ini, print peer's name (not ours)
    def ini_str(self):
        return " ".join([el.ini_str() for el in self.elements])

    # for config.json
    def get_config_as_dict(self):
        return {
            "role": self.role,
            "peer": [el.ini_str() for el in self.elements],
            "is_source": str(self.is_source),
        }

    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError("VectorPort index must be integer")
        if key >= len(self.elements):
            # need to extend list
            ext = [
                VectorPortElementRef(
                    self.simobj, self.name, self.role, self.is_source, i
                )
                for i in range(len(self.elements), key + 1)
            ]
            self.elements.extend(ext)
        return self.elements[key]

    def _get_next(self):
        return self[len(self.elements)]

    def __setitem__(self, key, value):
        if not isinstance(key, int):
            raise TypeError("VectorPort index must be integer")
        self[key].connect(value)

    def connect(self, other):
        if isinstance(other, (list, tuple)):
            # Assign list of port refs to vector port.
            # For now, append them... not sure if that's the right semantics
            # or if it should replace the current vector.
            for ref in other:
                self._get_next().connect(ref)
        else:
            # scalar assignment to plain VectorPort is implicit append
            self._get_next().connect(other)

    def clone(self, simobj, memo):
        if self in memo:
            return memo[self]
        newRef = copy.copy(self)
        memo[self] = newRef
        newRef.simobj = simobj
        assert isSimObject(newRef.simobj)
        newRef.elements = [el.clone(simobj, memo) for el in self.elements]
        return newRef

    def unproxy(self, simobj):
        [el.unproxy(simobj) for el in self.elements]

    def ccConnect(self):
        [el.ccConnect() for el in self.elements]


# Port description object.  Like a ParamDesc object, this represents a
# logical port in the SimObject class, not a particular port on a
# SimObject instance.  The latter are represented by PortRef objects.
class Port:
    # Port("role", "description")

    _compat_dict = {}

    @classmethod
    def compat(cls, role, peer):
        cls._compat_dict.setdefault(role, set()).add(peer)
        cls._compat_dict.setdefault(peer, set()).add(role)

    @classmethod
    def is_compat(cls, one, two):
        for port in one, two:
            if not port.role in Port._compat_dict:
                fatal("Unrecognized role '%s' for port %s\n", port.role, port)
        return one.role in Port._compat_dict[two.role]

    def __init__(self, role, desc, is_source=False):
        self.desc = desc
        self.role = role
        self.is_source = is_source

    # Generate a PortRef for this port on the given SimObject with the
    # given name
    def makeRef(self, simobj):
        return PortRef(simobj, self.name, self.role, self.is_source)

    # Connect an instance of this port (on the given SimObject with
    # the given name) with the port described by the supplied PortRef
    def connect(self, simobj, ref):
        self.makeRef(simobj).connect(ref)

    # No need for any pre-declarations at the moment as we merely rely
    # on an unsigned int.
    def cxx_predecls(self, code):
        pass

    def pybind_predecls(self, code):
        cls.cxx_predecls(self, code)

    # Declare an unsigned int with the same name as the port, that
    # will eventually hold the number of connected ports (and thus the
    # number of elements for a VectorPort).
    def cxx_decl(self, code):
        code("unsigned int port_${{self.name}}_connection_count;")


Port.compat("GEM5 REQUESTOR", "GEM5 RESPONDER")


class RequestPort(Port):
    # RequestPort("description")
    def __init__(self, desc):
        super().__init__("GEM5 REQUESTOR", desc, is_source=True)


class ResponsePort(Port):
    # ResponsePort("description")
    def __init__(self, desc):
        super().__init__("GEM5 RESPONDER", desc)


# VectorPort description object.  Like Port, but represents a vector
# of connections (e.g., as on a XBar).
class VectorPort(Port):
    def makeRef(self, simobj):
        return VectorPortRef(simobj, self.name, self.role, self.is_source)


class VectorRequestPort(VectorPort):
    # VectorRequestPort("description")
    def __init__(self, desc):
        super().__init__("GEM5 REQUESTOR", desc, is_source=True)


class VectorResponsePort(VectorPort):
    # VectorResponsePort("description")
    def __init__(self, desc):
        super().__init__("GEM5 RESPONDER", desc)


# Old names, maintained for compatibility.
MasterPort = RequestPort
SlavePort = ResponsePort
VectorMasterPort = VectorRequestPort
VectorSlavePort = VectorResponsePort


# 'Fake' ParamDesc for Port references to assign to the _pdesc slot of
# proxy objects (via set_param_desc()) so that proxy error messages
# make sense.
class PortParamDesc(metaclass=Singleton):
    ptype_str = "Port"
    ptype = Port
