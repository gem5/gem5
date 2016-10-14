# Copyright (c) 2012 ARM Limited
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
#
# Authors: Andreas Sandberg

from abc import ABCMeta, abstractmethod
import m5
from m5.objects import *
from m5.proxy import *
m5.util.addToPath('../configs/')
from common import FSConfig
from common.Caches import *
from base_config import *

class LinuxAlphaSystemBuilder(object):
    """Mix-in that implements create_system.

    This mix-in is intended as a convenient way of adding an
    Alpha-specific create_system method to a class deriving from one of
    the generic base systems.
    """
    def __init__(self):
        """
        Arguments:
          machine_type -- String describing the platform to simulate
        """
        pass

    def create_system(self):
        system = FSConfig.makeLinuxAlphaSystem(self.mem_mode)
        self.init_system(system)
        return system

class LinuxAlphaFSSystem(LinuxAlphaSystemBuilder,
                         BaseFSSystem):
    """Basic Alpha full system builder."""

    def __init__(self, **kwargs):
        """Initialize an Alpha system that supports full system simulation.

        Note: Keyword arguments that are not listed below will be
        passed to the BaseFSSystem.

        Keyword Arguments:
          -
        """
        BaseSystem.__init__(self, **kwargs)
        LinuxAlphaSystemBuilder.__init__(self)

class LinuxAlphaFSSystemUniprocessor(LinuxAlphaSystemBuilder,
                                     BaseFSSystemUniprocessor):
    """Basic Alpha full system builder for uniprocessor systems.

    Note: This class is a specialization of the AlphaFSSystem and is
    only really needed to provide backwards compatibility for existing
    test cases.
    """

    def __init__(self, **kwargs):
        BaseFSSystemUniprocessor.__init__(self, **kwargs)
        LinuxAlphaSystemBuilder.__init__(self)

class LinuxAlphaFSSwitcheroo(LinuxAlphaSystemBuilder, BaseFSSwitcheroo):
    """Uniprocessor Alpha system prepared for CPU switching"""

    def __init__(self, **kwargs):
        BaseFSSwitcheroo.__init__(self, **kwargs)
        LinuxAlphaSystemBuilder.__init__(self)
