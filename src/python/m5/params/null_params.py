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

from ..util import Singleton


# Special class for NULL pointers.  Note the special check in
# make_param_value() above that lets these be assigned where a
# SimObject is required.
# only one copy of a particular node
class NullSimObject(metaclass=Singleton):
    _name = "Null"

    def __call__(cls):
        return cls

    def _instantiate(self, parent=None, path=""):
        pass

    def ini_str(self):
        return "Null"

    def unproxy(self, base):
        return self

    def set_path(self, parent, name):
        pass

    def set_parent(self, parent, name):
        pass

    def clear_parent(self, old_parent):
        pass

    def descendants(self):
        return
        yield None

    def get_config_as_dict(self):
        return {}

    def __str__(self):
        return self._name

    def config_value(self):
        return None

    def getValue(self):
        return None


# The only instance you'll ever need...
NULL = NullSimObject()


def isNullPointer(value):
    return isinstance(value, NullSimObject)


class NullOptT(metaclass=Singleton):
    _name = "NullOpt"

    def __call__(cls):
        return cls

    def ini_str(self):
        return "NullOpt"

    def unproxy(self, base):
        return self

    def __str__(self):
        return self._name

    def config_value(self):
        return None

    def getValue(self):
        return None


NullOpt = NullOptT()


def isNullOpt(value):
    return isinstance(value, NullOptT)
