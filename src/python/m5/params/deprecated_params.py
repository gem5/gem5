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

from ..util import warn


class DeprecatedParam:
    """A special type for deprecated parameter variable names.

    There are times when we need to change the name of parameter, but this
    breaks the external-facing python API used in configuration files. Using
    this "type" for a parameter will warn users that they are using the old
    name, but allow for backwards compatibility.

    Usage example:
    In the following example, the `time` parameter is changed to `delay`.

    ```
    class SomeDevice(SimObject):
        delay = Param.Latency('1ns', 'The time to wait before something')
        time = DeprecatedParam(delay, '`time` is now called `delay`')
    ```
    """

    def __init__(self, new_param, message=""):
        """new_param: the new parameter variable that users should be using
        instead of this parameter variable.
        message: an optional message to print when warning the user
        """
        self.message = message
        self.newParam = new_param
        # Note: We won't know the string variable names until later in the
        # SimObject initialization process. Note: we expect that the setters
        # will be called when the SimObject type (class) is initialized so
        # these variables should be filled in before the instance of the
        # SimObject with this parameter is constructed
        self._oldName = ""
        self._newName = ""

    @property
    def oldName(self):
        assert self._oldName != ""  # should already be set
        return self._oldName

    @oldName.setter
    def oldName(self, name):
        assert self._oldName == ""  # Cannot "re-set" this value
        self._oldName = name

    @property
    def newName(self):
        assert self._newName != ""  # should already be set
        return self._newName

    @newName.setter
    def newName(self, name):
        assert self._newName == ""  # Cannot "re-set" this value
        self._newName = name

    def printWarning(self, instance_name, simobj_name):
        """Issue a warning that this variable name should not be used.

        instance_name: str, the name of the instance used in python
        simobj_name: str, the name of the SimObject type
        """
        if not self.message:
            self.message = f"See {simobj_name} for more information"
        warn(f"{instance_name}.{self._oldName} is deprecated. {self.message}")
