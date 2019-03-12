# Copyright 2018 Google, Inc.
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
# Authors: Gabe Black

from m5.SimObject import SimObject, cxxMethod

# This class represents the systemc kernel. There should be exactly one in the
# simulation. It receives gem5 SimObject lifecycle callbacks (init, regStats,
# etc.) and manages the lifecycle of the systemc simulation accordingly.
class SystemC_Kernel(SimObject):
    type = 'SystemC_Kernel'
    cxx_class = 'sc_gem5::Kernel'
    cxx_header = 'systemc/core/kernel.hh'

# This class represents systemc sc_object instances in python config files. It
# inherits from SimObject in python, but the c++ version, sc_core::sc_object,
# doesn't inherit from gem5's c++ SimObject class.
class SystemC_ScObject(SimObject):
    type = 'SystemC_ScObject'
    abstract = True
    cxx_class = 'sc_core::sc_object'
    cxx_header = 'systemc/ext/core/sc_object.hh'

    # Clear cxx_base to stop the c++ binding code from assuming
    # sc_core::sc_object inherits from SimObject, even though SystemC_ScObject
    # does on the python side.
    cxx_base = None

    # Hide the cxx_exports from SimObject since we don't inherit from
    # SimObject on the c++ side and so don't have those methods to call down
    # into.
    locals().update({
        method.name: (lambda *a, **k: None) for method in SimObject.cxx_exports
    })

class SystemC_ScModule(SystemC_ScObject):
    type = 'SystemC_ScModule'
    abstract = True
    cxx_class = 'sc_core::sc_module'
    cxx_header = 'systemc/ext/core/sc_module.hh'

    @cxxMethod(return_value_policy="reference", cxx_name="gem5_getPort")
    def getPort(self, if_name, iex):
        return None

try:
    import _m5
except:
    pass
else:
    import _m5.systemc
    _m5.systemc.python_ready()
