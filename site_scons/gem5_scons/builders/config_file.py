# Copyright (c) 2013, 2015-2020 ARM Limited
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
# Copyright (c) 2011 Advanced Micro Devices, Inc.
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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

from gem5_scons import Transform, MakeAction

###################################################
#
# Define a SCons builder for configuration flag headers.
#
###################################################

def ConfigFile(env):
    # This function generates a config header file that #defines the
    # variable symbol to the current variable setting (0 or 1).  The source
    # operands are the name of the variable and a Value node containing the
    # value of the variable.
    def build_config_file(target, source, env):
        (variable, value) = [s.get_contents().decode('utf-8') for s in source]
        with open(str(target[0].abspath), 'w') as f:
            print('#define', variable, value, file=f)
        return None

    # Combine the two functions into a scons Action object.
    config_action = MakeAction(build_config_file, Transform("CONFIG H", 2))

    # The emitter munges the source & target node lists to reflect what
    # we're really doing.
    def config_emitter(target, source, env):
        # extract variable name from Builder arg
        variable = str(target[0])
        # True target is config header file
        target = env.Dir('config').File(variable.lower() + '.hh')
        val = env['CONF'][variable]
        if isinstance(val, bool):
            # Force value to 0/1
            val = str(int(val))
        elif isinstance(val, str):
            val = '"' + val + '"'

        # Sources are variable name & value (packaged in SCons Value nodes)
        return [target], [env.Value(variable), env.Value(val)]

    config_builder = env.Builder(emitter=config_emitter, action=config_action)

    env.Append(BUILDERS = { 'ConfigFile' : config_builder })
