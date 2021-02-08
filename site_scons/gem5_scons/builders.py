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

import os
import sys

from os.path import join, split

# SCons includes
import SCons
import SCons.Node
import SCons.Node.FS
import SCons.Script

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
        val = env[variable]
        if isinstance(val, bool):
            # Force value to 0/1
            val = int(val)
        elif isinstance(val, str):
            val = '"' + val + '"'

        # Sources are variable name & value (packaged in SCons Value nodes)
        return [target], [env.Value(variable), env.Value(val)]

    config_builder = env.Builder(emitter=config_emitter, action=config_action)

    env.Append(BUILDERS = { 'ConfigFile' : config_builder })

def AddLocalRPATH(env):
    def add_local_rpath(env, *targets):
        '''Set up an RPATH for a library which lives in the build directory.

        The construction environment variable BIN_RPATH_PREFIX should be set
        to the relative path of the build directory starting from the location
        of the binary.'''
        for target in targets:
            target = env.Entry(target)
            if not isinstance(target, SCons.Node.FS.Dir):
                target = target.dir
            relpath = os.path.relpath(target.abspath, env['BUILDDIR'])
            components = [
                '\\$$ORIGIN',
                '${BIN_RPATH_PREFIX}',
                relpath
            ]
            env.Append(RPATH=[env.Literal(os.path.join(*components))])

    if sys.platform != "darwin":
        env.Append(LINKFLAGS=env.Split('-z origin'))

    env.AddMethod(add_local_rpath, 'AddLocalRPATH')

###################################################
#
# This builder and wrapper method are used to set up a directory with
# switching headers. Those are headers which are in a generic location and
# that include more specific headers from a directory chosen at build time
# based on the current build settings.
#
###################################################

def SwitchingHeaders(env):
    def build_switching_header(target, source, env):
        path = str(target[0])
        subdir = str(source[0])
        dp, fp = os.path.split(path)
        dp = os.path.relpath(os.path.realpath(dp),
                             os.path.realpath(env['BUILDDIR']))
        with open(path, 'w') as hdr:
            print('#include "%s/%s/%s"' % (dp, subdir, fp), file=hdr)

    switching_header_action = MakeAction(build_switching_header,
                                         Transform('GENERATE'))

    switching_header_builder = env.Builder(action=switching_header_action,
                                           source_factory=env.Value,
                                           single_source=True)

    env.Append(BUILDERS = { 'SwitchingHeader': switching_header_builder })

    def switching_headers(self, headers, source):
        for header in headers:
            self.SwitchingHeader(header, source)

    env.AddMethod(switching_headers, 'SwitchingHeaders')
