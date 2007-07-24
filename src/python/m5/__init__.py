# Copyright (c) 2005 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert
#          Steve Reinhardt

import os
import sys

import smartdict

# define a MaxTick parameter
MaxTick = 2**63 - 1

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

# force scalars to one-element lists for uniformity
def makeList(objOrList):
    if isinstance(objOrList, list):
        return objOrList
    return [objOrList]

# Prepend given directory to system module search path.  We may not
# need this anymore if we can structure our config library more like a
# Python package.
def AddToPath(path):
    # if it's a relative path and we know what directory the current
    # python script is in, make the path relative to that directory.
    if not os.path.isabs(path) and sys.path[0]:
        path = os.path.join(sys.path[0], path)
    path = os.path.realpath(path)
    # sys.path[0] should always refer to the current script's directory,
    # so place the new dir right after that.
    sys.path.insert(1, path)

# make a SmartDict out of the build options for our local use
build_env = smartdict.SmartDict()

# make a SmartDict out of the OS environment too
env = smartdict.SmartDict()
env.update(os.environ)

# Since we have so many mutual imports in this package, we should:
# 1. Put all intra-package imports at the *bottom* of the file, unless
#    they're absolutely needed before that (for top-level statements
#    or class attributes).  Imports of "trivial" packages that don't
#    import other packages (e.g., 'smartdict') can be at the top.
# 2. Never use 'from foo import *' on an intra-package import since
#    you can get the wrong result if foo is only partially imported
#    at the point you do that (i.e., because foo is in the middle of
#    importing *you*).
try:
    import internal
    running_m5 = True
except ImportError:
    running_m5 = False

if running_m5:
    from event import *
    from simulate import *
    from main import options

if running_m5:
    import defines
    build_env.update(defines.m5_build_env)
else:
    import __scons
    build_env.update(__scons.m5_build_env)

import SimObject
import params
import objects
