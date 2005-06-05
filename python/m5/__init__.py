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

import sys, os

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

# Add given directory to system module search path, if it is not
# already there.
def AddToPath(path):
    path = os.path.realpath(path)
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

# find the m5 compile options: must be specified as a dict in
# __main__.m5_build_env.
import __main__
if not hasattr(__main__, 'm5_build_env'):
    panic("__main__ must define m5_build_env")

# make a SmartDict out of the build options for our local use
import smartdict
build_env = smartdict.SmartDict()
build_env.update(__main__.m5_build_env)

# make a SmartDict out of the OS environment too
env = smartdict.SmartDict()
env.update(os.environ)

# import the main m5 config code
from config import *

# import the built-in object definitions
from objects import *

