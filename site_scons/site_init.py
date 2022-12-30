# Copyright (c) 2013, 2015-2017 ARM Limited
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

from __future__ import print_function

# Check for recent-enough Python and SCons versions.
try:
    EnsureSConsVersion(3, 0, 0)
except SystemExit as e:
    print(
        """
For more details, see:
    http://gem5.org/documentation/general_docs/building
"""
    )
    raise


# Check for the python version. Python 2 is no longer supported.
try:
    EnsurePythonVersion(3, 6)
except SystemExit as e:
    print(
        """\033[93m
Python 3 is now required.

The following are steps to compile gem5 in Python 3 environment,

*Step 1*: ensure Python 3 is installed. On Ubuntu like systems, you can try \
this command:

    sudo apt-get install python3 python-is-python3 python3-pydot

To run Python 3 from a container, you can try the Docker files in \
util/dockerfiles folder.

*Step 2*: ensure that scons is run in the Python 3 environment. If scons \
isn't run automatically with Python 3, you can force it by replacing `scons` \
by the following phrase,

    /usr/bin/env python3 $(which scons)

For example, the following command will let scons compile gem5/X86 in the \
Python 3 environment,

   /usr/bin/env python3 $(which scons) build/X86/gem5.opt

(Optional) For convenience reasons, you can set up an alias for the Python3 \
scons phrase in your environment. \033[0m
"""
    )
    raise

from gem5_python_paths import extra_python_paths

sys.path[1:1] = extra_python_paths
