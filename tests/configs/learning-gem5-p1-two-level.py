# -*- coding: utf-8 -*-
# Copyright (c) 2015 Jason Lowe-Power
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
# Authors: Jason Lowe-Power

# A wrapper around configs/learning_gem5/part1/two_level.py

# For some reason, this is implicitly needed by run.py
root = None

import m5

def run_test(root):
        # Called from tests/run.py


        # Set the working directory in case we are executing from
        # outside gem5's source tree
        import os
        os.chdir(os.path.join(os.path.dirname(__file__), "../"))

        # Add paths that we need
        m5.util.addToPath('../configs/learning_gem5/part1')
        m5.util.addToPath('../configs/common')

        # The path to this script is the only parameter. Delete it so we can
        # execute the script that we want to execute.
        import sys
        del sys.argv[1:]
        # Note: at this point, we could add options we want to test.
        # For instance, sys.argv.append('--l2_size=512kB')

        # Execute the script we are wrapping
        execfile(srcpath('configs/learning_gem5/part1/two_level.py'))
