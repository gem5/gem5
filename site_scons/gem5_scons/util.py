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
import itertools
import re
import sys

import m5.util.terminal
import SCons.Script


def ignore_style():
    """Determine whether we should ignore style checks"""
    return SCons.Script.GetOption("ignore_style") or not sys.stdin.isatty()


def get_termcap():
    return m5.util.terminal.get_termcap(SCons.Script.GetOption("use_colors"))


def readCommand(cmd, **kwargs):
    """
    run the command cmd, read the results and return them
    this is sorta like `cmd` in shell

    :param cmd: command to run with Popen
    :type cmd: string, list
    :returns: command stdout
    :rtype: string
    """
    from subprocess import Popen, PIPE, STDOUT

    if isinstance(cmd, str):
        cmd = cmd.split()

    no_exception = "exception" in kwargs
    exception = kwargs.pop("exception", None)

    kwargs.setdefault("shell", False)
    kwargs.setdefault("stdout", PIPE)
    kwargs.setdefault("stderr", STDOUT)
    kwargs.setdefault("close_fds", True)
    try:
        subp = Popen(cmd, **kwargs)
    except Exception as e:
        if no_exception:
            return -1, exception
        raise

    output = subp.communicate()[0].decode("utf-8")
    return output


def compareVersions(v1, v2):
    """helper function: compare arrays or strings of version numbers.
    E.g., compare_version((1,3,25), (1,4,1)')
    returns -1, 0, 1 if v1 is <, ==, > v2
    """

    def make_version_list(v):
        if isinstance(v, (list, tuple)):
            return v
        elif isinstance(v, str):
            return list(
                map(lambda x: int(re.match("\d+", x).group()), v.split(".")),
            )
        else:
            raise TypeError()

    v1 = make_version_list(v1)
    v2 = make_version_list(v2)

    # Compare corresponding elements of lists
    # The shorter list is filled with 0 till the lists have the same length
    for n1, n2 in itertools.zip_longest(v1, v2, fillvalue=0):
        if n1 < n2:
            return -1
        if n1 > n2:
            return 1

    return 0
