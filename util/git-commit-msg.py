#!/usr/bin/env python
#
# Copyright (c) 2019 Inria
# All rights reserved
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

# If your commit has been canceled because of this file, do not panic: a
# copy of it has been stored in ./.git/COMMIT_EDITMSG

import os
import re
import sys

from style.repo import GitRepo

def _printErrorQuit(error_message):
    """
        Print an error message, followed my a help message and inform failure.

        @param error_message A message describing the error that caused the
            failure.
    """
    print(error_message)

    print("The commit has been cancelled, but a copy of it can be found in "
            + sys.argv[1] + " : ")

    print("""
--------------------------------------------------------------------------
    """)
    print(open(sys.argv[1], "r").read())
    print("""
--------------------------------------------------------------------------
    """)

    print("""
The first line of a commit must contain one or more gem5 tags separated by
commas (see MAINTAINERS for the possible tags), followed by a colon and a
commit title. There must be no leading nor trailing whitespaces.

This header line must then be followed by an empty line. A detailed message,
although highly recommended, is not mandatory and can follow that empty line.

e.g.:
    cpu: Refactor branch predictors

    Refactor branch predictor code to improve its readability, moving functions
    X and Y to the base class...

e.g.:
    mem,mem-cache: Improve packet class readability

    The packet class...
""")
    sys.exit(1)

def _validateTags(commit_header):
    """
        Check if all tags in the commit header belong to the list of valid
        gem5 tags.

        @param commit_header The first line of the commit message.
    """

    # List of valid tags
    # @todo this is error prone, and should be extracted automatically from
    #       a file

    valid_tags = ["arch", "arch-alpha", "arch-arm", "arch-gcn3", "arch-hsail",
        "arch-mips", "arch-power", "arch-riscv", "arch-sparc", "arch-x86",
        "base", "configs", "cpu", "cpu-kvm", "cpu-minor", "cpu-o3",
        "cpu-simple", "dev", "dev-arm", "dev-virtio", "ext", "fastmodel",
        "gpu-compute", "learning-gem5", "mem", "mem-cache", "mem-garnet",
        "mem-ruby", "misc", "python", "scons", "sim", "sim-se", "sim-power",
        "stats", "system", "system-alpha", "system-arm", "systemc", "tests",
        "util", "RFC", "WIP"]

    tags = ''.join(commit_header.split(':')[0].split()).split(',')
    if (any(tag not in valid_tags for tag in tags)):
        invalid_tag = next((tag for tag in tags if tag not in valid_tags))
        _printErrorQuit("Invalid Gem5 tag: " + invalid_tag)

# Go to git directory
os.chdir(GitRepo().repo_base())

# Get the commit message
commit_message = open(sys.argv[1]).read()

# The first line of a commit must contain at least one valid gem5 tag, and
# a commit title
commit_message_lines = commit_message.splitlines()
commit_header = commit_message_lines[0]
commit_header_match = \
    re.search("^(\S[\w\-][,\s*[\w\-]+]*:.+\S$)", commit_header)
if ((commit_header_match is None)):
    _printErrorQuit("Invalid commit header")
_validateTags(commit_header)

# Make sure commit title does not exceed threshold. This line is limited to
# a smaller number because version control systems may add a prefix, causing
# line-wrapping for longer lines
commit_title = commit_header.split(':')[1]
max_header_size = 65
if (len(commit_header) > max_header_size):
    _printErrorQuit("The commit header (tags + title) is too long (" + \
        str(len(commit_header)) + " > " + str(max_header_size) + ")")

# Then there must be at least one empty line between the commit header and
# the commit description
if (commit_message_lines[1] != ""):
    _printErrorQuit("Please add an empty line between the commit title and " \
        "its description")

# Encourage providing descriptions
if (re.search("^(Signed-off-by|Change-Id|Reviewed-by):",
    commit_message_lines[2])):
    print("Warning: Commit does not have a description")

sys.exit(0)
