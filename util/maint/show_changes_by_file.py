#!/usr/bin/env python3
#
# Copyright (c) 2018 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Brandon Potter


import subprocess
from collections import OrderedDict, defaultdict


class OrderedDefaultDict(OrderedDict, defaultdict):
    def __init__(self, default_factory=None, *args, **kwargs):
        super(OrderedDefaultDict, self).__init__(*args, **kwargs)
        self.default_factory = default_factory


def diff_files(upstream, feature, paths=[]):
    """Given two git branches and an optional parameter 'path', determine
    which files differ between the two branches. Afterwards, organize the
    files with a printer-friendly data structure.

    Returns: Dictionary of directories with their corresponding files
    """

    raw = subprocess.check_output(
        ["git", "diff", "--name-status", f"{upstream}..{feature}", "--"]
        + paths
    )

    path = [line.split("\t")[1] for line in raw.splitlines()]

    odd = OrderedDefaultDict(list)
    for p in path:
        direc = subprocess.check_output(["dirname", p]).strip() + "/"
        filename = subprocess.check_output(["basename", p]).strip()
        odd[direc].append(f"{filename}")

    return odd


def cl_hash(upstream, feature, path):
    """Given two git branches and full path, record the identifier hash
    for changesets which diff between the upstream branch and feature branch.
    The changesets are ordered from oldest to youngest changesets in the
    list.

    Returns: List of identifier hashes
    """

    raw = subprocess.check_output(
        ["git", "log", "--oneline", f"{upstream}..{feature}", "--", path]
    )

    return [l.split()[0] for l in raw.splitlines()]


def _main():
    import argparse

    parser = argparse.ArgumentParser(
        description="List all changes between an upstream branch and a "
        "feature branch by filename(s) and changeset hash(es)."
    )

    parser.add_argument(
        "--upstream",
        "-u",
        type=str,
        default="origin/develop",
        help="Upstream branch for comparison. Default: %(default)s",
    )
    parser.add_argument(
        "--feature",
        "-f",
        type=str,
        default="HEAD",
        help="Feature branch for comparison. Default: %(default)s",
    )
    parser.add_argument(
        "paths",
        metavar="PATH",
        type=str,
        nargs="*",
        help="Paths to list changes for",
    )

    args = parser.parse_args()

    odd = diff_files(args.upstream, args.feature, paths=args.paths)

    for key, value in odd.items():
        print(key)
        for entry in value:
            print(f"    {entry}")
            path = key + entry
            sha = cl_hash(args.upstream, args.feature, path)
            for s in sha:
                print(f"\t{s}")
        print()


if __name__ == "__main__":
    _main()
