#! /usr/bin/env python2
#
# Copyright (c) 2016 ARM Limited
# All rights reserved
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
# Authors: Andreas Sandberg

import os
import sys

from style.file_types import lang_type
import style.verifiers
from style.region import all_regions

from style.style import StdioUI
from style import repo

verifier_names = dict([
    (c.__name__, c) for c in style.verifiers.all_verifiers ])

def verify(filename, regions=all_regions, verbose=False, verifiers=None,
           auto_fix=False):
    ui = StdioUI()
    opts = {
        "fix_all" : auto_fix,
    }
    base = os.path.join(os.path.dirname(__file__), "..")
    if verifiers is None:
        verifiers = style.verifiers.all_verifiers

    if verbose:
        print "Verifying %s[%s]..." % (filename, regions)
    for verifier in [ v(ui, opts, base=base) for v in verifiers ]:
        if verbose:
            print "Applying %s (%s)" % (
                verifier.test_name, verifier.__class__.__name__)
        if verifier.apply(filename, regions=regions):
            return False
    return True

def detect_repo():
    repo_classes = repo.detect_repo()
    if not repo_classes:
        print >> sys.stderr, "Error: Failed to detect repository type, no " \
            "known repository type found."
        sys.exit(1)
    elif len(repo_classes) > 1:
        print >> sys.stderr, "Error: Detected multiple repository types."
        sys.exit(1)
    else:
        return repo_classes[0]()

repo_types = {
    "auto" : detect_repo,
    "none" : lambda : None,
    "git" : repo.GitRepo,
    "hg" : repo.MercurialRepo,
}

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description="Check a file for gem5 style violations",
        epilog="""If no files are specified, the style checker tries to
        determine the list of modified and added files from the version
        control system and checks those."""
    )

    parser.add_argument("--verbose", "-v", action="count",
                        help="Produce verbose output")

    parser.add_argument("--fix", "-f", action="store_true",
                        help="Automatically fix style violations.")

    parser.add_argument("--modifications", "-m", action="store_true",
                        help="""Apply the style checker to modified regions
                        instead of whole files""")

    parser.add_argument("--repo-type", choices=repo_types, default="auto",
                        help="Repository type to use to detect changes")

    parser.add_argument("--checker", "-c", choices=verifier_names, default=[],
                        action="append",
                        help="""Style checkers to run. Can be specified
                        multiple times.""")

    parser.add_argument("files", metavar="FILE", nargs="*",
                        type=str,
                        help="Source file(s) to inspect")

    args = parser.parse_args()

    repo = repo_types[args.repo_type]()

    verifiers = [ verifier_names[name] for name in args.checker ] \
                if args.checker else None

    files = args.files
    if not files and repo:
        added, modified = repo.staged_files()
        files = [ repo.file_path(f) for f in added + modified ]

    for filename in files:
        if args.modifications and repo and repo.in_repo(filename):
            regions = repo.modified_regions(filename)
        else:
            regions = all_regions

        if not verify(filename, regions=regions,
                      verbose=args.verbose,
                      verifiers=verifiers,
                      auto_fix=args.fix):
            sys.exit(1)
