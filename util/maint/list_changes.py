#!/usr/bin/env python2
#
# Copyright (c) 2017 ARM Limited
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


import subprocess
import re
from functools import wraps

class Commit(object):
    _re_tag = re.compile(r"^((?:\w|-)+): (.*)$")

    def __init__(self, rev):
        self.rev = rev
        self._log = None
        self._tags = None

    def _git(self, args):
        return subprocess.check_output([ "git", ] + args)

    @property
    def log(self):
        """Log message belonging to a commit returned as a list with on line
        per element.

        """
        if self._log is None:
            self._log = self._git(
                ["show", "--format=%B", "--no-patch", str(self.rev) ]
            ).rstrip("\n").split("\n")
        return self._log

    @property
    def tags(self):
        """Get all commit message tags in the current commit.

        Returns: { tag, [ value, ... ] }

        """
        if self._tags is None:
            tags = {}
            for l in self.log[1:]:
                m = Commit._re_tag.match(l)
                if m:
                    key, value = m.group(1), m.group(2)
                    try:
                        tags[key].append(value)
                    except KeyError:
                        tags[key] = [ value ]
            self._tags = tags

        return self._tags

    @property
    def change_id(self):
        """Get the Change-Id tag from the commit

        Returns: A change ID or None if no change ID has been
        specified.

        """
        try:
            cids = self.tags["Change-Id"]
        except KeyError:
            return None

        assert len(cids) == 1
        return cids[0]

    def __str__(self):
        return "%s: %s" % (self.rev[0:8], self.log[0])

def list_revs(upstream, branch):
    """Get a generator that lists git revisions that exist in 'branch' but
    not in 'upstream'.

    Returns: Generator of Commit objects

    """

    changes = subprocess.check_output(
        [ "git", "rev-list", "%s..%s" % (upstream, branch) ])

    if changes == "":
        return

    for rev in changes.rstrip("\n").split("\n"):
        assert rev != ""
        yield Commit(rev)

def list_changes(upstream, feature):
    feature_revs = tuple(list_revs(upstream, feature))
    upstream_revs = tuple(list_revs(feature, upstream))

    feature_cids = dict([
        (c.change_id, c) for c in feature_revs if c.change_id is not None ])
    upstream_cids = dict([
        (c.change_id, c) for c in upstream_revs if c.change_id is not None ])

    incoming = filter(
        lambda r: r.change_id and r.change_id not in feature_cids,
        reversed(upstream_revs))
    outgoing = filter(
        lambda r: r.change_id and r.change_id not in upstream_cids,
        reversed(feature_revs))
    common = filter(
        lambda r: r.change_id in upstream_cids,
        reversed(feature_revs))
    upstream_unknown = filter(
        lambda r: r.change_id is None,
        reversed(upstream_revs))
    feature_unknown = filter(
        lambda r: r.change_id is None,
        reversed(feature_revs))

    return incoming, outgoing, common, upstream_unknown, feature_unknown

def _main():
    import argparse
    parser = argparse.ArgumentParser(
        description="List incoming and outgoing changes in a feature branch")

    parser.add_argument("--upstream", "-u", type=str, default="origin/master",
                        help="Upstream branch for comparison. " \
                        "Default: %(default)s")
    parser.add_argument("--feature", "-f", type=str, default="HEAD",
                        help="Feature branch for comparison. " \
                        "Default: %(default)s")
    parser.add_argument("--show-unknown", action="store_true",
                        help="Print changes without Change-Id tags")
    parser.add_argument("--show-common", action="store_true",
                        help="Print common changes")

    args = parser.parse_args()

    incoming, outgoing, common, upstream_unknown, feature_unknown = \
        list_changes(args.upstream, args.feature)

    if incoming:
        print "Incoming changes:"
        for rev in incoming:
            print rev
        print

    if args.show_unknown and upstream_unknown:
        print "Upstream changes without change IDs:"
        for rev in upstream_unknown:
            print rev
        print

    if outgoing:
        print "Outgoing changes:"
        for rev in outgoing:
            print rev
        print

    if args.show_common and common:
        print "Common changes:"
        for rev in common:
            print rev
        print

    if args.show_unknown and feature_unknown:
        print "Outgoing changes without change IDs:"
        for rev in feature_unknown:
            print rev



if __name__ == "__main__":
    _main()
