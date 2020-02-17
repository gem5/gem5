#! /usr/bin/env python2.7
# Copyright (c) 2014 ARM Limited
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
# Copyright (c) 2006 The Regents of The University of Michigan
# Copyright (c) 2007,2011 The Hewlett-Packard Development Company
# Copyright (c) 2016 Advanced Micro Devices, Inc.
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

import sys
import os
from os.path import join as joinpath

current_dir = os.path.dirname(__file__)
sys.path.insert(0, current_dir)

from style.verifiers import all_verifiers
from style.style import MercurialUI, check_ignores
from style.region import *

from mercurial import bdiff, mdiff, commands

def modified_regions(old_data, new_data):
    regions = Regions()
    beg = None
    for pbeg, pend, fbeg, fend in bdiff.blocks(old_data, new_data):
        if beg is not None and beg != fbeg:
            regions.append(beg, fbeg)
        beg = fend
    return regions

def modregions(wctx, fname):
    fctx = wctx.filectx(fname)
    pctx = fctx.parents()

    file_data = fctx.data()
    lines = mdiff.splitnewlines(file_data)
    if len(pctx) in (1, 2):
        mod_regions = modified_regions(pctx[0].data(), file_data)
        if len(pctx) == 2:
            m2 = modified_regions(pctx[1].data(), file_data)
            # only the lines that are new in both
            mod_regions &= m2
    else:
        mod_regions = Regions()
        mod_regions.append(0, len(lines))

    return mod_regions


def _modified_regions(repo, patterns, **kwargs):
    opt_all = kwargs.get('all', False)
    opt_no_ignore = kwargs.get('no_ignore', False)

    # Import the match (repository file name matching helper)
    # function. Different versions of Mercurial keep it in different
    # modules and implement them differently.
    try:
        from mercurial import scmutil
        m = scmutil.match(repo[None], patterns, kwargs)
    except ImportError:
        from mercurial import cmdutil
        m = cmdutil.match(repo, patterns, kwargs)

    modified, added, removed, deleted, unknown, ignore, clean = \
        repo.status(match=m, clean=opt_all)

    if not opt_all:
        try:
            wctx = repo.workingctx()
        except:
            from mercurial import context
            wctx = context.workingctx(repo)

        files = [ (fn, all_regions) for fn in added ] + \
            [ (fn,  modregions(wctx, fn)) for fn in modified ]
    else:
        files = [ (fn, all_regions) for fn in added + modified + clean ]

    for fname, mod_regions in files:
        if opt_no_ignore or not check_ignores(fname):
            yield fname, mod_regions


def do_check_style(hgui, repo, *pats, **opts):
    """check files for proper m5 style guidelines

    Without an argument, checks all modified and added files for gem5
    coding style violations. A list of files can be specified to limit
    the checker to a subset of the repository. The style rules are
    normally applied on a diff of the repository state (i.e., added
    files are checked in their entirety while only modifications of
    modified files are checked).

    The --all option can be specified to include clean files and check
    modified files in their entirety.

    The --fix-<check>, --ignore-<check>, and --skip-<check> options
    can be used to control individual style checks:

    --fix-<check> will perform the check and automatically attempt to
      fix sny style error (printing a warning if unsuccessful)

    --ignore-<check> will perform the check but ignore any errors
      found (other than printing a message for each)

    --skip-<check> will skip performing the check entirely

    If none of these options are given, all checks will be performed
    and the user will be prompted on how to handle each error.

    --fix-all, --ignore-all, and --skip-all are equivalent to specifying
    --fix-<check>, --ignore-<check>, or --skip-<check> for all checks,
    respectively.  However, option settings for specific checks take
    precedence.  Thus --skip-all --fix-white can be used to skip every
    check other than whitespace errors, which will be checked and
    automatically fixed.

    The -v/--verbose flag will display the offending line(s) as well
    as their location.
    """

    ui = MercurialUI(hgui, verbose=hgui.verbose)

    # instantiate varifier objects
    verifiers = [v(ui, opts, base=repo.root) for v in all_verifiers]

    for fname, mod_regions in _modified_regions(repo, pats, **opts):
        for verifier in verifiers:
            if verifier.apply(joinpath(repo.root, fname), mod_regions):
                return True

    return False

def check_hook(hooktype):
    if hooktype not in ('pretxncommit', 'pre-qrefresh'):
        raise AttributeError, \
              "This hook is not meant for %s" % hooktype

# This function provides a hook that is called before transaction
# commit and on qrefresh
def check_style(ui, repo, hooktype, **kwargs):
    check_hook(hooktype)
    args = {}

    try:
        return do_check_style(ui, repo, **args)
    except Exception, e:
        import traceback
        traceback.print_exc()
        return True

try:
    from mercurial.i18n import _
except ImportError:
    def _(arg):
        return arg

_common_region_options = [
    ('a', 'all', False,
     _("include clean files and unmodified parts of modified files")),
    ('', 'no-ignore', False, _("ignore the style ignore list")),
    ]


fix_opts = [('f', 'fix-all', False, _("fix all style errors"))] + \
           [('', 'fix-' + v.opt_name, False,
             _('fix errors in ' + v.test_name)) for v in all_verifiers]
ignore_opts = [('', 'ignore-all', False, _("ignore all style errors"))] + \
              [('', 'ignore-' + v.opt_name, False,
                _('ignore errors in ' + v.test_name)) for v in all_verifiers]
skip_opts = [('', 'skip-all', False, _("skip all style error checks"))] + \
            [('', 'skip-' + v.opt_name, False,
              _('skip checking for ' + v.test_name)) for v in all_verifiers]
all_opts = fix_opts + ignore_opts + skip_opts


cmdtable = {
    '^m5style' : (
        do_check_style, all_opts + _common_region_options + commands.walkopts,
        _('hg m5style [-a] [FILE]...')),
}

if __name__ == '__main__':
    print >> sys.stderr, "This file cannot be used from the command line. Use"
    print >> sys.stderr, "style.py instead."
    sys.exit(1)
