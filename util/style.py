#! /usr/bin/env python
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
#
# Authors: Nathan Binkert
#          Steve Reinhardt

import heapq
import os
import re
import sys

from os.path import dirname, join as joinpath
from itertools import count
from mercurial import bdiff, mdiff, commands

current_dir = dirname(__file__)
sys.path.insert(0, current_dir)
sys.path.insert(1, joinpath(dirname(current_dir), 'src', 'python'))

from m5.util import neg_inf, pos_inf, Region, Regions
import sort_includes
from file_types import lang_type

all_regions = Regions(Region(neg_inf, pos_inf))

tabsize = 8
lead = re.compile(r'^([ \t]+)')
trail = re.compile(r'([ \t]+)$')
any_control = re.compile(r'\b(if|while|for)([ \t]*)\(')

format_types = set(('C', 'C++'))


def re_ignore(expr):
    """Helper function to create regular expression ignore file
    matcher functions"""

    rex = re.compile(expr)
    def match_re(fname):
        return rex.match(fname)
    return match_re

# This list contains a list of functions that are called to determine
# if a file should be excluded from the style matching rules or
# not. The functions are called with the file name relative to the
# repository root (without a leading slash) as their argument. A file
# is excluded if any function in the list returns true.
style_ignores = [
    # Ignore external projects as they are unlikely to follow the gem5
    # coding convention.
    re_ignore("^ext/"),
]

def check_ignores(fname):
    """Check if a file name matches any of the ignore rules"""

    for rule in style_ignores:
        if rule(fname):
            return True

    return False


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

class UserInterface(object):
    def __init__(self, verbose=False):
        self.verbose = verbose

    def prompt(self, prompt, results, default):
        while True:
            result = self.do_prompt(prompt, results, default)
            if result in results:
                return result

class MercurialUI(UserInterface):
    def __init__(self, ui, *args, **kwargs):
        super(MercurialUI, self).__init__(*args, **kwargs)
        self.ui = ui

    def do_prompt(self, prompt, results, default):
        return self.ui.prompt(prompt, default=default)

    def write(self, string):
        self.ui.write(string)

class StdioUI(UserInterface):
    def do_prompt(self, prompt, results, default):
        return raw_input(prompt) or default

    def write(self, string):
        sys.stdout.write(string)


class Verifier(object):
    """Base class for style verifier objects

    Subclasses must define these class attributes:
      languages = set of strings identifying applicable languages
      test_name = long descriptive name of test, will be used in
                  messages such as "error in <foo>" or "invalid <foo>"
      opt_name = short name used to generate command-line options to
                 control the test (--fix-<foo>, --ignore-<foo>, etc.)
    """

    def __init__(self, ui, repo, opts):
        self.ui = ui
        self.repo = repo
        # opt_name must be defined as a class attribute of derived classes.
        # Check test-specific opts first as these have precedence.
        self.opt_fix = opts.get('fix_' + self.opt_name, False)
        self.opt_ignore = opts.get('ignore_' + self.opt_name, False)
        self.opt_skip = opts.get('skip_' + self.opt_name, False)
        # If no test-specific opts were set, then set based on "-all" opts.
        if not (self.opt_fix or self.opt_ignore or self.opt_skip):
            self.opt_fix = opts.get('fix_all', False)
            self.opt_ignore = opts.get('ignore_all', False)
            self.opt_skip = opts.get('skip_all', False)

    def __getattr__(self, attr):
        if attr in ('prompt', 'write'):
            return getattr(self.ui, attr)

        if attr == 'wctx':
            try:
                wctx = repo.workingctx()
            except:
                from mercurial import context
                wctx = context.workingctx(repo)
            self.wctx = wctx
            return wctx

        raise AttributeError

    def open(self, filename, mode):
        filename = self.repo.wjoin(filename)

        try:
            f = file(filename, mode)
        except OSError, msg:
            print 'could not open file %s: %s' % (filename, msg)
            return None

        return f

    def skip(self, filename):
        filename = self.repo.wjoin(filename)

        # We never want to handle symlinks, so always skip them: If the location
        # pointed to is a directory, skip it. If the location is a file inside
        # the gem5 directory, it will be checked as a file, so symlink can be
        # skipped. If the location is a file outside gem5, we don't want to
        # check it anyway.
        if os.path.islink(filename):
            return True
        return lang_type(filename) not in self.languages

    def check(self, filename, regions=all_regions):
        """Check specified regions of file 'filename'.

        Line-by-line checks can simply provide a check_line() method
        that returns True if the line is OK and False if it has an
        error.  Verifiers that need a multi-line view (like
        SortedIncludes) must override this entire function.

        Returns a count of errors (0 if none), though actual non-zero
        count value is not currently used anywhere.
        """

        f = self.open(filename, 'r')

        errors = 0
        for num,line in enumerate(f):
            if num not in regions:
                continue
            line = line.rstrip('\n')
            if not self.check_line(line):
                self.write("invalid %s in %s:%d\n" % \
                           (self.test_name, filename, num + 1))
                if self.ui.verbose:
                    self.write(">>%s<<\n" % line[:-1])
                errors += 1
        return errors

    def fix(self, filename, regions=all_regions):
        """Fix specified regions of file 'filename'.

        Line-by-line fixes can simply provide a fix_line() method that
        returns the fixed line. Verifiers that need a multi-line view
        (like SortedIncludes) must override this entire function.
        """

        f = self.open(filename, 'r+')

        lines = list(f)

        f.seek(0)
        f.truncate()

        for i,line in enumerate(lines):
            if i in regions:
                line = self.fix_line(line)

            f.write(line)
        f.close()


    def apply(self, filename, regions=all_regions):
        """Possibly apply to specified regions of file 'filename'.

        Verifier is skipped if --skip-<test> option was provided or if
        file is not of an applicable type.  Otherwise file is checked
        and error messages printed.  Errors are fixed or ignored if
        the corresponding --fix-<test> or --ignore-<test> options were
        provided.  If neither, the user is prompted for an action.

        Returns True to abort, False otherwise.
        """
        if not (self.opt_skip or self.skip(filename)):
            errors = self.check(filename, regions)
            if errors and not self.opt_ignore:
                if self.opt_fix:
                    self.fix(filename, regions)
                else:
                    result = self.ui.prompt("(a)bort, (i)gnore, or (f)ix?",
                                            'aif', 'a')
                    if result == 'f':
                        self.fix(filename, regions)
                    elif result == 'a':
                        return True # abort

        return False


class Whitespace(Verifier):
    """Check whitespace.

    Specifically:
    - No tabs used for indent
    - No trailing whitespace
    """

    languages = set(('C', 'C++', 'swig', 'python', 'asm', 'isa', 'scons'))
    test_name = 'whitespace'
    opt_name = 'white'

    def check_line(self, line):
        match = lead.search(line)
        if match and match.group(1).find('\t') != -1:
            return False

        match = trail.search(line)
        if match:
            return False

        return True

    def fix_line(self, line):
        if lead.search(line):
            newline = ''
            for i,c in enumerate(line):
                if c == ' ':
                    newline += ' '
                elif c == '\t':
                    newline += ' ' * (tabsize - len(newline) % tabsize)
                else:
                    newline += line[i:]
                    break

            line = newline

        return line.rstrip() + '\n'


class ControlSpace(Verifier):
    """Check for exactly one space after if/while/for"""

    languages = set(('C', 'C++'))
    test_name = 'spacing after if/while/for'
    opt_name = 'control'

    def check_line(self, line):
        match = any_control.search(line)
        return not (match and match.group(2) != " ")

    def fix_line(self, line):
        new_line = any_control.sub(r'\1 (', line)
        return new_line


class SortedIncludes(Verifier):
    """Check for proper sorting of include statements"""

    languages = sort_includes.default_languages
    test_name = 'include file order'
    opt_name = 'include'

    def __init__(self, *args, **kwargs):
        super(SortedIncludes, self).__init__(*args, **kwargs)
        self.sort_includes = sort_includes.SortIncludes()

    def check(self, filename, regions=all_regions):
        f = self.open(filename, 'r')

        lines = [ l.rstrip('\n') for l in f.xreadlines() ]
        old = ''.join(line + '\n' for line in lines)
        f.close()

        if len(lines) == 0:
            return 0

        language = lang_type(filename, lines[0])
        sort_lines = list(self.sort_includes(lines, filename, language))
        new = ''.join(line + '\n' for line in sort_lines)

        mod = modified_regions(old, new)
        modified = mod & regions

        if modified:
            self.write("invalid sorting of includes in %s\n" % (filename))
            if self.ui.verbose:
                for start, end in modified.regions:
                    self.write("bad region [%d, %d)\n" % (start, end))
            return 1

        return 0

    def fix(self, filename, regions=all_regions):
        f = self.open(filename, 'r+')

        old = f.readlines()
        lines = [ l.rstrip('\n') for l in old ]
        language = lang_type(filename, lines[0])
        sort_lines = list(self.sort_includes(lines, filename, language))
        new = ''.join(line + '\n' for line in sort_lines)

        f.seek(0)
        f.truncate()

        for i,line in enumerate(sort_lines):
            f.write(line)
            f.write('\n')
        f.close()


def linelen(line):
    tabs = line.count('\t')
    if not tabs:
        return len(line)

    count = 0
    for c in line:
        if c == '\t':
            count += tabsize - count % tabsize
        else:
            count += 1

    return count

class LineLength(Verifier):
    languages = set(('C', 'C++', 'swig', 'python', 'asm', 'isa', 'scons'))
    test_name = 'line length'
    opt_name = 'length'

    def check_line(self, line):
        return linelen(line) <= 78

    def fix(self, filename, regions=all_regions):
        self.write("Warning: cannot automatically fix overly long lines.\n")


class BoolCompare(Verifier):
    languages = set(('C', 'C++', 'python'))
    test_name = 'boolean comparison'
    opt_name = 'boolcomp'

    regex = re.compile(r'\s*==\s*([Tt]rue|[Ff]alse)\b')

    def check_line(self, line):
        return self.regex.search(line) == None

    def fix_line(self, line):
        match = self.regex.search(line)
        if match:
            if match.group(1) in ('true', 'True'):
                line = self.regex.sub('', line)
            else:
                self.write("Warning: cannot automatically fix "
                           "comparisons with false/False.\n")
        return line


# list of all verifier classes
all_verifiers = [
    Whitespace,
    ControlSpace,
    LineLength,
    BoolCompare,
    SortedIncludes
]

class ValidationStats(object):
    def __init__(self):
        self.toolong = 0
        self.toolong80 = 0
        self.leadtabs = 0
        self.trailwhite = 0
        self.badcontrol = 0
        self.cret = 0

    def dump(self):
        print '''\
%d violations of lines over 79 chars. %d of which are 80 chars exactly.
%d cases of whitespace at the end of a line.
%d cases of tabs to indent.
%d bad parens after if/while/for.
%d carriage returns found.
''' % (self.toolong, self.toolong80, self.trailwhite, self.leadtabs,
       self.badcontrol, self.cret)

    def __nonzero__(self):
        return self.toolong or self.toolong80 or self.leadtabs or \
               self.trailwhite or self.badcontrol or self.cret

def validate(filename, stats, verbose, exit_code):
    lang = lang_type(filename)
    if lang not in format_types:
        return

    def msg(lineno, line, message):
        print '%s:%d>' % (filename, lineno + 1), message
        if verbose > 2:
            print line

    def bad():
        if exit_code is not None:
            sys.exit(exit_code)

    try:
        f = file(filename, 'r')
    except OSError:
        if verbose > 0:
            print 'could not open file %s' % filename
        bad()
        return

    for i,line in enumerate(f):
        line = line.rstrip('\n')

        # no carriage returns
        if line.find('\r') != -1:
            self.cret += 1
            if verbose > 1:
                msg(i, line, 'carriage return found')
            bad()

        # lines max out at 79 chars
        llen = linelen(line)
        if llen > 79:
            stats.toolong += 1
            if llen == 80:
                stats.toolong80 += 1
            if verbose > 1:
                msg(i, line, 'line too long (%d chars)' % llen)
            bad()

        # no tabs used to indent
        match = lead.search(line)
        if match and match.group(1).find('\t') != -1:
            stats.leadtabs += 1
            if verbose > 1:
                msg(i, line, 'using tabs to indent')
            bad()

        # no trailing whitespace
        if trail.search(line):
            stats.trailwhite +=1
            if verbose > 1:
                msg(i, line, 'trailing whitespace')
            bad()

        # for c++, exactly one space betwen if/while/for and (
        if lang == 'C++':
            match = any_control.search(line)
            if match and match.group(2) != " ":
                stats.badcontrol += 1
                if verbose > 1:
                    msg(i, line, 'improper spacing after %s' % match.group(1))
                bad()


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
    verifiers = [v(ui, repo, opts) for v in all_verifiers]

    for fname, mod_regions in _modified_regions(repo, pats, **opts):
        for verifier in verifiers:
            if verifier.apply(fname, mod_regions):
                return True

    return False

def do_check_format(hgui, repo, *pats, **opts):
    """check files for gem5 code formatting violations

    Without an argument, checks all modified and added files for gem5
    code formatting violations. A list of files can be specified to
    limit the checker to a subset of the repository. The style rules
    are normally applied on a diff of the repository state (i.e.,
    added files are checked in their entirety while only modifications
    of modified files are checked).

    The --all option can be specified to include clean files and check
    modified files in their entirety.
    """
    ui = MercurialUI(hgui, hgui.verbose)

    verbose = 0
    for fname, mod_regions in _modified_regions(repo, pats, **opts):
        stats = ValidationStats()
        validate(joinpath(repo.root, fname), stats, verbose, None)
        if stats:
            print "%s:" % fname
            stats.dump()
            result = ui.prompt("invalid formatting\n(i)gnore or (a)bort?",
                               'ai', 'a')
            if result == 'a':
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

def check_format(ui, repo, hooktype, **kwargs):
    check_hook(hooktype)
    args = {}

    try:
        return do_check_format(ui, repo, **args)
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
    '^m5format' :
    ( do_check_format, [
            ] + _common_region_options + commands.walkopts,
      _('hg m5format [FILE]...')),
}

if __name__ == '__main__':
    import getopt

    progname = sys.argv[0]
    if len(sys.argv) < 2:
        sys.exit('usage: %s <command> [<command args>]' % progname)

    fixwhite_usage = '%s fixwhite [-t <tabsize> ] <path> [...] \n' % progname
    chkformat_usage = '%s chkformat <path> [...] \n' % progname
    chkwhite_usage = '%s chkwhite <path> [...] \n' % progname

    command = sys.argv[1]
    if command == 'fixwhite':
        flags = 't:'
        usage = fixwhite_usage
    elif command == 'chkwhite':
        flags = 'nv'
        usage = chkwhite_usage
    elif command == 'chkformat':
        flags = 'nv'
        usage = chkformat_usage
    else:
        sys.exit(fixwhite_usage + chkwhite_usage + chkformat_usage)

    opts, args = getopt.getopt(sys.argv[2:], flags)

    code = 1
    verbose = 1
    for opt,arg in opts:
        if opt == '-n':
            code = None
        if opt == '-t':
            tabsize = int(arg)
        if opt == '-v':
            verbose += 1

    if command == 'fixwhite':
        for filename in args:
            fixwhite(filename, tabsize)
    elif command == 'chkwhite':
        for filename in args:
            for line,num in checkwhite(filename):
                print 'invalid whitespace: %s:%d' % (filename, num)
                if verbose:
                    print '>>%s<<' % line[:-1]
    elif command == 'chkformat':
        stats = ValidationStats()
        for filename in args:
            validate(filename, stats=stats, verbose=verbose, exit_code=code)

        if verbose > 0:
            stats.dump()
    else:
        sys.exit("command '%s' not found" % command)
