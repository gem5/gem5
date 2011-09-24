#! /usr/bin/env python
# Copyright (c) 2006 The Regents of The University of Michigan
# Copyright (c) 2007,2011 The Hewlett-Packard Development Company
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

import heapq
import os
import re
import sys

from os.path import dirname, join as joinpath
from itertools import count
from mercurial import bdiff, mdiff

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
any_control = re.compile(r'\b(if|while|for)[ \t]*[(]')
good_control = re.compile(r'\b(if|while|for) [(]')

format_types = set(('C', 'C++'))

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
        mod_regions.add(0, len(lines))

    return mod_regions

class UserInterface(object):
    def __init__(self, verbose=False, auto=False):
        self.auto = auto
        self.verbose = verbose

    def prompt(self, prompt, results, default):
        if self.auto:
            return self.auto

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
    def __init__(self, ui, repo=None):
        self.ui = ui
        self.repo = repo
        if repo is None:
            self.wctx = None

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
        if self.repo:
            filename = self.repo.wjoin(filename)

        try:
            f = file(filename, mode)
        except OSError, msg:
            print 'could not open file %s: %s' % (filename, msg)
            return None

        return f

    def skip(self, filename):
        return lang_type(filename) not in self.languages

    def check(self, filename, regions=all_regions):
        f = self.open(filename, 'r')

        errors = 0
        for num,line in enumerate(f):
            if num not in regions:
                continue
            if not self.check_line(line):
                self.write("invalid %s in %s:%d\n" % \
                               (self.test_name, filename, num + 1))
                if self.ui.verbose:
                    self.write(">>%s<<\n" % line[-1])
                errors += 1
        return errors

    def fix(self, filename, regions=all_regions):
        f = self.open(filename, 'r+')

        lines = list(f)

        f.seek(0)
        f.truncate()

        for i,line in enumerate(lines):
            if i in regions:
                line = self.fix_line(line)

            f.write(line)
        f.close()

    def apply(self, filename, prompt, regions=all_regions):
        if not self.skip(filename):
            errors = self.check(filename, regions)
            if errors:
                if prompt(filename, self.fix, regions):
                    return True
        return False


class Whitespace(Verifier):
    languages = set(('C', 'C++', 'swig', 'python', 'asm', 'isa', 'scons'))
    test_name = 'whitespace'
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

class SortedIncludes(Verifier):
    languages = sort_includes.default_languages
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
            self.write("invalid sorting of includes\n")
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
    if lang_type(filename) not in format_types:
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
        if cpp:
            match = any_control.search(line)
            if match and not good_control.search(line):
                stats.badcontrol += 1
                if verbose > 1:
                    msg(i, line, 'improper spacing after %s' % match.group(1))
                bad()

def do_check_style(hgui, repo, *files, **args):
    """check files for proper m5 style guidelines"""
    from mercurial import mdiff, util

    auto = args.get('auto', False)
    if auto:
        auto = 'f'
    ui = MercurialUI(hgui, hgui.verbose, auto)

    if files:
        files = frozenset(files)

    def skip(name):
        return files and name in files

    def prompt(name, func, regions=all_regions):
        result = ui.prompt("(a)bort, (i)gnore, or (f)ix?", 'aif', 'a')
        if result == 'a':
            return True
        elif result == 'f':
            func(repo.wjoin(name), regions)

        return False

    modified, added, removed, deleted, unknown, ignore, clean = repo.status()

    whitespace = Whitespace(ui)
    sorted_includes = SortedIncludes(ui)
    for fname in added:
        if skip(fname):
            continue

        fpath = joinpath(repo.root, fname)

        if whitespace.apply(fpath, prompt):
            return True

        if sorted_includes.apply(fpath, prompt):
            return True

    try:
        wctx = repo.workingctx()
    except:
        from mercurial import context
        wctx = context.workingctx(repo)

    for fname in modified:
        if skip(fname):
            continue

        fpath = joinpath(repo.root, fname)
        regions = modregions(wctx, fname)

        if whitespace.apply(fpath, prompt, regions):
            return True

        if sorted_includes.apply(fpath, prompt, regions):
            return True

    return False

def do_check_format(hgui, repo, **args):
    ui = MercurialUI(hgui, hgui.verbose, auto)

    modified, added, removed, deleted, unknown, ignore, clean = repo.status()

    verbose = 0
    stats = ValidationStats()
    for f in modified + added:
        validate(joinpath(repo.root, f), stats, verbose, None)

    if stats:
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

cmdtable = {
    '^m5style' :
    ( do_check_style,
      [ ('a', 'auto', False, _("automatically fix whitespace")) ],
      _('hg m5style [-a] [FILE]...')),
    '^m5format' :
    ( do_check_format,
      [ ],
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
