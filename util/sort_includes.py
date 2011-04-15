#!/usr/bin/env python

import os
import re
import sys

from file_types import *

cpp_c_headers = {
    'assert.h' : 'cassert',
    'ctype.h'  : 'cctype',
    'errno.h'  : 'cerrno',
    'float.h'  : 'cfloat',
    'limits.h' : 'climits',
    'locale.h' : 'clocale',
    'math.h'   : 'cmath',
    'setjmp.h' : 'csetjmp',
    'signal.h' : 'csignal',
    'stdarg.h' : 'cstdarg',
    'stddef.h' : 'cstddef',
    'stdio.h'  : 'cstdio',
    'stdlib.h' : 'cstdlib',
    'string.h' : 'cstring',
    'time.h'   : 'ctime',
    'wchar.h'  : 'cwchar',
    'wctype.h' : 'cwctype',
}

include_re = re.compile(r'([#%])(include|import).*[<"](.*)[">]')
def include_key(line):
    '''Mark directories with a leading space so directories
    are sorted before files'''

    match = include_re.match(line)
    assert match, line
    keyword = match.group(2)
    include = match.group(3)

    # Everything but the file part needs to have a space prepended
    parts = include.split('/')
    if len(parts) == 2 and parts[0] == 'dnet':
        # Don't sort the dnet includes with respect to each other, but
        # make them sorted with respect to non dnet includes.  Python
        # guarantees that sorting is stable, so just clear the
        # basename part of the filename.
        parts[1] = ' '
    parts[0:-1] = [ ' ' + s for s in parts[0:-1] ]
    key = '/'.join(parts)

    return key

class SortIncludes(object):
    # different types of includes for different sorting of headers
    # <Python.h>         - Python header needs to be first if it exists
    # <*.h>              - system headers (directories before files)
    # <*>                - STL headers
    # <*.(hh|hxx|hpp|H)> - C++ Headers (directories before files)
    # "*"                - M5 headers (directories before files)
    includes_re = (
        ('python', '<>', r'^(#include)[ \t]+<(Python.*\.h)>(.*)'),
        ('c', '<>', r'^(#include)[ \t]<(.+\.h)>(.*)'),
        ('stl', '<>', r'^(#include)[ \t]+<([0-9A-z_]+)>(.*)'),
        ('cc', '<>', r'^(#include)[ \t]+<([0-9A-z_]+\.(hh|hxx|hpp|H))>(.*)'),
        ('m5cc', '""', r'^(#include)[ \t]"(.+\.h{1,2})"(.*)'),
        ('swig0', '<>', r'^(%import)[ \t]<(.+)>(.*)'),
        ('swig1', '<>', r'^(%include)[ \t]<(.+)>(.*)'),
        ('swig2', '""', r'^(%import)[ \t]"(.+)"(.*)'),
        ('swig3', '""', r'^(%include)[ \t]"(.+)"(.*)'),
        )

    # compile the regexes
    includes_re = tuple((a, b, re.compile(c)) for a,b,c in includes_re)

    def __init__(self):
        self.reset()

    def reset(self):
        # clear all stored headers
        self.includes = {}
        for include_type,_,_ in self.includes_re:
            self.includes[include_type] = []

    def dump_block(self):
        '''dump the includes'''
        first = True
        for include,_,_ in self.includes_re:
            if not self.includes[include]:
                continue

            if not first:
                # print a newline between groups of
                # include types
                yield ''
            first = False

            # print out the includes in the current group
            # and sort them according to include_key()
            prev = None
            for l in sorted(self.includes[include],
                            key=include_key):
                if l != prev:
                    yield l
                prev = l

    def __call__(self, lines, filename, language):
        leading_blank = False
        blanks = 0
        block = False

        for line in lines:
            if not line:
                blanks += 1
                if not block:
                    # if we're not in an include block, spit out the
                    # newline otherwise, skip it since we're going to
                    # control newlines withinin include block
                    yield ''
                continue

            # Try to match each of the include types
            for include_type,(ldelim,rdelim),include_re in self.includes_re:
                match = include_re.match(line)
                if not match:
                    continue

                # if we've got a match, clean up the #include line,
                # fix up stl headers and store it in the proper category
                groups = match.groups()
                keyword = groups[0]
                include = groups[1]
                extra = groups[-1]
                if include_type == 'c' and language == 'C++':
                    stl_inc = cpp_c_headers.get(include, None)
                    if stl_inc:
                        include = stl_inc
                        include_type = 'stl'

                line = keyword + ' ' + ldelim + include + rdelim + extra

                self.includes[include_type].append(line)

                # We've entered a block, don't keep track of blank
                # lines while in a block
                block = True
                blanks = 0
                break
            else:
                # this line did not match a #include
                assert not include_re.match(line)

                # if we're not in a block and we didn't match an include
                # to enter a block, just emit the line and continue
                if not block:
                    yield line
                    continue

                # We've exited an include block.
                for block_line in self.dump_block():
                    yield block_line

                # if there are any newlines after the include block,
                # emit a single newline (removing extras)
                if blanks and block:
                    yield ''

                blanks = 0
                block = False
                self.reset()

                # emit the line that ended the block
                yield line

        if block:
            # We've exited an include block.
            for block_line in self.dump_block():
                yield block_line



# default language types to try to apply our sorting rules to
default_languages = frozenset(('C', 'C++', 'isa', 'python', 'scons', 'swig'))

def options():
    import optparse
    options = optparse.OptionParser()
    add_option = options.add_option
    add_option('-d', '--dir_ignore', metavar="DIR[,DIR]", type='string',
               default=','.join(default_dir_ignore),
               help="ignore directories")
    add_option('-f', '--file_ignore', metavar="FILE[,FILE]", type='string',
               default=','.join(default_file_ignore),
               help="ignore files")
    add_option('-l', '--languages', metavar="LANG[,LANG]", type='string',
               default=','.join(default_languages),
               help="languages")
    add_option('-n', '--dry-run', action='store_true',
               help="don't overwrite files")

    return options

def parse_args(parser):
    opts,args = parser.parse_args()

    opts.dir_ignore = frozenset(opts.dir_ignore.split(','))
    opts.file_ignore = frozenset(opts.file_ignore.split(','))
    opts.languages = frozenset(opts.languages.split(','))

    return opts,args

if __name__ == '__main__':
    parser = options()
    opts, args = parse_args(parser)

    for base in args:
        for filename,language in find_files(base, languages=opts.languages,
                file_ignore=opts.file_ignore, dir_ignore=opts.dir_ignore):
            if opts.dry_run:
                print "%s: %s" % (filename, language)
            else:
                update_file(filename, filename, language, SortIncludes())
