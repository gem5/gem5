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


def _include_matcher(keyword="#include", delim="<>"):
    """Match an include statement and return a (keyword, file, extra)
    duple, or a touple of None values if there isn't a match."""

    rex = re.compile(r'^(%s)\s*%s(.*)%s(.*)$' % (keyword, delim[0], delim[1]))

    def matcher(context, line):
        m = rex.match(line)
        return m.groups() if m else (None, ) * 3

    return matcher

def _include_matcher_fname(fname, **kwargs):
    """Match an include of a specific file name. Any keyword arguments
    are forwarded to _include_matcher, which is used to match the
    actual include line."""

    rex = re.compile(fname)
    base_matcher = _include_matcher(**kwargs)

    def matcher(context, line):
        (keyword, fname, extra) = base_matcher(context, line)
        if fname and rex.match(fname):
            return (keyword, fname, extra)
        else:
            return (None, ) * 3

    return matcher


def _include_matcher_main():
    """Match a C/C++ source file's primary header (i.e., a file with
    the same base name, but a header extension)."""

    base_matcher = _include_matcher(delim='""')
    rex = re.compile(r"^src/(.*)\.([^.]+)$")
    header_map = {
        "c" : "h",
        "cc" : "hh",
        "cpp" : "hh",
        }
    def matcher(context, line):
        m = rex.match(context["filename"])
        if not m:
            return (None, ) * 3
        base, ext = m.groups()
        (keyword, fname, extra) = base_matcher(context, line)
        try:
            if fname == "%s.%s" % (base, header_map[ext]):
                return (keyword, fname, extra)
        except KeyError:
            pass

        return (None, ) * 3

    return matcher

class SortIncludes(object):
    # different types of includes for different sorting of headers
    # <Python.h>         - Python header needs to be first if it exists
    # <*.h>              - system headers (directories before files)
    # <*>                - STL headers
    # <*.(hh|hxx|hpp|H)> - C++ Headers (directories before files)
    # "*"                - M5 headers (directories before files)
    includes_re = (
        ('main', '""', _include_matcher_main()),
        ('python', '<>', _include_matcher_fname("^Python\.h$")),
        ('c', '<>', _include_matcher_fname("^.*\.h$")),
        ('stl', '<>', _include_matcher_fname("^\w+$")),
        ('cc', '<>', _include_matcher_fname("^.*\.(hh|hxx|hpp|H)$")),
        ('m5header', '""', _include_matcher_fname("^.*\.h{1,2}$", delim='""')),
        ('swig0', '<>', _include_matcher(keyword="%import")),
        ('swig1', '<>', _include_matcher(keyword="%include")),
        ('swig2', '""', _include_matcher(keyword="%import", delim='""')),
        ('swig3', '""', _include_matcher(keyword="%include", delim='""')),
        )

    block_order = (
        ('main', ),
        ('python', ),
        ('c', ),
        ('stl', ),
        ('cc', ),
        ('m5header', ),
        ('swig0', 'swig1', 'swig2', 'swig3', ),
        )

    def __init__(self):
        self.block_priority = {}
        for prio, keys in enumerate(self.block_order):
            for key in keys:
                self.block_priority[key] = prio

    def reset(self):
        # clear all stored headers
        self.includes = {}

    def dump_blocks(self, block_types):
        """Merge includes of from several block types into one large
        block of sorted includes. This is useful when we have multiple
        include block types (e.g., swig includes) with the same
        priority."""

        includes = []
        for block_type in block_types:
            try:
                includes += self.includes[block_type]
            except KeyError:
                pass

        return sorted(set(includes))

    def dump_includes(self):
        blocks = []
        # Create a list of blocks in the prescribed include
        # order. Each entry in the list is a multi-line string with
        # multiple includes.
        for types in self.block_order:
            block = "\n".join(self.dump_blocks(types))
            if block:
                blocks.append(block)

        self.reset()
        return "\n\n".join(blocks)

    def __call__(self, lines, filename, language):
        self.reset()

        context = {
            "filename" : filename,
            "language" : language,
            }

        def match_line(line):
            if not line:
                return (None, line)

            for include_type, (ldelim, rdelim), matcher in self.includes_re:
                keyword, include, extra = matcher(context, line)
                if keyword:
                    # if we've got a match, clean up the #include line,
                    # fix up stl headers and store it in the proper category
                    if include_type == 'c' and language == 'C++':
                        stl_inc = cpp_c_headers.get(include, None)
                        if stl_inc:
                            include = stl_inc
                            include_type = 'stl'

                    return (include_type,
                            keyword + ' ' + ldelim + include + rdelim + extra)

            return (None, line)

        processing_includes = False
        for line in lines:
            include_type, line = match_line(line)
            if include_type:
                try:
                    self.includes[include_type].append(line)
                except KeyError:
                    self.includes[include_type] = [ line ]

                processing_includes = True
            elif processing_includes and not line.strip():
                # Skip empty lines while processing includes
                pass
            elif processing_includes:
                # We are now exiting an include block
                processing_includes = False

                # Output pending includes, a new line between, and the
                # current l.
                yield self.dump_includes()
                yield ''
                yield line
            else:
                # We are not in an include block, so just emit the line
                yield line

        # We've reached EOF, so dump any pending includes
        if processing_includes:
            yield self.dump_includes()



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
