#!/usr/bin/env python

import os
import re
import sys

from file_types import lang_type, find_files

mode_line = re.compile('(-\*- *mode:.* *-\*-)')
shell_comment = re.compile(r'^\s*#')
lisp_comment = re.compile(r';')
cpp_comment = re.compile(r'//')
c_comment_start = re.compile(r'/\*')
c_comment_end   = re.compile(r'\*/')
def find_copyright_block(lines, lang_type):
    start = None
    if lang_type in ('python', 'make', 'shell', 'perl', 'scons'):
        for i,line in enumerate(lines):
            if i == 0 and (line.startswith('#!') or mode_line.search(line)):
                continue

            if shell_comment.search(line):
                if start is None:
                    start = i
            elif start is None:
                if line.strip():
                    return
            else:
                yield start, i-1
                start = None

    elif lang_type in ('lisp', ):
        for i,line in enumerate(lines):
            if i == 0 and mode_line.search(line):
                continue

            if lisp_comment.search(line):
                if start is None:
                    start = i
            elif start is None:
                if line.strip():
                    return
            else:
                yield start, i-1
                start = None

    elif lang_type in ('C', 'C++', 'swig', 'isa', 'asm', 'slicc',
                       'lex', 'yacc'):
        mode = None
        for i,line in enumerate(lines):
            if i == 0 and mode_line.search(line):
                continue

            if mode == 'C':
                assert start is not None, 'on line %d' % (i + 1)
                match = c_comment_end.search(line)
                if match:
                    yield start, i
                    mode = None
                continue

            cpp_match = cpp_comment.search(line)
            c_match = c_comment_start.search(line)

            if cpp_match:
                assert not c_match, 'on line %d' % (i + 1)
                if line[:cpp_match.start()].strip():
                    return
                if mode is None:
                    mode = 'CPP'
                    start = i
                else:
                    text = line[cpp_match.end():].lstrip()
                    if text.startswith("Copyright") > 0:
                        yield start, i-1
                        start = i
                continue
            elif mode == 'CPP':
                assert start is not None, 'on line %d' % (i + 1)
                if not line.strip():
                    continue
                yield start, i-1
                mode = None
                if not c_match:
                    return

            if c_match:
                assert mode is None, 'on line %d' % (i + 1)
                mode = 'C'
                start = i

            if mode is None and line.strip():
                return

    else:
        raise AttributeError, "Could not handle language %s" % lang_type

date_range_re = re.compile(r'([0-9]{4})\s*-\s*([0-9]{4})')
def process_dates(dates):
    dates = [ d.strip() for d in dates.split(',') ]

    output = set()
    for date in dates:
        match = date_range_re.match(date)
        if match:
            f,l = [ int(d) for d in match.groups() ]
            for i in xrange(f, l+1):
                output.add(i)
        else:
            try:
                date = int(date)
                output.add(date)
            except ValueError:
                pass

    return output

copyright_re = \
    re.compile(r'Copyright (\([cC]\)) ([-, 0-9]+)[\s*#/]*([A-z-,. ]+)',
               re.DOTALL)

authors_re = re.compile(r'^[\s*#/]*Authors:\s*([A-z .]+)\s*$')
more_authors_re = re.compile(r'^[\s*#/]*([A-z .]+)\s*$')

all_owners = set()
def get_data(lang_type, lines):
    data = []
    last = None
    for start,end in find_copyright_block(lines, lang_type):
        joined = ''.join(lines[start:end+1])
        match = copyright_re.search(joined)
        if not match:
            continue

        c,dates,owner = match.groups()
        dates = dates.strip()
        owner = owner.strip()

        all_owners.add(owner)
        try:
            dates = process_dates(dates)
        except Exception:
            print dates
            print owner
            raise

        authors = []
        for i in xrange(start,end+1):
            line = lines[i]
            if not authors:
                match = authors_re.search(line)
                if match:
                    authors.append(match.group(1).strip())
            else:
                match = more_authors_re.search(line)
                if not match:
                    for j in xrange(i, end+1):
                        line = lines[j].strip()
                        if not line:
                            end = j
                            break
                        if line.startswith('//'):
                            line = line[2:].lstrip()
                            if line:
                                end = j - 1
                                break
                    break
                authors.append(match.group(1).strip())

        info = (owner, dates, authors, start, end)
        data.append(info)

    return data

def datestr(dates):
    dates = list(dates)
    dates.sort()

    output = []
    def add_output(first, second):
        if first == second:
            output.append('%d' % (first))
        else:
            output.append('%d-%d' % (first, second))

    first = dates.pop(0)
    second = first
    while dates:
        next = dates.pop(0)
        if next == second + 1:
            second = next
        else:
            add_output(first, second)
            first = next
            second = next

    add_output(first, second)

    return ','.join(output)

usage_str = """usage:
%s [-v] <directory>"""

def usage(exitcode):
    print usage_str % sys.argv[0]
    if exitcode is not None:
        sys.exit(exitcode)

if __name__ == '__main__':
    import getopt

    show_counts = False
    ignore = set()
    verbose = False
    try:
        opts, args = getopt.getopt(sys.argv[1:], "ci:v")
    except getopt.GetoptError:
        usage(1)

    for o,a in opts:
        if o == '-c':
            show_counts = True
        if o == '-i':
            ignore.add(a)
        if o == '-v':
            verbose = True

    files = []

    for base in args:
        if os.path.isfile(base):
            files += [ (base, lang_type(base)) ]
        elif os.path.isdir(base):
            files += find_files(base)
        else:
            raise AttributeError, "can't access '%s'" %  base

    copyrights = {}
    counts = {}

    for filename, lang in files:
        f = file(filename, 'r')
        lines = f.readlines()
        if not lines:
            continue

        lines = [ line.rstrip('\r\n') for line in lines ]

        lt = lang_type(filename, lines[0])
        try:
            data = get_data(lt, lines)
        except Exception, e:
            if verbose:
                if len(e.args) == 1:
                    e.args = ('%s (%s))' % (e, filename), )
                print "could not parse %s: %s" % (filename, e)
            continue

        for owner, dates, authors, start, end in data:
            if owner not in copyrights:
                copyrights[owner] = set()
            if owner not in counts:
                counts[owner] = 0

            copyrights[owner] |= dates
            counts[owner] += 1

    info = [ (counts[o], d, o) for o,d in copyrights.items() ]

    for count,dates,owner in sorted(info, reverse=True):
        if show_counts:
            owner = '%s (%s files)' % (owner, count)
        print 'Copyright (c) %s %s' % (datestr(dates), owner)
