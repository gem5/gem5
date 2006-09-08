#! /usr/bin/python

# Generate list of files to index with cscope.

# From the m5 directory, run:
#    util/cscope-find.py > cscope.files
#    cscope -b

import os

# absolute paths to skip
skipdirs = [ 'src/unittest', 'src/doxygen' ]

# suffixes of files to index
suffixes = [ '.cc', '.hh', '.c', '.h' ]

def oksuffix(f):
    for s in suffixes:
        if f.endswith(s):
            return True
    return False

for dirpath,subdirs,files in os.walk('src'):
    # filter out undesirable subdirectories
    for i,dir in enumerate(subdirs):
        if dir == 'SCCS':
            del subdirs[i]
            break

    # filter out undesriable absolute paths
    if dirpath in skipdirs:
        del subdirs[:]
        continue

    # find C/C++ sources
    okfiles = [f for f in files if oksuffix(f)]
    if okfiles:
        print '\n'.join([os.path.join(dirpath, f) for f in okfiles])
