#!/usr/bin/env python
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Ali Saidi
#          Steve Reinhardt
#          Nathan Binkert

import os
import re
import shutil
import sys

from glob import glob
from os import system
from os.path import basename, dirname, exists, isdir, isfile, join as joinpath

def mkdir(*args):
    path = joinpath(*args)
    os.mkdir(path)

def touch(*args):
    path = joinpath(*args)
    os.utime(path, None)

def rmtree(*args):
    path = joinpath(*args)
    for match in glob(path):
        if isdir(match):
            shutil.rmtree(match)
        else:
            os.unlink(match)

def remove(*args):
    path = joinpath(*args)
    for match in glob(path):
        if not isdir(match):
            os.unlink(match)

def movedir(srcdir, destdir, dir):
    src = joinpath(srcdir, dir)
    dest = joinpath(destdir, dir)

    if not isdir(src):
        raise AttributeError

    os.makedirs(dirname(dest))
    shutil.move(src, dest)

if not isdir('BitKeeper'):
    sys.exit('Not in the top level of an m5 tree!')

usage = '%s <destdir> <release name>' % sys.argv[0]

if len(sys.argv) != 3:
    sys.exit(usage)

destdir = sys.argv[1]
releasename = sys.argv[2]
release_dest = joinpath(destdir, 'release')
encumbered_dest = joinpath(destdir, 'encumbered')
release_dir = joinpath(release_dest, releasename)
encumbered_dir = joinpath(encumbered_dest, releasename)

if exists(destdir):
    if not isdir(destdir):
        raise AttributeError, '%s exists, but is not a directory' % destdir
else:
    mkdir(destdir)

if exists(release_dest):
    if not isdir(release_dest):
        raise AttributeError, \
              '%s exists, but is not a directory' % release_dest
    rmtree(release_dest)

if exists(encumbered_dest):
    if not isdir(encumbered_dest):
        raise AttributeError, \
              '%s exists, but is not a directory' % encumbered_dest
    rmtree(encumbered_dest)

mkdir(release_dest)
mkdir(encumbered_dest)
mkdir(release_dir)
mkdir(encumbered_dir)

system('bk export -tplain -w -r+ %s' % release_dir)

# make sure scons doesn't try to run flex unnecessarily
touch(release_dir, 'src/encumbered/eio/exolex.cc')

# make sure scons doesn't try to rebuild the de.msg file since it
# might fail on non linux machines
touch(release_dir, 'ext/libelf/po/de.msg')

# get rid of non-shipping code
rmtree(release_dir, 'src/encumbered/dev')
rmtree(release_dir, 'src/cpu/ozone')
rmtree(release_dir, 'src/mem/cache/tags/split*.cc')
rmtree(release_dir, 'src/mem/cache/tags/split*.hh')
rmtree(release_dir, 'src/mem/cache/prefetch/ghb_*.cc')
rmtree(release_dir, 'src/mem/cache/prefetch/ghb_*.hh')
rmtree(release_dir, 'src/mem/cache/prefetch/stride_*.cc')
rmtree(release_dir, 'src/mem/cache/prefetch/stride_*.hh')
rmtree(release_dir, 'configs/fullsys')
rmtree(release_dir, 'configs/test')
rmtree(release_dir, 'configs/splash2')
rmtree(release_dir, 'tests/long/*/ref')
rmtree(release_dir, 'tests/old')
rmtree(release_dir, 'src/dev/i8*')

# get rid of some of private scripts
remove(release_dir, 'util/chgcopyright')
remove(release_dir, 'util/make_release.py')

def remove_sources(regex, subdir):
    script = joinpath(release_dir, subdir, 'SConscript')
    if isinstance(regex, str):
        regex = re.compile(regex)
    inscript = file(script, 'r').readlines()
    outscript = file(script, 'w')
    for line in inscript:
        if regex.match(line):
            continue

        outscript.write(line)
    outscript.close()

# fix up the SConscript to deal with files we've removed
remove_sources(r'.*split.*\.cc', 'src/mem/cache/tags')
remove_sources(r'.*(ghb|stride)_prefetcher\.cc', 'src/mem/cache/prefetch')
remove_sources(r'.*i8254xGBe.*', 'src/dev')

benches = [ 'bzip2', 'eon', 'gzip', 'mcf', 'parser', 'perlbmk',
            'twolf', 'vortex' ]
for bench in benches:
    rmtree(release_dir, 'tests', 'test-progs', bench)

movedir(release_dir, encumbered_dir, 'src/encumbered')
movedir(release_dir, encumbered_dir, 'tests/test-progs/anagram')
movedir(release_dir, encumbered_dir, 'tests/quick/20.eio-short')

def taritup(directory, destdir, filename):
    basedir = dirname(directory)
    tarball = joinpath(destdir, filename)
    tardir = basename(directory)

    system('cd %s; tar cfj %s %s' % (basedir, tarball, tardir))

taritup(release_dir, destdir, '%s.tar.bz2' % releasename)
taritup(encumbered_dir, destdir, '%s-encumbered.tar.bz2' % releasename)

print "release created in %s" % destdir
print "don't forget to tag the repository! The following command will do it:"
print "bk tag %s" % releasename
