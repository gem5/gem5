#!/usr/bin/env python
# Copyright (c) 2005 The Regents of The University of Michigan
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
#          Nathan Binkert

import os, os.path, re, sys
from os import environ as env, listdir
from os.path import basename, isdir, isfile, islink, join as joinpath
from filecmp import cmp as filecmp
from shutil import copyfile

progname = basename(sys.argv[0])
usage = """\
Usage:
    %(progname)s [-c] [-e] [-f] [-q queue] [-v] <regexp>
    -c           clean directory if job can be run
    -e           only echo pbs command info, don't actually send the job
    -f           force the job to run regardless of state
    -q <queue>   submit job to the named queue
    -v           be verbose

    %(progname)s -l [-v] <regexp>
    -l           list job names, don't submit
    -v           be verbose (list job parameters)

    %(progname)s -h
    -h           display this help
""" % locals()

try:
    import getopt
    opts, args = getopt.getopt(sys.argv[1:], '-cefhlq:v')
except getopt.GetoptError:
    sys.exit(usage)

clean = False
onlyecho = False
exprs = []
force = False
listonly = False
queue = ''
verbose = False
for o,a in opts:
    if o == '-c':
        clean = True
    if o == '-e':
        onlyecho = True
    if o == '-f':
        force = True
    if o == '-h':
        print usage
        sys.exit(0)
    if o == '-l':
        listonly = True
    if o == '-q':
        queue = a
    if o == '-v':
        verbose = True

for arg in args:
    exprs.append(re.compile(arg))

if not listonly and not onlyecho and isdir('Link'):
    print 'Checking for outdated files in Link directory'
    entries = listdir('Link')
    for entry in entries:
        link = joinpath('Link', entry)
        if not islink(link):
            continue

        base = joinpath('Base', entry)
        if not isfile(base) or not filecmp(link, base):
            print '%s is different than source %s...copying' % (base, link)
            copyfile(link, base)

import job, jobfile, pbs

test = jobfile.JobFile(joinpath('Base', 'test.py'))

joblist = []
for jobname in test.jobs:
    if not exprs:
        joblist.append(jobname)
        continue

    for expr in exprs:
        if expr.match(jobname):
            joblist.append(jobname)
            break

if listonly:
    if verbose:
        for jobname in joblist:
            test.printinfo(jobname)
    else:
        for jobname in joblist:
            print jobname
    sys.exit(0)

if not onlyecho:
    jl = []
    for jobname in joblist:
        if os.path.exists(jobname):
            if not force:
                if os.path.isfile(joinpath(jobname, '.success')):
                    continue

                if os.path.isfile(joinpath(jobname, '.start')) and \
                       not os.path.isfile(joinpath(jobname, '.stop')):
                    continue

            if not clean:
                sys.exit('job directory not clean!')

            job.cleandir(jobname)
        else:
            os.mkdir(jobname)
    jl.append(jobname)
    joblist = jl

rootdir = re.sub(r'^/\.automount/', r'/n/', os.getcwd())
for jobname in joblist:
    jobdir = joinpath(rootdir, jobname)

    if not onlyecho and not os.path.isdir(jobdir):
        sys.exit('%s is not a directory.  Cannot build job' % jobdir)

    print >>sys.stderr, 'Job name:       %s' % jobname
    print >>sys.stderr, 'Job directory:  %s' % jobdir

    qsub = pbs.qsub()
    qsub.pbshost = 'simpool.eecs.umich.edu'
    qsub.stdout = joinpath(jobdir, 'jobout')
    qsub.name = jobname
    qsub.join = True
    qsub.node_type = 'FAST'
    qsub.onlyecho = onlyecho
    qsub.env['ROOTDIR'] = rootdir
    qsub.verbose = verbose
    if len(queue):
        qsub.queue = queue

    qsub.do(joinpath('Base', 'job.py'))
    print >>sys.stderr, ''
