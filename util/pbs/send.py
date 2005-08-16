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

import os, os.path, re, socket, sys
from os import environ as env, listdir
from os.path import basename, isdir, isfile, islink, join as joinpath, normpath
from filecmp import cmp as filecmp
from shutil import copy

def nfspath(dir):
    if dir.startswith('/.automount/'):
        dir = '/n/%s' % dir[12:]
    elif not dir.startswith('/n/'):
        dir = '/n/%s%s' % (socket.gethostname().split('.')[0], dir)
    return dir

def syncdir(srcdir, destdir):
    srcdir = normpath(srcdir)
    destdir = normpath(destdir)
    if not isdir(destdir):
        sys.exit('destination directory "%s" does not exist' % destdir)

    for root, dirs, files in os.walk(srcdir):
        root = normpath(root)
        prefix = os.path.commonprefix([root, srcdir])
        root = root[len(prefix):]
        if root.startswith('/'):
            root = root[1:]
        for rem in [ d for d in dirs if d.startswith('.') or d == 'SCCS']:
            dirs.remove(rem)

        for entry in dirs:
            newdir = joinpath(destdir, root, entry)
            if not isdir(newdir):
                os.mkdir(newdir)
                print 'mkdir', newdir

        for i,d in enumerate(dirs):
            if islink(joinpath(srcdir, root, d)):
                dirs[i] = joinpath(d, '.')

        for entry in files:
            dest = normpath(joinpath(destdir, root, entry))
            src = normpath(joinpath(srcdir, root, entry))
            if not isfile(dest) or not filecmp(src, dest):
                print 'copy %s %s' % (dest, src)
                copy(src, dest)

progpath = nfspath(sys.path[0])
progname = basename(sys.argv[0])
usage = """\
Usage:
    %(progname)s [-c] [-e] [-f] [-j <jobfile>] [-q queue] [-v] <regexp>
    -c           clean directory if job can be run
    -e           only echo pbs command info, don't actually send the job
    -f           force the job to run regardless of state
    -q <queue>   submit job to the named queue
    -j <jobfile> specify the jobfile (default is <basedir>/test.py)
    -v           be verbose

    %(progname)s [-j <jobfile>] -l [-v] <regexp>
    -j <jobfile> specify the jobfile (default is <basedir>/test.py)
    -l           list job names, don't submit
    -v           be verbose (list job parameters)

    %(progname)s -h
    -h           display this help
""" % locals()

try:
    import getopt
    opts, args = getopt.getopt(sys.argv[1:], '-cd:efhj:lq:v')
except getopt.GetoptError:
    sys.exit(usage)

clean = False
onlyecho = False
exprs = []
force = False
listonly = False
queue = ''
verbose = False
rootdir = nfspath(os.getcwd())
jfile = 'test.py'
for opt,arg in opts:
    if opt == '-c':
        clean = True
    if opt == '-d':
        rootdir = arg
    if opt == '-e':
        onlyecho = True
    if opt == '-f':
        force = True
    if opt == '-h':
        print usage
        sys.exit(0)
    if opt == '-j':
        jfile = arg
    if opt == '-l':
        listonly = True
    if opt == '-q':
        queue = arg
    if opt == '-v':
        verbose = True

basedir = joinpath(rootdir, 'Base')
linkdir = joinpath(rootdir, 'Link')

for arg in args:
    exprs.append(re.compile(arg))

if not listonly and not onlyecho and isdir(linkdir):
    if verbose:
        print 'Checking for outdated files in Link directory'
    syncdir(linkdir, basedir)

import job, jobfile, pbs

test = jobfile.JobFile(joinpath(basedir, jfile))

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
        jobdir = joinpath(rootdir, jobname)
        if os.path.exists(jobname):
            if not force:
                if os.path.isfile(joinpath(jobdir, '.success')):
                    continue

                if os.path.isfile(joinpath(jobdir, '.start')) and \
                       not os.path.isfile(joinpath(jobdir, '.stop')):
                    continue

            if not clean:
                sys.exit('job directory not clean!')

            job.cleandir(jobdir)
        else:
            os.mkdir(jobdir)
        jl.append(jobname)
    joblist = jl

def setname(jobid, jobname):
    # since pbs can handle jobnames of 15 characters or less, don't
    # use the raj hack.
    if len(jobname) <= 15:
        return

    import socket
    s = socket.socket()
    # Connect to pbs.pool and send the jobid/jobname pair to port
    # 24465 (Raj didn't realize that there are only 64k ports and
    # setup inetd to point to port 90001)
    s.connect(("pbs.pool", 24465))
    s.send("%s %s\n" % (jobid, jobname))
    s.close()

for jobname in joblist:
    jobdir = joinpath(rootdir, jobname)

    if not onlyecho and not os.path.isdir(jobdir):
        sys.exit('%s is not a directory.  Cannot build job' % jobdir)

    print 'Job name:       %s' % jobname
    print 'Job directory:  %s' % jobdir

    qsub = pbs.qsub()
    qsub.pbshost = 'simpool.eecs.umich.edu'
    qsub.stdout = joinpath(jobdir, 'jobout')
    qsub.name = jobname[:15]
    qsub.join = True
    qsub.node_type = 'FAST'
    qsub.env['ROOTDIR'] = rootdir
    qsub.env['JOBNAME'] = jobname
    if len(queue):
        qsub.queue = queue
    qsub.build(joinpath(progpath, 'job.py'))

    if verbose:
        print 'PBS Command:    %s' % qsub.command

    if not onlyecho:
        ec = qsub.do()
        if ec == 0:
            jobid = qsub.result
            print 'PBS Jobid:      %s' % jobid
            setname(jobid, jobname)
        else:
            print 'PBS Failed'
