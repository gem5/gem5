#!/usr/bin/env python2
# Copyright (c) 2006 The Regents of The University of Michigan
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
# Authors: Kevin Lim

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
    -C           submit the checkpointing runs
    -d           Make jobs be dependent on the completion of the checkpoint runs
    -e           only echo pbs command info, don't actually send the job
    -f           force the job to run regardless of state
    -q <queue>   submit job to the named queue
    -j <jobfile> specify the jobfile (default is <rootdir>/Test.py)
    -v           be verbose

    %(progname)s [-j <jobfile>] -l [-v] <regexp>
    -j <jobfile> specify the jobfile (default is <rootdir>/Test.py)
    -l           list job names, don't submit
    -v           be verbose (list job parameters)

    %(progname)s -h
    -h           display this help
""" % locals()

try:
    import getopt
    opts, args = getopt.getopt(sys.argv[1:], '-Ccdefhj:lnq:Rt:v')
except getopt.GetoptError:
    sys.exit(usage)

depend = False
clean = False
onlyecho = False
exprs = []
force = False
listonly = False
queue = ''
verbose = False
jfile = 'Test.py'
docpts = False
doruns = True
runflag = False
node_type = 'FAST'
update = True

for opt,arg in opts:
    if opt == '-C':
        docpts = True
    if opt == '-c':
        clean = True
    if opt == '-d':
        depend = True
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
    if opt == '-n':
        update = False
    if opt == '-q':
        queue = arg
    if opt == '-R':
        runflag = True
    if opt == '-t':
        node_type = arg
    if opt == '-v':
        verbose = True

if docpts:
    doruns = runflag

for arg in args:
    exprs.append(re.compile(arg))

import jobfile, batch
from job import JobDir, date

conf = jobfile.JobFile(jfile)

if update and not listonly and not onlyecho and isdir(conf.linkdir):
    if verbose:
        print 'Checking for outdated files in Link directory'
    if not isdir(conf.basedir):
        os.mkdir(conf.basedir)
    syncdir(conf.linkdir, conf.basedir)

jobnames = {}
joblist = []

if docpts and doruns:
    gen = conf.alljobs()
elif docpts:
    gen = conf.checkpoints()
elif doruns:
    gen = conf.jobs()

for job in gen:
    if job.name in jobnames:
        continue

    if exprs:
        for expr in exprs:
            if expr.match(job.name):
                joblist.append(job)
                break
    else:
        joblist.append(job)

if listonly:
    if verbose:
        for job in joblist:
            job.printinfo()
    else:
        for job in joblist:
            print job.name
    sys.exit(0)

if not onlyecho:
    newlist = []
    for job in joblist:
        jobdir = JobDir(joinpath(conf.rootdir, job.name))
        if jobdir.exists():
            if not force:
                status = jobdir.getstatus()
                if status == 'queued':
                    continue

                if status == 'running':
                    continue

                if status == 'success':
                    continue

            if not clean:
                sys.exit('job directory %s not clean!' % jobdir)

            jobdir.clean()
        newlist.append(job)
    joblist = newlist

class NameHack(object):
    def __init__(self, host='pbs.pool', port=24465):
        self.host = host
        self.port = port
        self.socket = None

    def setname(self, jobid, jobname):
        try:
            jobid = int(jobid)
        except ValueError:
            jobid = int(jobid.strip().split('.')[0])

        jobname = jobname.strip()
        # since pbs can handle jobnames of 15 characters or less,
        # don't use the raj hack.
        if len(jobname) <= 15:
            return

        if self.socket is None:
            import socket
            self.socket = socket.socket()
            # Connect to pbs.pool and send the jobid/jobname pair to port
            # 24465 (Raj didn't realize that there are only 64k ports and
            # setup inetd to point to port 90001)
            self.socket.connect((self.host, self.port))

        self.socket.send("%s %s\n" % (jobid, jobname))

namehack = NameHack()

rootdir = conf.rootdir
script = joinpath(rootdir, 'Base', 'job.py')

for job in joblist:
    jobdir = JobDir(joinpath(rootdir, job.name))
    if depend:
        cptdir = JobDir(joinpath(rootdir, job.checkpoint.name))
        path = str(cptdir)
        if not isdir(path) or not isfile(joinpath(path, '.success')):
            continue

        cptjob = cptdir.readval('.batch_jobid')

    if not onlyecho:
        jobdir.create()
        os.chdir(str(jobdir))
        os.environ['PWD'] = str(jobdir)

    print 'Job name:       %s' % job.name
    print 'Job directory:  %s' % jobdir


    qsub = batch.oarsub()
    qsub.oarhost = 'poolfs.eecs.umich.edu'
    #qsub.stdout = jobdir.file('jobout')
    qsub.name = job.name
    qsub.walltime = '50'
    #qsub.join = True
    #qsub.node_type = node_type
    #qsub.env['ROOTDIR'] = conf.rootdir
    #qsub.env['JOBNAME'] = job.name
    #if depend:
    #    qsub.afterok = cptjob
    #if queue:
    #    qsub.queue = queue
    qsub.properties = "64bit = 'Yes' or 64bit = 'No'"
    qsub.build(script)

    if verbose:
        print 'cwd:    %s' % qsub.command
        print 'PBS Command:    %s' % qsub.command

    if not onlyecho:
        ec = qsub.do()
        if ec == 0:
            jobid = qsub.result
            print 'OAR Jobid:      %s' % jobid
            #namehack.setname(jobid, job.name)
            queued = date()
            jobdir.echofile('.batch_jobid', jobid)
            jobdir.echofile('.batch_jobname', job.name)
            jobdir.echofile('.queued', queued)
            jobdir.setstatus('queued on %s' % queued)
        else:
            print 'OAR Failed'
    print
    print
