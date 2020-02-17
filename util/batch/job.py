#!/usr/bin/env python2.7
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

import os, os.path, shutil, signal, socket, sys
from os import environ as env
from os.path import join as joinpath, expanduser

def date():
    import time
    return time.strftime('%a %b %e %H:%M:%S %Z %Y', time.localtime())

def cleandir(dir):
    for root, dirs, files in os.walk(dir, False):
        for name in files:
            os.remove(joinpath(root, name))
        for name in dirs:
            os.rmdir(joinpath(root, name))

class rsync:
    def __init__(self):
        self.sudo = False
        self.rsync = 'rsync'
        self.compress = False
        self.archive = True
        self.delete = False
        self.options = ''

    def do(self, src, dst):
        args = []
        if self.sudo:
            args.append('sudo')

        args.append(self.rsync)
        if (self.archive):
            args.append('-a')
        if (self.compress):
            args.append('-z')
        if (self.delete):
            args.append('--delete')
        if len(self.options):
            args.append(self.options)
        args.append(src)
        args.append(dst)

        return os.spawnvp(os.P_WAIT, args[0], args)

class JobDir(object):
    def __init__(self, dir):
        self.dir = dir

    def file(self, filename):
        return joinpath(self.dir, filename)

    def create(self):
        if os.path.exists(self.dir):
            if not os.path.isdir(self.dir):
                sys.exit('%s is not a directory.  Cannot build job' % self.dir)
        else:
            os.mkdir(self.dir)

    def exists(self):
        return os.path.isdir(self.dir)

    def clean(self):
        cleandir(self.dir)

    def hasfile(self, filename):
        return os.path.isfile(self.file(filename))

    def echofile(self, filename, string):
        filename = self.file(filename)
        try:
            f = file(filename, 'w')
            print >>f, string
            f.flush()
            f.close()
        except IOError,e:
            sys.exit(e)

    def rmfile(self, filename):
        filename = self.file(filename)
        if os.path.isfile(filename):
            os.unlink(filename)

    def readval(self, filename):
        filename = self.file(filename)
        f = file(filename, 'r')
        value = f.readline().strip()
        f.close()
        return value

    def setstatus(self, string):
        filename = self.file('.status')
        try:
            f = file(filename, 'a')
            print >>f, string
            f.flush()
            f.close()
        except IOError,e:
            sys.exit(e)

    def getstatus(self):
        filename = self.file('.status')
        try:
            f = file(filename, 'r')
        except IOError, e:
            return 'none'

        # fast forward to the end
        for line in f: pass

        # the first word on the last line is the status
        return line.split(' ')[0]

    def __str__(self):
        return self.dir

if __name__ == '__main__':
    import platform
    binaries = { 'i686' : 'm5.i386',
                 'x86_64' : 'm5.amd64' }
    binary = binaries[platform.machine()]

    cwd = os.getcwd()
    rootdir = env.setdefault('ROOTDIR', os.path.dirname(cwd))
    oar_jobid = int(env['OAR_JOBID'])
    oar_jobname = os.path.basename(cwd)
    #pbs_jobname = env['PBS_JOBNAME']
    basedir = joinpath(rootdir, 'Base')
    jobname = env.setdefault('JOBNAME', oar_jobname)
    jobfile = env.setdefault('JOBFILE', joinpath(rootdir, 'Test.py'))
    outdir = env.setdefault('OUTPUT_DIR', cwd)
    env['POOLJOB'] = 'True'

    if os.path.isdir("/work"):
        workbase = "/work"
    else:
        workbase = "/tmp/"

    workdir = joinpath(workbase, '%s.%s' % (env['USER'], oar_jobid))
    host = socket.gethostname()

    os.umask(0022)

    jobdir = JobDir(outdir)

    started = date()
    jobdir.echofile('.running', started)
    jobdir.rmfile('.queued')
    jobdir.echofile('.host', host)

    jobdir.setstatus('running on %s on %s' % (host, started))

    if os.path.isdir(workdir):
        cleandir(workdir)
    else:
        os.mkdir(workdir)

    if False and os.path.isdir('/z/dist'):
        sync = rsync()
        sync.delete = True
        sync.sudo = True
        sync.do('poolfs::dist/m5/', '/z/dist/m5/')

    try:
        os.chdir(workdir)
    except OSError,e:
        sys.exit(e)

    os.symlink(jobdir.file('output'), 'status.out')

    args = [ joinpath(basedir, binary), joinpath(basedir, 'run.py') ]
    if not len(args):
        sys.exit("no arguments")

    print 'starting job... %s' % started
    print ' '.join(args)
    print
    sys.stdout.flush()

    childpid = os.fork()
    if not childpid:
        # Execute command
        sys.stdin.close()
        fd = os.open(jobdir.file("output"),
                     os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(fd, sys.stdout.fileno())
        os.dup2(fd, sys.stderr.fileno())
        os.execvp(args[0], args)

    def handler(signum, frame):
        if childpid != 0:
            os.kill(childpid, signum)

    signal.signal(signal.SIGHUP, handler)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGQUIT, handler)
    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGCONT, handler)
    signal.signal(signal.SIGUSR1, handler)
    signal.signal(signal.SIGUSR2, handler)

    done = 0
    while not done:
        try:
            thepid,ec = os.waitpid(childpid, 0)
            if ec:
                print 'Exit code ', ec
                status = 'failure'
            else:
                status = 'success'
            done = 1
        except OSError:
            pass

    complete = date()
    print '\njob complete... %s' % complete
    jobdir.echofile('.%s' % status, complete)
    jobdir.rmfile('.running')
    jobdir.setstatus('%s on %s' % (status, complete))
