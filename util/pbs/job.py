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
# Authors: Nathan Binkert
#          Steve Reinhardt
#          Ali Saidi

import os, os.path, shutil, signal, socket, sys, time
from os import environ as env
from os.path import join as joinpath, expanduser

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

def cleandir(dir):
    for root, dirs, files in os.walk(dir, False):
        for name in files:
            os.remove(joinpath(root, name))
        for name in dirs:
            os.rmdir(joinpath(root, name))

def date():
    return time.strftime('%a %b %e %H:%M:%S %Z %Y', time.localtime())

def remfile(file):
    if os.path.isfile(file):
        os.unlink(file)

def readval(filename):
    file = open(filename, 'r')
    value = file.readline().strip()
    file.close()
    return value

if __name__ == '__main__':
    rootdir = env.setdefault('ROOTDIR', os.getcwd())
    jobid = env['PBS_JOBID']
    jobname = env['PBS_JOBNAME']
    jobdir = joinpath(rootdir, jobname)
    basedir = joinpath(rootdir, 'Base')
    user = env['USER']

    env['POOLJOB'] = 'True'
    env['OUTPUT_DIR'] = jobdir
    env['JOBFILE'] = joinpath(basedir, 'test.py')
    env['JOBNAME'] = jobname

    def echofile(filename, string):
        try:
            f = file(joinpath(jobdir, filename), 'w')
            print >>f, string
            f.flush()
            f.close()
        except IOError,e:
            sys.exit(e)

    if os.path.isdir("/work"):
        workbase = "/work"
    else:
        workbase = "/tmp/"

    workdir = joinpath(workbase, '%s.%s' % (user, jobid))

    os.umask(0022)

    echofile('.start', date())
    echofile('.jobid', jobid)
    echofile('.host', socket.gethostname())

    if os.path.isdir(workdir):
        cleandir(workdir)
    else:
        os.mkdir(workdir)

    if os.path.isdir('/z/dist'):
        sync = rsync()
        sync.delete = True
        sync.sudo = True
        sync.do('poolfs::dist/m5/', '/z/dist/m5/')

    try:
        os.chdir(workdir)
    except OSError,e:
        sys.exit(e)

    os.symlink(joinpath(jobdir, 'output'), 'status.out')

    args = [ joinpath(basedir, 'm5'), '-d', '%s' % jobdir, joinpath(basedir, 'run.mpy') ]
    if not len(args):
        sys.exit("no arguments")

    print 'starting job... %s' % date()
    print ' '.join(args)
    print
    sys.stdout.flush()

    childpid = os.fork()
    if not childpid:
        # Execute command
        sys.stdin.close()
        fd = os.open(joinpath(jobdir, "output"),
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
    signal.signal(signal.SIGSTOP, handler)
    signal.signal(signal.SIGCONT, handler)
    signal.signal(signal.SIGUSR1, handler)
    signal.signal(signal.SIGUSR2, handler)

    done = 0
    while not done:
        try:
            thepid,ec = os.waitpid(childpid, 0)
            if ec:
                print 'Exit code ', ec
                echofile('.failure', date())
            else:
                echofile('.success', date())
            done = 1
        except OSError:
            pass

    print '\njob complete... %s' % date()
    echofile('.stop', date())
