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

from os.path import expanduser, isfile, join as joinpath
import sys

def crossproduct(options):
    number = len(options)
    indexes = [ 0 ] * number
    maxes = [ len(opt) for opt in options ]
    def next():
        for i in xrange(number - 1, -1, -1):
            indexes[i] += 1
            if indexes[i] < maxes[i]:
                return False

            indexes[i] = 0
        return True

    done = False
    while not done:
        result = []
        for i in xrange(number):
            result.append(options[i][indexes[i]])
        yield result
        done = next()

class JobFile(object):
    def __init__(self, jfile):
        self.data = {}
        jfile = expanduser(jfile)
        if not isfile(jfile):
            for p in sys.path:
                if isfile(joinpath(p, jfile)):
                    jfile = joinpath(p, jfile)
                    break

        execfile(jfile, self.data)
        self.options = self.data['options']
        self.environment = self.data['environment']
        self.jobinfo = {}
        self.jobs = []
        for job in crossproduct(self.options):
            jobname = '.'.join([ id[0] for id in job ])
            self.jobs.append(jobname)
            list = []
            for info in job:
                for item in info[1:]:
                    list.append(item)
            self.jobinfo[jobname] = list

    def env(self, jobname):
        env = {}
        for key,val in self.jobinfo[jobname]:
            env[key] = val

        for key,val in self.environment:
            env[key] = val
        return env

    def printinfo(self, jobname):
        print '%s:' % jobname
        for key,val in self.jobinfo[jobname]:
            print '    %s = %s' % (key, val)

        for key,val in self.environment:
            print '    %s = %s' % (key, val)
