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

import os, re, sys

def ssh(host, script, tty = False, user = ''):
    args = [ 'ssh', '-x' ]
    if user:
        args.append('-l' + user)
    if tty:
        args.append('-t')
    args.append(host)
    args.append(script)

    return os.spawnvp(os.P_WAIT, args[0], args)

class qsub:
    def __init__(self):
        self.hold = False
        self.join = False
        self.keep_stdout = False
        self.keep_stderr = False
        self.node_type = ''
        self.mail_abort = False
        self.mail_begin = False
        self.mail_end = False
        self.name = ''
        self.stdout = ''
        self.priority = 0
        self.queue = ''
        self.pbshost = ''
        self.qsub = 'qsub'
        self.env = {}
        self.onlyecho = False
        self.verbose = False

    def do(self, script, ):
        args = [self.qsub]

        if self.env:
            arg = '-v'
            arg += ','.join([ '%s=%s' % i for i in self.env.iteritems() ])
            args.append(arg)

        if self.hold:
            args.append('-h')

        if len(self.stdout):
            args.append('-olocalhost:' + self.stdout)

        if self.keep_stdout and self.keep_stderr:
            args.append('-koe')
        elif self.keep_stdout:
            args.append('-ko')
        elif self.keep_stderr:
            args.append('-ke')
        else:
            args.append('-kn')

        if self.join:
            args.append('-joe')

        if len(self.node_type):
            args.append('-lnodes=' + self.node_type)

        if self.mail_abort or self.mail_begin or self.mail_end:
            flags = ''
            if self.mail_abort:
                flags.append('a')
            if self.mail_begin:
                flags.append('b')
            if self.mail_end:
                flags.append('e')
            if len(flags):
                args.append('-m ' + flags)

        if len(self.name):
            args.append("-N%s" % self.name)

        if self.priority != 0:
            args.append('-p' + self.priority)

        if len(self.queue):
            args.append('-q' + self.queue)

        args.append(script)

        if self.verbose or self.onlyecho:
            print >>sys.stderr, 'PBS Command:   ', ' '.join(args)

        if self.onlyecho:
            return 0

        print >>sys.stderr, 'PBS Jobid:      ',

        ec = os.spawnvp(os.P_WAIT, args[0], args)

        if ec != 0 and len(self.pbshost):
            ec = ssh(self.pbshost, ' '.join(args))

        return ec
