# Copyright (c) 2009 The Hewlett-Packard Development Company
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

from __future__ import print_function

import os
import sys

class PairContainer(object):
    def __init__(self, pairs=None):
        self.pairs = {}
        if pairs:
            self.pairs.update(pairs)

    def __contains__(self, item):
        return item in self.pairs

    def __getitem__(self, item):
        return self.pairs[item]

    def __setitem__(self, item, value):
        self.pairs[item] = value

    def get(self, item, failobj=None):
        return self.pairs.get(item, failobj)

class Location(object):
    def __init__(self, filename, lineno, no_warning=False):
        if not isinstance(filename, basestring):
            raise AttributeError, \
                "filename must be a string, found '%s'" % (type(filename), )
        if not isinstance(lineno, (int, long)):
            raise AttributeError, \
                "filename must be an integer, found '%s'" % (type(lineno), )
        self.filename = filename
        self.lineno = lineno
        self.no_warning = no_warning

    def __str__(self):
        return '%s:%d' % (os.path.basename(self.filename), self.lineno)

    def warning(self, message, *args):
        if self.no_warning:
            return
        if args:
            message = message % args
        #raise Exception, "%s: Warning: %s" % (self, message)
        print("%s: Warning: %s" % (self, message), file=sys.stderr)

    def error(self, message, *args):
        if args:
            message = message % args
        raise Exception, "%s: Error: %s" % (self, message)
        sys.exit("\n%s: Error: %s" % (self, message))

__all__ = [ 'PairContainer', 'Location' ]
