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

from __future__ import print_function
from __future__ import absolute_import

import optparse
import sys

from optparse import *

class nodefault(object): pass

class splitter(object):
    def __init__(self, split):
        self.split = split
    def __call__(self, option, opt_str, value, parser):
        values = value.split(self.split)
        dest = getattr(parser.values, option.dest)
        if dest is None:
            setattr(parser.values, option.dest, values)
        else:
            dest.extend(values)

class OptionParser(dict):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('formatter', optparse.TitledHelpFormatter())
        self._optparse = optparse.OptionParser(*args, **kwargs)
        self._optparse.disable_interspersed_args()

        self._allopts = {}

        # current option group
        self._group = self._optparse

    def set_defaults(self, *args, **kwargs):
        return self._optparse.set_defaults(*args, **kwargs)

    def set_group(self, *args, **kwargs):
        '''set the current option group'''
        if not args and not kwargs:
            self._group = self._optparse
        else:
            self._group = self._optparse.add_option_group(*args, **kwargs)

    def add_option(self, *args, **kwargs):
        '''add an option to the current option group, or global none set'''

        # if action=split, but allows the option arguments
        # themselves to be lists separated by the split variable'''

        if kwargs.get('action', None) == 'append' and 'split' in kwargs:
            split = kwargs.pop('split')
            kwargs['default'] = []
            kwargs['type'] = 'string'
            kwargs['action'] = 'callback'
            kwargs['callback'] = splitter(split)

        option = self._group.add_option(*args, **kwargs)
        dest = option.dest
        if dest not in self._allopts:
            self._allopts[dest] = option

        return option

    def bool_option(self, name, default, help):
        '''add a boolean option called --name and --no-name.
        Display help depending on which is the default'''

        tname = '--%s' % name
        fname = '--no-%s' % name
        dest = name.replace('-', '_')
        if default:
            thelp = optparse.SUPPRESS_HELP
            fhelp = help
        else:
            thelp = help
            fhelp = optparse.SUPPRESS_HELP

        topt = self.add_option(tname, action="store_true", default=default,
                               help=thelp)
        fopt = self.add_option(fname, action="store_false", dest=dest,
                               help=fhelp)

        return topt,fopt

    def __getattr__(self, attr):
        if attr.startswith('_'):
            return super(OptionParser, self).__getattribute__(attr)

        if attr in self:
            return self[attr]

        return super(OptionParser, self).__getattribute__(attr)

    def __setattr__(self, attr, value):
        if attr.startswith('_'):
            super(OptionParser, self).__setattr__(attr, value)
        elif attr in self._allopts:
            defaults = { attr : value }
            self.set_defaults(**defaults)
            if attr in self:
                self[attr] = value
        else:
            super(OptionParser, self).__setattr__(attr, value)

    def parse_args(self):
        opts,args = self._optparse.parse_args()

        for key,val in opts.__dict__.items():
            if val is not None or key not in self:
                self[key] = val

        return args

    def usage(self, exitcode=None):
        self._optparse.print_help()
        if exitcode is not None:
            sys.exit(exitcode)

