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

import sys

class ternary(object):
    def __new__(cls, *args):
        if len(args) > 1:
            raise TypeError, \
                  '%s() takes at most 1 argument (%d given)' % \
                  (cls.__name__, len(args))

        if args:
            if not isinstance(args[0], (bool, ternary)):
                raise TypeError, \
                      '%s() argument must be True, False, or Any' % \
                      cls.__name__
            return args[0]
        return super(ternary, cls).__new__(cls)

    def __bool__(self):
        return True

    def __neg__(self):
        return self

    def __eq__(self, other):
        return True

    def __ne__(self, other):
        return False

    def __str__(self):
        return 'Any'

    def __repr__(self):
        return 'Any'

Any = ternary()

class Flags(dict):
    def __init__(self, *args, **kwargs):
        super(Flags, self).__init__()
        self.update(*args, **kwargs)

    def __getattr__(self, attr):
        return self[attr]

    def __setattr__(self, attr, value):
        self[attr] = value

    def __setitem__(self, item, value):
        return super(Flags, self).__setitem__(item, ternary(value))

    def __getitem__(self, item):
        if item not in self:
            return False
        return super(Flags, self).__getitem__(item)

    def update(self, *args, **kwargs):
        for arg in args:
            if isinstance(arg, Flags):
                super(Flags, self).update(arg)
            elif isinstance(arg, dict):
                for key,val in kwargs.iteritems():
                    self[key] = val
            else:
                raise AttributeError, \
                      'flags not of type %s or %s, but %s' % \
                      (Flags, dict, type(arg))

        for key,val in kwargs.iteritems():
            self[key] = val

    def match(self, *args, **kwargs):
        match = Flags(*args, **kwargs)

        for key,value in match.iteritems():
            if self[key] != value:
                return False

        return True

def crossproduct(items):
    if not isinstance(items, (list, tuple)):
        raise AttributeError, 'crossproduct works only on sequences'

    if not items:
        yield None
        return

    current = items[0]
    remainder = items[1:]

    if not hasattr(current, '__iter__'):
        current = [ current ]

    for item in current:
        for rem in crossproduct(remainder):
            data = [ item ]
            if rem:
                data += rem
            yield data

def flatten(items):
    if not isinstance(items, (list, tuple)):
        yield items
        return

    for item in items:
        for flat in flatten(item):
            yield flat

class Data(object):
    def __init__(self, name, desc, **kwargs):
        self.name = name
        self.desc = desc
        self.system = None
        self.flags = Flags()
        self.env = {}
        for k,v in kwargs.iteritems():
            setattr(self, k, v)

    def update(self, obj):
        if not isinstance(obj, Data):
            raise AttributeError, "can only update from Data object"

        self.env.update(obj.env)
        self.flags.update(obj.flags)
        if obj.system:
            if self.system and self.system != obj.system:
                raise AttributeError, \
                      "conflicting values for system: '%s'/'%s'" % \
                      (self.system, obj.system)
            self.system = obj.system

    def printinfo(self):
        if self.name:
            print 'name: %s' % self.name
        if self.desc:
            print 'desc: %s' % self.desc
        if self.system:
            print 'system: %s' % self.system

    def printverbose(self):
        print 'flags:'
        keys = self.flags.keys()
        keys.sort()
        for key in keys:
            print '    %s = %s' % (key, self.flags[key])
        print 'env:'
        keys = self.env.keys()
        keys.sort()
        for key in keys:
            print '    %s = %s' % (key, self.env[key])
        print

    def __str__(self):
        return self.name

class Job(Data):
    def __init__(self, options):
        super(Job, self).__init__('', '')
        self.setoptions(options)

        self.checkpoint = False
        opts = []
        for opt in options:
            cpt = opt.group.checkpoint
            if not cpt:
                self.checkpoint = True
                continue
            if isinstance(cpt, Option):
                opt = cpt.clone(suboptions=False)
            else:
                opt = opt.clone(suboptions=False)

            opts.append(opt)

        if not opts:
            self.checkpoint = False

        if self.checkpoint:
            self.checkpoint = Job(opts)

    def clone(self):
        return Job(self.options)

    def __getattribute__(self, attr):
        if attr == 'name':
            names = [ ]
            for opt in self.options:
                if opt.name:
                    names.append(opt.name)
            return ':'.join(names)

        if attr == 'desc':
            descs = [ ]
            for opt in self.options:
                if opt.desc:
                    descs.append(opt.desc)
            return ', '.join(descs)

        return super(Job, self).__getattribute__(attr)

    def setoptions(self, options):
        config = options[0].config
        for opt in options:
            if opt.config != config:
                raise AttributeError, \
                      "All options are not from the same Configuration"

        self.config = config
        self.groups = [ opt.group for opt in options ]
        self.options = options

        self.update(self.config)
        for group in self.groups:
            self.update(group)

        for option in self.options:
            self.update(option)
            if option._suboption:
                self.update(option._suboption)

    def printinfo(self):
        super(Job, self).printinfo()
        if self.checkpoint:
            print 'checkpoint: %s' % self.checkpoint.name
        print 'config: %s' % self.config.name
        print 'groups: %s' % [ g.name for g in self.groups ]
        print 'options: %s' % [ o.name for o in self.options ]
        super(Job, self).printverbose()

class SubOption(Data):
    def __init__(self, name, desc, **kwargs):
        super(SubOption, self).__init__(name, desc, **kwargs)
        self.number = None

class Option(Data):
    def __init__(self, name, desc, **kwargs):
        super(Option, self).__init__(name, desc, **kwargs)
        self._suboptions = []
        self._suboption = None
        self.number = None

    def __getattribute__(self, attr):
        if attr == 'name':
            name = self.__dict__[attr]
            if self._suboption is not None:
                name = '%s:%s' % (name, self._suboption.name)
            return name

        if attr == 'desc':
            desc = [ self.__dict__[attr] ]
            if self._suboption is not None and self._suboption.desc:
                desc.append(self._suboption.desc)
            return ', '.join(desc)


        return super(Option, self).__getattribute__(attr)

    def suboption(self, name, desc, **kwargs):
        subo = SubOption(name, desc, **kwargs)
        subo.config = self.config
        subo.group = self.group
        subo.option = self
        subo.number = len(self._suboptions)
        self._suboptions.append(subo)
        return subo

    def clone(self, suboptions=True):
        option = Option(self.__dict__['name'], self.__dict__['desc'])
        option.update(self)
        option.group = self.group
        option.config = self.config
        option.number = self.number
        if suboptions:
            option._suboptions.extend(self._suboptions)
            option._suboption = self._suboption
        return option

    def subopts(self):
        if not self._suboptions:
            return [ self ]

        subopts = []
        for subo in self._suboptions:
            option = self.clone()
            option._suboption = subo
            subopts.append(option)

        return subopts

    def printinfo(self):
        super(Option, self).printinfo()
        print 'config: %s' % self.config.name
        super(Option, self).printverbose()

class Group(Data):
    def __init__(self, name, desc, **kwargs):
        super(Group, self).__init__(name, desc, **kwargs)
        self._options = []
        self.checkpoint = False
        self.number = None

    def option(self, name, desc, **kwargs):
        opt = Option(name, desc, **kwargs)
        opt.config = self.config
        opt.group = self
        opt.number = len(self._options)
        self._options.append(opt)
        return opt

    def options(self):
        return self._options

    def subopts(self):
        subopts = []
        for opt in self._options:
            for subo in opt.subopts():
                subopts.append(subo)
        return subopts

    def printinfo(self):
        super(Group, self).printinfo()
        print 'config: %s' % self.config.name
        print 'options: %s' % [ o.name for o in self._options ]
        super(Group, self).printverbose()

class Configuration(Data):
    def __init__(self, name, desc, **kwargs):
        super(Configuration, self).__init__(name, desc, **kwargs)
        self._groups = []
        self._posfilters = []
        self._negfilters = []

    def group(self, name, desc, **kwargs):
        grp = Group(name, desc, **kwargs)
        grp.config = self
        grp.number = len(self._groups)
        self._groups.append(grp)
        return grp

    def groups(self, flags=Flags(), sign=True):
        if not flags:
            return self._groups

        return [ grp for grp in self._groups if sign ^ grp.flags.match(flags) ]

    def checkchildren(self, kids):
        for kid in kids:
            if kid.config != self:
                raise AttributeError, "child from the wrong configuration"

    def sortgroups(self, groups):
        groups = [ (grp.number, grp) for grp in groups ]
        groups.sort()
        return [ grp[1] for grp in groups ]

    def options(self, groups = None, checkpoint = False):
        if groups is None:
            groups = self._groups
        self.checkchildren(groups)
        groups = self.sortgroups(groups)
        if checkpoint:
            groups = [ grp for grp in groups if grp.checkpoint ]
            optgroups = [ g.options() for g in groups ]
        else:
            optgroups = [ g.subopts() for g in groups ]
        for options in crossproduct(optgroups):
            for opt in options:
                cpt = opt.group.checkpoint
                if not isinstance(cpt, bool) and cpt != opt:
                    if checkpoint:
                        break
                    else:
                        yield options
            else:
                if checkpoint:
                    yield options

    def addfilter(self, filt, pos=True):
        import re
        filt = re.compile(filt)
        if pos:
            self._posfilters.append(filt)
        else:
            self._negfilters.append(filt)

    def jobfilter(self, job):
        for filt in self._negfilters:
            if filt.match(job.name):
                return False

        if not self._posfilters:
            return True

        for filt in self._posfilters:
            if filt.match(job.name):
                return True

        return False

    def checkpoints(self, groups = None):
        for options in self.options(groups, True):
            job = Job(options)
            if self.jobfilter(job):
                yield job

    def jobs(self, groups = None):
        for options in self.options(groups, False):
            job = Job(options)
            if self.jobfilter(job):
                yield job

    def alljobs(self, groups = None):
        for options in self.options(groups, True):
            yield Job(options)
        for options in self.options(groups, False):
            yield Job(options)

    def find(self, jobname):
        for job in self.alljobs():
            if job.name == jobname:
                return job
        else:
            raise AttributeError, "job '%s' not found" % jobname

    def job(self, options):
        self.checkchildren(options)
        options = [ (opt.group.number, opt) for opt in options ]
        options.sort()
        options = [ opt[1] for opt in options ]
        job = Job(options)
        return job

    def printinfo(self):
        super(Configuration, self).printinfo()
        print 'groups: %s' % [ g.name for g in self._grouips ]
        super(Configuration, self).printverbose()

def JobFile(jobfile):
    from os.path import expanduser, isfile, join as joinpath
    filename = expanduser(jobfile)

    # Can't find filename in the current path, search sys.path
    if not isfile(filename):
        for path in sys.path:
            testname = joinpath(path, filename)
            if isfile(testname):
                filename = testname
                break
        else:
            raise AttributeError, \
                  "Could not find file '%s'" % jobfile

    data = {}
    execfile(filename, data)
    if 'conf' not in data:
        raise ImportError, 'cannot import name conf from %s' % jobfile
    conf = data['conf']
    import jobfile
    if not isinstance(conf, Configuration):
        raise AttributeError, \
              'conf in jobfile: %s (%s) is not type %s' % \
              (jobfile, type(conf), Configuration)
    return conf

if __name__ == '__main__':
    from jobfile import *
    import sys

    usage = 'Usage: %s [-b] [-c] [-v] <jobfile>' % sys.argv[0]

    try:
        import getopt
        opts, args = getopt.getopt(sys.argv[1:], '-bcv')
    except getopt.GetoptError:
        sys.exit(usage)

    if len(args) != 1:
        raise AttributeError, usage

    both = False
    checkpoint = False
    verbose = False
    for opt,arg in opts:
        if opt == '-b':
            both = True
            checkpoint = True
        if opt == '-c':
            checkpoint = True
        if opt == '-v':
            verbose = True

    jobfile = args[0]
    conf = JobFile(jobfile)

    if both:
        gen = conf.alljobs()
    elif checkpoint:
        gen = conf.checkpoints()
    else:
        gen = conf.jobs()

    for job in gen:
        if not verbose:
            cpt = ''
            if job.checkpoint:
                cpt = job.checkpoint.name
            print job.name, cpt
        else:
            job.printinfo()
