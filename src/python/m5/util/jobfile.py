# Copyright (c) 2005-2006 The Regents of The University of Michigan
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

from __future__ import print_function

import sys

class Data(object):
    def __init__(self, name, desc, **kwargs):
        self.name = name
        self.desc = desc
        self.__dict__.update(kwargs)

    def update(self, obj):
        if not isinstance(obj, Data):
            raise AttributeError, "can only update from Data object"

        for key,val in obj.__dict__.iteritems():
            if key.startswith('_') or key in ('name', 'desc'):
                continue

            if key not in self.__dict__:
                self.__dict__[key] = val
                continue

            if not isinstance(val, dict):
                if self.__dict__[key] == val:
                    continue

                raise AttributeError, \
                      "%s specified more than once old: %s new: %s" % \
                      (key, self.__dict__[key], val)

            d = self.__dict__[key]
            for k,v in val.iteritems():
                if k in d:
                    raise AttributeError, \
                          "%s specified more than once in %s" % (k, key)
                d[k] = v

        if hasattr(self, 'system') and hasattr(obj, 'system'):
            if self.system != obj.system:
                raise AttributeError, \
                      "conflicting values for system: '%s'/'%s'" % \
                      (self.system, obj.system)

    def printinfo(self):
        if self.name:
            print('name: %s' % self.name)
        if self.desc:
            print('desc: %s' % self.desc)
        try:
            if self.system:
                print('system: %s' % self.system)
        except AttributeError:
            pass

    def printverbose(self):
        for key in self:
            val = self[key]
            if isinstance(val, dict):
                import pprint
                val = pprint.pformat(val)
            print('%-20s = %s' % (key, val))
        print()

    def __contains__(self, attr):
        if attr.startswith('_'):
            return False
        return attr in self.__dict__

    def __getitem__(self, key):
        if key.startswith('_'):
            raise KeyError, "Key '%s' not found" % attr
        return self.__dict__[key]

    def __iter__(self):
        keys = self.__dict__.keys()
        keys.sort()
        for key in keys:
            if not key.startswith('_'):
                yield key

    def optiondict(self):
        import m5.util
        result = m5.util.optiondict()
        for key in self:
            result[key] = self[key]
        return result

    def __repr__(self):
        d = {}
        for key,value in self.__dict__.iteritems():
            if not key.startswith('_'):
                d[key] = value

        return "<%s: %s>" % (type(self).__name__, d)

    def __str__(self):
        return self.name

class Job(Data):
    def __init__(self, options):
        super(Job, self).__init__('', '')

        config = options[0]._config
        for opt in options:
            if opt._config != config:
                raise AttributeError, \
                      "All options are not from the same Configuration"

        self._config = config
        self._groups = [ opt._group for opt in options ]
        self._options = options

        self.update(self._config)
        for group in self._groups:
            self.update(group)

        self._is_checkpoint = True

        for option in self._options:
            self.update(option)
            if not option._group._checkpoint:
                self._is_checkpoint = False

            if option._suboption:
                self.update(option._suboption)
                self._is_checkpoint = False

        names = [ ]
        for opt in self._options:
            if opt.name:
                names.append(opt.name)
        self.name = ':'.join(names)

        descs = [ ]
        for opt in self._options:
            if opt.desc:
                descs.append(opt.desc)
        self.desc = ', '.join(descs)

        self._checkpoint = None
        if not self._is_checkpoint:
            opts = []
            for opt in options:
                cpt = opt._group._checkpoint
                if not cpt:
                    continue
                if isinstance(cpt, Option):
                    opt = cpt.clone(suboptions=False)
                else:
                    opt = opt.clone(suboptions=False)

                opts.append(opt)

            if opts:
                self._checkpoint = Job(opts)

    def clone(self):
        return Job(self._options)

    def printinfo(self):
        super(Job, self).printinfo()
        if self._checkpoint:
            print('checkpoint: %s' % self._checkpoint.name)
        print('config: %s' % self._config.name)
        print('groups: %s' % [ g.name for g in self._groups ])
        print('options: %s' % [ o.name for o in self._options ])
        super(Job, self).printverbose()

class SubOption(Data):
    def __init__(self, name, desc, **kwargs):
        super(SubOption, self).__init__(name, desc, **kwargs)
        self._number = None

class Option(Data):
    def __init__(self, name, desc, **kwargs):
        super(Option, self).__init__(name, desc, **kwargs)
        self._suboptions = []
        self._suboption = None
        self._number = None

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
        subo._config = self._config
        subo._group = self._group
        subo._option = self
        subo._number = len(self._suboptions)
        self._suboptions.append(subo)
        return subo

    def clone(self, suboptions=True):
        option = Option(self.__dict__['name'], self.__dict__['desc'])
        option.update(self)
        option._group = self._group
        option._config = self._config
        option._number = self._number
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
        print('config: %s' % self._config.name)
        super(Option, self).printverbose()

class Group(Data):
    def __init__(self, name, desc, **kwargs):
        super(Group, self).__init__(name, desc, **kwargs)
        self._options = []
        self._number = None
        self._checkpoint = False

    def option(self, name, desc, **kwargs):
        opt = Option(name, desc, **kwargs)
        opt._config = self._config
        opt._group = self
        opt._number = len(self._options)
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
        print('config: %s' % self._config.name)
        print('options: %s' % [ o.name for o in self._options ])
        super(Group, self).printverbose()

class Configuration(Data):
    def __init__(self, name, desc, **kwargs):
        super(Configuration, self).__init__(name, desc, **kwargs)
        self._groups = []
        self._posfilters = []
        self._negfilters = []

    def group(self, name, desc, **kwargs):
        grp = Group(name, desc, **kwargs)
        grp._config = self
        grp._number = len(self._groups)
        self._groups.append(grp)
        return grp

    def groups(self):
        return self._groups

    def checkchildren(self, kids):
        for kid in kids:
            if kid._config != self:
                raise AttributeError, "child from the wrong configuration"

    def sortgroups(self, groups):
        groups = [ (grp._number, grp) for grp in groups ]
        groups.sort()
        return [ grp[1] for grp in groups ]

    def options(self, groups=None, checkpoint=False):
        if groups is None:
            groups = self._groups
        self.checkchildren(groups)
        groups = self.sortgroups(groups)
        if checkpoint:
            groups = [ grp for grp in groups if grp._checkpoint ]
            optgroups = [ g.options() for g in groups ]
        else:
            optgroups = [ g.subopts() for g in groups ]
        if not optgroups:
            return

        import m5.util
        for options in m5.util.crossproduct(optgroups):
            for opt in options:
                cpt = opt._group._checkpoint
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

    def checkpoints(self, groups=None):
        for options in self.options(groups, True):
            job = Job(options)
            if self.jobfilter(job):
                yield job

    def jobs(self, groups=None):
        for options in self.options(groups, False):
            job = Job(options)
            if self.jobfilter(job):
                yield job

    def alljobs(self, groups=None):
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
        options = [ (opt._group._number, opt) for opt in options ]
        options.sort()
        options = [ opt[1] for opt in options ]
        job = Job(options)
        return job

    def printinfo(self):
        super(Configuration, self).printinfo()
        print('groups: %s' % [ g.name for g in self._groups ])
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
    return data['conf']

def main(conf=None):
    usage = 'Usage: %s [-b] [-c] [-v]' % sys.argv[0]
    if conf is None:
        usage += ' <jobfile>'

    try:
        import getopt
        opts, args = getopt.getopt(sys.argv[1:], '-bcv')
    except getopt.GetoptError:
        sys.exit(usage)

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

    if conf is None:
        if len(args) != 1:
            raise AttributeError, usage
        conf = JobFile(args[0])
    else:
        if len(args) != 0:
            raise AttributeError, usage

    if both:
        jobs = conf.alljobs()
    elif checkpoint:
        jobs = conf.checkpoints()
    else:
        jobs = conf.jobs()

    for job in jobs:
        if verbose:
            job.printinfo()
        else:
            cpt = ''
            if job._checkpoint:
                cpt = job._checkpoint.name
            print(job.name, cpt)

if __name__ == '__main__':
    main()
