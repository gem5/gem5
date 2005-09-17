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

from orderdict import orderdict
import output

class ProfileData(object):
    def __init__(self):
        self.data = {}
        self.total = {}
        self.runs = orderdict()
        self.runlist = []

    def addvalue(self, run, cpu, symbol, value):
        value = float(value)
        self.data[run, cpu, symbol] = self.getvalue(run, cpu, symbol) + value
        self.total[run, cpu] = self.gettotal(run, cpu) + value
        if run not in self.runs:
            self.runs[run] = orderdict()

        if cpu not in self.runs[run]:
            self.runs[run][cpu] = {}

        if symbol not in self.runs[run][cpu]:
            self.runs[run][cpu][symbol] = 0

        self.runs[run][cpu][symbol] += value

    def getvalue(self, run, cpu, symbol):
        return self.data.get((run, cpu, symbol), 0)

    def gettotal(self, run, cpu):
        return self.total.get((run, cpu), 0)

class Profile(object):
    default_order = ['ste', 'hte', 'htd', 'ocm', 'occ', 'ocp']

    # This list controls the order of values in stacked bar data output
    default_categories = [ 'interrupt',
                           'driver',
                           'stack',
                           'bufmgt',
                           'copy',
                           'user',
                           'other',
                           'idle']

    def __init__(self, run_order=[], categories=[], stacknames=[]):
        if not run_order:
            run_order = Profile.default_order
        if not categories:
            categories = Profile.default_categories

        self.run_order = run_order
        self.categories = categories
        self.rcategories = []
        self.rcategories.extend(categories)
        self.rcategories.reverse()
        self.stacknames = stacknames
        self.prof = ProfileData()
        self.categorize = True
        self.showidle = True
        self.maxsymlen = 0

    def category(self, symbol):
        from categories import categories, categories_re
        if categories.has_key(symbol):
            return categories[symbol]
        for regexp, cat in categories_re:
            if regexp.match(symbol):
                return cat
        return 'other'

    # Parse input file and put the results in the given run and cpu
    def parsefile(self, run, cpu, filename):
        fd = file(filename)

        for line in fd:
            (symbol, count) = line.split()
            if symbol == "0x0":
                continue
            count = int(count)

            if self.categorize:
                symbol = self.category(symbol)
                if symbol == 'idle' and not self.showidle:
                    continue

                if symbol not in self.categories:
                    symbol = 'other'

            self.maxsymlen = max(self.maxsymlen, len(symbol))
            self.prof.addvalue(run, cpu, symbol, count)

        fd.close()

    # Read in files
    def inputdir(self, directory):
        import os, os.path, re
        from os.path import expanduser, join as joinpath

        directory = expanduser(directory)
        label_ex = re.compile(r'm5prof\.(.*)')
        for root,dirs,files in os.walk(directory):
            for name in files:
                match = label_ex.match(name)
                if not match:
                    continue

                filename = joinpath(root, name)
                prefix = os.path.commonprefix([root, directory])
                dirname = root[len(prefix)+1:]
                self.parsefile(dirname, match.group(1), filename)

    def get(self, job, stat):
        if job.system is None:
            raise AttributeError, 'The job must have a system set'

        cpu =  '%s.full0' % job.system
        values = []
        for cat in self.categories:
            values.append(self.prof.getvalue(job.name, cpu, cat))
        return values
