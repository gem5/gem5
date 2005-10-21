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

class StatOutput(object):
    def __init__(self, name, jobfile, info, stat=None, binstats=None):
        self.name = name
        self.jobfile = jobfile
        self.stat = stat
        self.binstats = None
        self.label = self.name
        self.invert = False
        self.info = info

    def printdata(self, bin = None, printmode = 'G'):
        import info

        if bin:
            print '%s %s stats' % (self.name, bin)

        if self.binstats:
            for stat in self.binstats:
                stat.bins = bin

        if printmode == 'G':
            valformat = '%g'
        elif printmode != 'F' and value > 1e6:
            valformat = '%0.5e'
        else:
            valformat = '%f'

        for job in self.jobfile.jobs():
            value = self.info.get(job, self.stat)
            if value is None:
                return

            if not isinstance(value, list):
                value = [ value ]

            if self.invert:
                for i,val in enumerate(value):
                    if val != 0.0:
                        value[i] = 1 / val

            valstring = ', '.join([ valformat % val for val in value ])
            print '%-50s    %s' % (job.name + ':', valstring)

    def display(self, binned = False, printmode = 'G'):
        if binned and self.binstats:
            self.printdata('kernel', printmode)
            self.printdata('idle', printmode)
            self.printdata('user', printmode)
            self.printdata('interrupt', printmode)

            print '%s total stats' % self.name
        self.printdata(printmode=printmode)

    def graph(self, graphdir):
        from os.path import expanduser, isdir, join as joinpath
        from barchart import BarChart
        from matplotlib.numerix import Float, array, zeros
        import os, re

        confgroups = self.jobfile.groups()
        ngroups = len(confgroups)
        skiplist = [ False ] * ngroups
        groupopts = None
        baropts = None
        groups = []
        for i,group in enumerate(confgroups):
            if group.flags.graph_group:
                if groupopts is not None:
                    raise AttributeError, \
                          'Two groups selected for graph group'
                groupopts = group.subopts()
                skiplist[i] = True
            elif group.flags.graph_bars:
                if baropts is not None:
                    raise AttributeError, \
                          'Two groups selected for graph bars'
                baropts = group.subopts()
                skiplist[i] = True
            else:
                groups.append(group)

        if groupopts is None:
            raise AttributeError, 'No group selected for graph group'

        if baropts is None:
            raise AttributeError, 'No group selected for graph bars'

        directory = expanduser(graphdir)
        if not isdir(directory):
            os.mkdir(directory)
        html = file(joinpath(directory, '%s.html' % self.name), 'w')
        print >>html, '<html>'
        print >>html, '<title>Graphs for %s</title>' % self.name
        print >>html, '<body>'

        for options in self.jobfile.options(groups):
            data = zeros((len(groupopts), len(baropts)), Float)
            data = [ [ None ] * len(baropts) for i in xrange(len(groupopts)) ]
            enabled = False
            stacked = 0
            for g,gopt in enumerate(groupopts):
                for b,bopt in enumerate(baropts):
                    job = self.jobfile.job(options + [ gopt, bopt ])
                    if not job:
                        continue

                    val = self.info.get(job, self.stat)
                    if isinstance(val, (list, tuple)):
                        if len(val) == 1:
                            val = val[0]
                        else:
                            stacked = len(val)

                    data[g][b] = val

            if stacked == 0:
                for i in xrange(len(groupopts)):
                    for j in xrange(len(baropts)):
                        if data[i][j] is None:
                            data[i][j] = 0.0
            else:
                for i in xrange(len(groupopts)):
                    for j in xrange(len(baropts)):
                        val = data[i][j]
                        if val is None:
                            data[i][j] = [] * stacked
                        elif len(val) != stacked:
                            raise ValueError, "some stats stacked, some not"

            data = array(data)
            if data.sum() == 0:
                continue

            bar_descs = [ opt.desc for opt in baropts ]
            group_descs = [ opt.desc for opt in groupopts ]
            if stacked:
                try:
                    legend = self.info.rcategories
                except:
                    legend = [ str(i) for i in xrange(stacked) ]
            else:
                legend = bar_descs

            chart = BarChart(data=data, xlabel='Benchmark', ylabel=self.label,
                             legend=legend, xticks=group_descs)
            chart.graph()

            names = [ opt.name for opt in options ]
            descs = [ opt.desc for opt in options ]

            filename =  '%s-%s.png' % (self.name, ':'.join(names))
            desc = ' '.join(descs)
            filepath = joinpath(directory, filename)
            chart.savefig(filepath)
            filename = re.sub(':', '%3A', filename)
            print >>html, '''%s<br><img src="%s"><br>''' % (desc, filename)

        print >>html, '</body>'
        print >>html, '</html>'
        html.close()
