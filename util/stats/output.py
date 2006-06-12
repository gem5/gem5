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

from chart import ChartOptions

class StatOutput(ChartOptions):
    def __init__(self, jobfile, info, stat=None):
        super(StatOutput, self).__init__()
        self.jobfile = jobfile
        self.stat = stat
        self.invert = False
        self.info = info

    def display(self, name, printmode = 'G'):
        import info

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

    def graph(self, name, graphdir, proxy=None):
        from os.path import expanduser, isdir, join as joinpath
        from barchart import BarChart
        from matplotlib.numerix import Float, array, zeros
        import os, re, urllib
        from jobfile import crossproduct

        confgroups = self.jobfile.groups()
        ngroups = len(confgroups)
        skiplist = [ False ] * ngroups
        groupopts = []
        baropts = []
        groups = []
        for i,group in enumerate(confgroups):
            if group.flags.graph_group:
                groupopts.append(group.subopts())
                skiplist[i] = True
            elif group.flags.graph_bars:
                baropts.append(group.subopts())
                skiplist[i] = True
            else:
                groups.append(group)

        has_group = bool(groupopts)
        if has_group:
            groupopts = [ group for group in crossproduct(groupopts) ]
        else:
            groupopts = [ None ]

        if baropts:
            baropts = [ bar for bar in crossproduct(baropts) ]
        else:
            raise AttributeError, 'No group selected for graph bars'

        directory = expanduser(graphdir)
        if not isdir(directory):
            os.mkdir(directory)
        html = file(joinpath(directory, '%s.html' % name), 'w')
        print >>html, '<html>'
        print >>html, '<title>Graphs for %s</title>' % name
        print >>html, '<body>'
        html.flush()

        for options in self.jobfile.options(groups):
            chart = BarChart(self)

            data = [ [ None ] * len(baropts) for i in xrange(len(groupopts)) ]
            enabled = False
            stacked = 0
            for g,gopt in enumerate(groupopts):
                for b,bopt in enumerate(baropts):
                    if gopt is None:
                        gopt = []
                    job = self.jobfile.job(options + gopt + bopt)
                    if not job:
                        continue

                    if proxy:
                        import db
                        proxy.dict['system'] = self.info[job.system]
                    val = self.info.get(job, self.stat)
                    if val is None:
                        print 'stat "%s" for job "%s" not found' % \
                              (self.stat, job)

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
                            data[i][j] = [ 0.0 ] * stacked
                        elif len(val) != stacked:
                            raise ValueError, "some stats stacked, some not"

            data = array(data)
            if data.sum() == 0:
                continue

            dim = len(data.shape)
            x = data.shape[0]
            xkeep = [ i for i in xrange(x) if data[i].sum() != 0 ]
            y = data.shape[1]
            ykeep = [ i for i in xrange(y) if data[:,i].sum() != 0 ]
            data = data.take(xkeep, axis=0)
            data = data.take(ykeep, axis=1)
            if not has_group:
                data = data.take([ 0 ], axis=0)
            chart.data = data


            bopts = [ baropts[i] for i in ykeep ]
            bdescs = [ ' '.join([o.desc for o in opt]) for opt in bopts]

            if has_group:
                gopts = [ groupopts[i] for i in xkeep ]
                gdescs = [ ' '.join([o.desc for o in opt]) for opt in gopts]

            if chart.legend is None:
                if stacked:
                    try:
                        chart.legend = self.info.rcategories
                    except:
                        chart.legend = [ str(i) for i in xrange(stacked) ]
                else:
                    chart.legend = bdescs

            if chart.xticks is None:
                if has_group:
                    chart.xticks = gdescs
                else:
                    chart.xticks = []
            chart.graph()

            names = [ opt.name for opt in options ]
            descs = [ opt.desc for opt in options ]

            if names[0] == 'run':
                names = names[1:]
                descs = descs[1:]

            basename = '%s-%s' % (name, ':'.join(names))
            desc = ' '.join(descs)

            pngname = '%s.png' % basename
            psname = '%s.eps' % re.sub(':', '-', basename)
            epsname = '%s.ps' % re.sub(':', '-', basename)
            chart.savefig(joinpath(directory, pngname))
            chart.savefig(joinpath(directory, epsname))
            chart.savefig(joinpath(directory, psname))
            html_name = urllib.quote(pngname)
            print >>html, '''%s<br><img src="%s"><br>''' % (desc, html_name)
            html.flush()

        print >>html, '</body>'
        print >>html, '</html>'
        html.close()
