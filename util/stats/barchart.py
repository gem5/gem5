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

import matplotlib, pylab
from matplotlib.font_manager import FontProperties
from matplotlib.numerix import array, arange, reshape, shape, transpose, zeros
from matplotlib.numerix import Float
from matplotlib.ticker import NullLocator

matplotlib.interactive(False)

from chart import ChartOptions

class BarChart(ChartOptions):
    def __init__(self, default=None, **kwargs):
        super(BarChart, self).__init__(default, **kwargs)
        self.inputdata = None
        self.chartdata = None
        self.inputerr = None
        self.charterr = None

    def gen_colors(self, count):
        cmap = matplotlib.cm.get_cmap(self.colormap)
        if count == 1:
            return cmap([ 0.5 ])

        if count < 5:
            return cmap(arange(5) / float(4))[:count]

        return cmap(arange(count) / float(count - 1))

    # The input data format does not match the data format that the
    # graph function takes because it is intuitive.  The conversion
    # from input data format to chart data format depends on the
    # dimensionality of the input data.  Check here for the
    # dimensionality and correctness of the input data
    def set_data(self, data):
        if data is None:
            self.inputdata = None
            self.chartdata = None
            return

        data = array(data)
        dim = len(shape(data))
        if dim not in (1, 2, 3):
            raise AttributeError, "Input data must be a 1, 2, or 3d matrix"
        self.inputdata = data

        # If the input data is a 1d matrix, then it describes a
        # standard bar chart.
        if dim == 1:
            self.chartdata = array([[data]])

        # If the input data is a 2d matrix, then it describes a bar
        # chart with groups. The matrix being an array of groups of
        # bars.
        if dim == 2:
            self.chartdata = transpose([data], axes=(2,0,1))

        # If the input data is a 3d matrix, then it describes an array
        # of groups of bars with each bar being an array of stacked
        # values.
        if dim == 3:
            self.chartdata = transpose(data, axes=(1,2,0))

    def get_data(self):
        return self.inputdata

    data = property(get_data, set_data)

    def set_err(self, err):
        if err is None:
            self.inputerr = None
            self.charterr = None
            return

        err = array(err)
        dim = len(shape(err))
        if dim not in (1, 2, 3):
            raise AttributeError, "Input err must be a 1, 2, or 3d matrix"
        self.inputerr = err

        if dim == 1:
            self.charterr = array([[err]])

        if dim == 2:
            self.charterr = transpose([err], axes=(2,0,1))

        if dim == 3:
            self.charterr = transpose(err, axes=(1,2,0))

    def get_err(self):
        return self.inputerr

    err = property(get_err, set_err)

    # Graph the chart data.
    # Input is a 3d matrix that describes a plot that has multiple
    # groups, multiple bars in each group, and multiple values stacked
    # in each bar.  The underlying bar() function expects a sequence of
    # bars in the same stack location and same group location, so the
    # organization of the matrix is that the inner most sequence
    # represents one of these bar groups, then those are grouped
    # together to make one full stack of bars in each group, and then
    # the outer most layer describes the groups.  Here is an example
    # data set and how it gets plotted as a result.
    #
    # e.g. data = [[[10,11,12], [13,14,15],  [16,17,18], [19,20,21]],
    #              [[22,23,24], [25,26,27],  [28,29,30], [31,32,33]]]
    #
    # will plot like this:
    #
    #    19 31    20 32    21 33
    #    16 28    17 29    18 30
    #    13 25    14 26    15 27
    #    10 22    11 23    12 24
    #
    # Because this arrangement is rather conterintuitive, the rearrange
    # function takes various matricies and arranges them to fit this
    # profile.
    #
    # This code deals with one of the dimensions in the matrix being
    # one wide.
    #
    def graph(self):
        if self.chartdata is None:
            raise AttributeError, "Data not set for bar chart!"

        dim = len(shape(self.inputdata))
        cshape = shape(self.chartdata)
        if self.charterr is not None and shape(self.charterr) != cshape:
            raise AttributeError, 'Dimensions of error and data do not match'

        if dim == 1:
            colors = self.gen_colors(cshape[2])
            colors = [ [ colors ] * cshape[1] ] * cshape[0]

        if dim == 2:
            colors = self.gen_colors(cshape[0])
            colors = [ [ [ c ] * cshape[2] ] * cshape[1] for c in colors ]

        if dim == 3:
            colors = self.gen_colors(cshape[1])
            colors = [ [ [ c ] * cshape[2] for c in colors ] ] * cshape[0]

        colors = array(colors)

        self.figure = pylab.figure(figsize=self.chart_size)

        outer_axes = None
        inner_axes = None
        if self.xsubticks is not None:
            color = self.figure.get_facecolor()
            self.metaaxes = self.figure.add_axes(self.figure_size,
                                                 axisbg=color, frameon=False)
            for tick in self.metaaxes.xaxis.majorTicks:
                tick.tick1On = False
                tick.tick2On = False
            self.metaaxes.set_yticklabels([])
            self.metaaxes.set_yticks([])
            size = [0] * 4
            size[0] = self.figure_size[0]
            size[1] = self.figure_size[1] + .12
            size[2] = self.figure_size[2]
            size[3] = self.figure_size[3] - .12
            self.axes = self.figure.add_axes(size)
            outer_axes = self.metaaxes
            inner_axes = self.axes
        else:
            self.axes = self.figure.add_axes(self.figure_size)
            outer_axes = self.axes
            inner_axes = self.axes

        bars_in_group = len(self.chartdata)

        width = 1.0 / ( bars_in_group + 1)
        center = width / 2

        bars = []
        for i,stackdata in enumerate(self.chartdata):
            bottom = array([0.0] * len(stackdata[0]), Float)
            stack = []
            for j,bardata in enumerate(stackdata):
                bardata = array(bardata)
                ind = arange(len(bardata)) + i * width + center
                yerr = None
                if self.charterr is not None:
                    yerr = self.charterr[i][j]
                bar = self.axes.bar(ind, bardata, width, bottom=bottom,
                                    color=colors[i][j], yerr=yerr)
                if self.xsubticks is not None:
                    self.metaaxes.bar(ind, [0] * len(bardata), width)
                stack.append(bar)
                bottom += bardata
            bars.append(stack)

        if self.xlabel is not None:
            outer_axes.set_xlabel(self.xlabel)

        if self.ylabel is not None:
            inner_axes.set_ylabel(self.ylabel)

        if self.yticks is not None:
            ymin, ymax = self.axes.get_ylim()
            nticks = float(len(self.yticks))
            ticks = arange(nticks) / (nticks - 1) * (ymax - ymin)  + ymin
            inner_axes.set_yticks(ticks)
            inner_axes.set_yticklabels(self.yticks)
        elif self.ylim is not None:
            inner_axes.set_ylim(self.ylim)

        if self.xticks is not None:
            outer_axes.set_xticks(arange(cshape[2]) + .5)
            outer_axes.set_xticklabels(self.xticks)

        if self.xsubticks is not None:
            numticks = (cshape[0] + 1) * cshape[2]
            inner_axes.set_xticks(arange(numticks) * width + 2 * center)
            xsubticks = list(self.xsubticks) + [ '' ]
            inner_axes.set_xticklabels(xsubticks * cshape[2], fontsize=7,
                                       rotation=30)

        if self.legend is not None:
            if dim == 1:
                lbars = bars[0][0]
            if dim == 2:
                lbars = [ bars[i][0][0] for i in xrange(len(bars))]
            if dim == 3:
                number = len(bars[0])
                lbars = [ bars[0][number - j - 1][0] for j in xrange(number)]

            if self.fig_legend:
                self.figure.legend(lbars, self.legend, self.legend_loc,
                                   prop=FontProperties(size=self.legend_size))
            else:
                self.axes.legend(lbars, self.legend, self.legend_loc,
                                 prop=FontProperties(size=self.legend_size))

        if self.title is not None:
            self.axes.set_title(self.title)

    def savefig(self, name):
        self.figure.savefig(name)

    def savecsv(self, name):
        f = file(name, 'w')
        data = array(self.inputdata)
        dim = len(data.shape)

        if dim == 1:
            #if self.xlabel:
            #    f.write(', '.join(list(self.xlabel)) + '\n')
            f.write(', '.join([ '%f' % val for val in data]) + '\n')
        if dim == 2:
            #if self.xlabel:
            #    f.write(', '.join([''] + list(self.xlabel)) + '\n')
            for i,row in enumerate(data):
                ylabel = []
                #if self.ylabel:
                #    ylabel = [ self.ylabel[i] ]
                f.write(', '.join(ylabel + [ '%f' % v for v in row]) + '\n')
        if dim == 3:
            f.write("don't do 3D csv files\n")
            pass

        f.close()

if __name__ == '__main__':
    from random import randrange
    import random, sys

    dim = 3
    number = 5

    args = sys.argv[1:]
    if len(args) > 3:
        sys.exit("invalid number of arguments")
    elif len(args) > 0:
        myshape = [ int(x) for x in args ]
    else:
        myshape = [ 3, 4, 8 ]

    # generate a data matrix of the given shape
    size = reduce(lambda x,y: x*y, myshape)
    #data = [ random.randrange(size - i) + 10 for i in xrange(size) ]
    data = [ float(i)/100.0 for i in xrange(size) ]
    data = reshape(data, myshape)

    # setup some test bar charts
    if True:
        chart1 = BarChart()
        chart1.data = data

        chart1.xlabel = 'Benchmark'
        chart1.ylabel = 'Bandwidth (GBps)'
        chart1.legend = [ 'x%d' % x for x in xrange(myshape[-1]) ]
        chart1.xticks = [ 'xtick%d' % x for x in xrange(myshape[0]) ]
        chart1.title = 'this is the title'
        if len(myshape) > 2:
            chart1.xsubticks = [ '%d' % x for x in xrange(myshape[1]) ]
        chart1.graph()
        chart1.savefig('/tmp/test1.png')
        chart1.savefig('/tmp/test1.ps')
        chart1.savefig('/tmp/test1.eps')
        chart1.savecsv('/tmp/test1.csv')

    if False:
        chart2 = BarChart()
        chart2.data = data
        chart2.colormap = 'gray'
        chart2.graph()
        chart2.savefig('/tmp/test2.png')
        chart2.savefig('/tmp/test2.ps')

#    pylab.show()
