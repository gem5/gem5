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
#          Lisa Hsu

class ChartOptions(object):
    defaults = { 'chart_size' : (8, 4),
                 'figure_size' : [0.1, 0.1, 0.6, 0.85],
                 'title' : None,
                 'fig_legend' : True,
                 'legend' : None,
                 'legend_loc' : 'upper right',
                 'legend_size' : 6,
                 'colormap' : 'jet',
                 'xlabel' : None,
                 'ylabel' : None,
                 'xticks' : None,
                 'xsubticks' : None,
                 'yticks' : None,
                 'ylim' : None,
                 }

    def __init__(self, options=None, **kwargs):
        self.init(options, **kwargs)

    def clear(self):
        self.options = {}

    def init(self, options=None, **kwargs):
        self.clear()
        self.update(options, **kwargs)

    def update(self, options=None, **kwargs):
        if options is not None:
            if not isinstance(options, ChartOptions):
                raise AttributeError, \
                      'attribute options of type %s should be %s' % \
                      (type(options), ChartOptions)
            self.options.update(options.options)

        for key,value in kwargs.iteritems():
            if key not in ChartOptions.defaults:
                raise AttributeError, \
                      "%s instance has no attribute '%s'" % (type(self), key)
            self.options[key] = value

    def __getattr__(self, attr):
        if attr in self.options:
            return self.options[attr]

        if attr in ChartOptions.defaults:
            return ChartOptions.defaults[attr]

        raise AttributeError, \
              "%s instance has no attribute '%s'" % (type(self), attr)

    def __setattr__(self, attr, value):
        if attr in ChartOptions.defaults:
            self.options[attr] = value
        else:
            super(ChartOptions, self).__setattr__(attr, value)

