# Copyright (c) 2003-2004 The Regents of The University of Michigan
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

all = False
descriptions = False

class Value:
    def __init__(self, value, precision, percent = False):
        self.value = value
        self.precision = precision
        self.percent = percent
    def __str__(self):
        if isinstance(self.value, str):
            if self.value.lower() == 'nan':
                value = 'NaN'
            if self.value.lower() == 'inf':
                value = 'Inf'
        else:
            if self.precision >= 0:
                format = "%%.%df" % self.precision
            elif self.value == 0.0:
                format = "%.0f"
            elif self.value % 1.0 == 0.0:
                format = "%.0f"
            else:
                format = "%f"
            value = self.value
            if self.percent:
                value = value * 100.0
            value = format % value

        if self.percent:
            value = value + "%"

        return value

class Print:
    def __init__(self, **vals):
        self.__dict__.update(vals)

    def __str__(self):
        value = Value(self.value, self.precision)
        pdf = ''
        cdf = ''
        if self.__dict__.has_key('pdf'):
            pdf = Value(self.pdf, 2, True)
        if self.__dict__.has_key('cdf'):
            cdf = Value(self.cdf, 2, True)

        output = "%-40s %12s %8s %8s" % (self.name, value, pdf, cdf)

        if descriptions and self.__dict__.has_key('desc') and self.desc:
            output = "%s # %s" % (output, self.desc)

        return output

    def doprint(self):
        if display_all:
            return True
        if self.value == 0.0 and (self.flags & flags_nozero):
            return False
        if isinstance(self.value, str):
            if self.value == 'NaN' and (self.flags & flags_nonan):
                return False
        return True

    def display(self):
        if self.doprint():
            print self

class VectorDisplay:
    def display(self):
        p = Print()
        p.flags = self.flags
        p.precision = self.precision

        if isinstance(self.value, (list, tuple)):
            if not len(self.value):
                return

            mytotal = reduce(lambda x,y: float(x) + float(y), self.value)
            mycdf = 0.0

            value = self.value

            if display_all:
                subnames = [ '[%d]' % i for i in range(len(value)) ]
            else:
                subnames = [''] * len(value)

            if self.__dict__.has_key('subnames'):
                for i,each in enumerate(self.subnames):
                    if len(each) > 0:
                        subnames[i] = '.%s' % each

            subdescs = [self.desc]*len(value)
            if self.__dict__.has_key('subdescs'):
                for i in xrange(min(len(value), len(self.subdescs))):
                    subdescs[i] = self.subdescs[i]

            for val,sname,sdesc in map(None, value, subnames, subdescs):
                if mytotal > 0.0:
                    mypdf = float(val) / float(mytotal)
                    mycdf += mypdf
                    if (self.flags & flags_pdf):
                        p.pdf = mypdf
                        p.cdf = mycdf

                if len(sname) == 0:
                    continue

                p.name = self.name + sname
                p.desc = sdesc
                p.value = val
                p.display()

            if (self.flags & flags_total):
                if (p.__dict__.has_key('pdf')): del p.__dict__['pdf']
                if (p.__dict__.has_key('cdf')): del p.__dict__['cdf']
                p.name = self.name + '.total'
                p.desc = self.desc
                p.value = mytotal
                p.display()

        else:
            p.name = self.name
            p.desc = self.desc
            p.value = self.value
            p.display()

