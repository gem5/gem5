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

from __future__ import division
import operator, re, types

class ProxyError(Exception):
    pass

def unproxy(proxy):
    if hasattr(proxy, '__unproxy__'):
        return proxy.__unproxy__()

    return proxy

def scalar(stat):
    stat = unproxy(stat)
    assert(stat.__scalar__() != stat.__vector__())
    return stat.__scalar__()

def vector(stat):
    stat = unproxy(stat)
    assert(stat.__scalar__() != stat.__vector__())
    return stat.__vector__()

def value(stat, *args):
    stat = unproxy(stat)
    return stat.__value__(*args)

def values(stat, run):
    stat = unproxy(stat)
    result = []
    for i in xrange(len(stat)):
        val = value(stat, run, i)
        if val is None:
            return None
        result.append(val)
    return result

def total(stat, run):
    return sum(values(stat, run))

def len(stat):
    stat = unproxy(stat)
    return stat.__len__()

class Value(object):
    def __scalar__(self):
        raise AttributeError, "must define __scalar__ for %s" % (type (self))
    def __vector__(self):
        raise AttributeError, "must define __vector__ for %s" % (type (self))

    def __add__(self, other):
        return BinaryProxy(operator.__add__, self, other)
    def __sub__(self, other):
        return BinaryProxy(operator.__sub__, self, other)
    def __mul__(self, other):
        return BinaryProxy(operator.__mul__, self, other)
    def __div__(self, other):
        return BinaryProxy(operator.__div__, self, other)
    def __truediv__(self, other):
        return BinaryProxy(operator.__truediv__, self, other)
    def __floordiv__(self, other):
        return BinaryProxy(operator.__floordiv__, self, other)

    def __radd__(self, other):
        return BinaryProxy(operator.__add__, other, self)
    def __rsub__(self, other):
        return BinaryProxy(operator.__sub__, other, self)
    def __rmul__(self, other):
        return BinaryProxy(operator.__mul__, other, self)
    def __rdiv__(self, other):
        return BinaryProxy(operator.__div__, other, self)
    def __rtruediv__(self, other):
        return BinaryProxy(operator.__truediv__, other, self)
    def __rfloordiv__(self, other):
        return BinaryProxy(operator.__floordiv__, other, self)

    def __neg__(self):
        return UnaryProxy(operator.__neg__, self)
    def __pos__(self):
        return UnaryProxy(operator.__pos__, self)
    def __abs__(self):
        return UnaryProxy(operator.__abs__, self)

class Scalar(Value):
    def __scalar__(self):
        return True

    def __vector__(self):
        return False

    def __value__(self, run):
        raise AttributeError, '__value__ must be defined'

class VectorItemProxy(Value):
    def __init__(self, proxy, index):
        self.proxy = proxy
        self.index = index

    def __scalar__(self):
        return True

    def __vector__(self):
        return False

    def __value__(self, run):
        return value(self.proxy, run, self.index)

class Vector(Value):
    def __scalar__(self):
        return False

    def __vector__(self):
        return True

    def __value__(self, run, index):
        raise AttributeError, '__value__ must be defined'

    def __getitem__(self, index):
        return VectorItemProxy(self, index)

class ScalarConstant(Scalar):
    def __init__(self, constant):
        self.constant = constant
    def __value__(self, run):
        return self.constant
    def __str__(self):
        return str(self.constant)

class VectorConstant(Vector):
    def __init__(self, constant):
        self.constant = constant
    def __value__(self, run, index):
        return self.constant[index]
    def __len__(self):
        return len(self.constant)
    def __str__(self):
        return str(self.constant)

def WrapValue(value):
    if isinstance(value, (int, long, float)):
        return ScalarConstant(value)
    if isinstance(value, (list, tuple)):
        return VectorConstant(value)
    if isinstance(value, Value):
        return value

    raise AttributeError, 'Only values can be wrapped'

class Statistic(object):
    def __getattr__(self, attr):
        if attr in ('data', 'x', 'y'):
            result = self.source.data(self, self.ticks)
            self.data = result.data
            self.x = result.x
            self.y = result.y
        return super(Statistic, self).__getattribute__(attr)

    def __setattr__(self, attr, value):
        if attr == 'stat':
            raise AttributeError, '%s is read only' % stat
        if attr in ('source', 'ticks'):
            if getattr(self, attr) != value:
                if hasattr(self, 'data'):
                    delattr(self, 'data')

        super(Statistic, self).__setattr__(attr, value)

    def __str__(self):
        return self.name

class ValueProxy(Value):
    def __getattr__(self, attr):
        if attr == '__value__':
            if scalar(self):
                return self.__scalarvalue__
            if vector(self):
                return self.__vectorvalue__
        if attr == '__len__':
            if vector(self):
                return self.__vectorlen__
        return super(ValueProxy, self).__getattribute__(attr)

class UnaryProxy(ValueProxy):
    def __init__(self, op, arg):
        self.op = op
        self.arg = WrapValue(arg)

    def __scalar__(self):
        return scalar(self.arg)

    def __vector__(self):
        return vector(self.arg)

    def __scalarvalue__(self, run):
        val = value(self.arg, run)
        if val is None:
            return None
        return self.op(val)

    def __vectorvalue__(self, run, index):
        val = value(self.arg, run, index)
        if val is None:
            return None
        return self.op(val)

    def __vectorlen__(self):
        return len(unproxy(self.arg))

    def __str__(self):
        if self.op == operator.__neg__:
            return '-%s' % str(self.arg)
        if self.op == operator.__pos__:
            return '+%s' % str(self.arg)
        if self.op == operator.__abs__:
            return 'abs(%s)' % self.arg

class BinaryProxy(ValueProxy):
    def __init__(self, op, arg0, arg1):
        super(BinaryProxy, self).__init__()
        self.op = op
        self.arg0 = WrapValue(arg0)
        self.arg1 = WrapValue(arg1)

    def __scalar__(self):
        return scalar(self.arg0) and scalar(self.arg1)

    def __vector__(self):
        return vector(self.arg0) or vector(self.arg1)

    def __scalarvalue__(self, run):
        val0 = value(self.arg0, run)
        val1 = value(self.arg1, run)
        if val0 is None or val1 is None:
            return None
        try:
            return self.op(val0, val1)
        except ZeroDivisionError:
            return None

    def __vectorvalue__(self, run, index):
        if scalar(self.arg0):
            val0 = value(self.arg0, run)
        if vector(self.arg0):
            val0 = value(self.arg0, run, index)
        if scalar(self.arg1):
            val1 = value(self.arg1, run)
        if vector(self.arg1):
            val1 = value(self.arg1, run, index)

        if val0 is None or val1 is None:
            return None

        try:
            return self.op(val0, val1)
        except ZeroDivisionError:
            return None

    def __vectorlen__(self):
        if vector(self.arg0) and scalar(self.arg1):
            return len(self.arg0)
        if scalar(self.arg0) and vector(self.arg1):
            return len(self.arg1)

        len0 = len(self.arg0)
        len1 = len(self.arg1)

        if len0 != len1:
            raise AttributeError, \
                  "vectors of different lengths %d != %d" % (len0, len1)

        return len0

    def __str__(self):
        ops = { operator.__add__ : '+',
                operator.__sub__ : '-',
                operator.__mul__ : '*',
                operator.__div__ : '/',
                operator.__truediv__ : '/',
                operator.__floordiv__ : '//' }

        return '(%s %s %s)' % (str(self.arg0), ops[self.op], str(self.arg1))

class Proxy(Value):
    def __init__(self, name, dict):
        self.name = name
        self.dict = dict

    def __unproxy__(self):
        return unproxy(self.dict[self.name])

    def __getitem__(self, index):
        return ItemProxy(self, index)

    def __getattr__(self, attr):
        return AttrProxy(self, attr)

    def __str__(self):
        return str(self.dict[self.name])

class ItemProxy(Proxy):
    def __init__(self, proxy, index):
        self.proxy = proxy
        self.index = index

    def __unproxy__(self):
        return unproxy(unproxy(self.proxy)[self.index])

    def __str__(self):
        return '%s[%s]' % (self.proxy, self.index)

class AttrProxy(Proxy):
    def __init__(self, proxy, attr):
        self.proxy = proxy
        self.attr = attr

    def __unproxy__(self):
        proxy = unproxy(self.proxy)
        try:
            attr = getattr(proxy, self.attr)
        except AttributeError, e:
            raise ProxyError, e
        return unproxy(attr)

    def __str__(self):
        return '%s.%s' % (self.proxy, self.attr)

class ProxyGroup(object):
    def __init__(self, dict=None, **kwargs):
        self.__dict__['dict'] = {}

        if dict is not None:
            self.dict.update(dict)

        if kwargs:
            self.dict.update(kwargs)

    def __getattr__(self, name):
        return Proxy(name, self.dict)

    def __setattr__(self, attr, value):
        self.dict[attr] = value

class ScalarStat(Statistic,Scalar):
    def __value__(self, run):
        if run not in self.data:
            return None
        return self.data[run][0][0]

    def display(self, run=None):
        import display
        p = display.Print()
        p.name = self.name
        p.desc = self.desc
        p.value = value(self, run)
        p.flags = self.flags
        p.precision = self.precision
        if display.all or (self.flags & flags.printable):
            p.display()

class VectorStat(Statistic,Vector):
    def __value__(self, run, item):
        if run not in self.data:
            return None
        return self.data[run][item][0]

    def __len__(self):
        return self.x

    def display(self, run=None):
        import display
        d = display.VectorDisplay()
        d.name = self.name
        d.desc = self.desc
        d.value = [ value(self, run, i) for i in xrange(len(self)) ]
        d.flags = self.flags
        d.precision = self.precision
        d.display()

class Formula(Value):
    def __getattribute__(self, attr):
        if attr not in ( '__scalar__', '__vector__', '__value__', '__len__' ):
            return super(Formula, self).__getattribute__(attr)

        formula = re.sub(':', '__', self.formula)
        value = eval(formula, self.source.stattop)
        return getattr(value, attr)

    def __str__(self):
        return self.name

class SimpleDist(Statistic):
    def __init__(self, sums, squares, samples):
        self.sums = sums
        self.squares = squares
        self.samples = samples

    def display(self, name, desc, flags, precision):
        import display
        p = display.Print()
        p.flags = flags
        p.precision = precision

        if self.samples > 0:
            p.name = name + ".mean"
            p.value = self.sums / self.samples
            p.display()

            p.name = name + ".stdev"
            if self.samples > 1:
                var = (self.samples * self.squares - self.sums ** 2) \
                      / (self.samples * (self.samples - 1))
                if var >= 0:
                    p.value = math.sqrt(var)
                else:
                    p.value = 'NaN'
            else:
                p.value = 0.0
            p.display()

        p.name = name + ".samples"
        p.value = self.samples
        p.display()

    def comparable(self, other):
        return True

    def __eq__(self, other):
        return self.sums == other.sums and self.squares == other.squares and \
               self.samples == other.samples

    def __isub__(self, other):
        self.sums -= other.sums
        self.squares -= other.squares
        self.samples -= other.samples
        return self

    def __iadd__(self, other):
        self.sums += other.sums
        self.squares += other.squares
        self.samples += other.samples
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        self.sums /= other
        self.squares /= other
        self.samples /= other
        return self

class FullDist(SimpleDist):
    def __init__(self, sums, squares, samples, minval, maxval,
                 under, vec, over, min, max, bsize, size):
        self.sums = sums
        self.squares = squares
        self.samples = samples
        self.minval = minval
        self.maxval = maxval
        self.under = under
        self.vec = vec
        self.over = over
        self.min = min
        self.max = max
        self.bsize = bsize
        self.size = size

    def display(self, name, desc, flags, precision):
        import display
        p = display.Print()
        p.flags = flags
        p.precision = precision

        p.name = name + '.min_val'
        p.value = self.minval
        p.display()

        p.name = name + '.max_val'
        p.value = self.maxval
        p.display()

        p.name = name + '.underflow'
        p.value = self.under
        p.display()

        i = self.min
        for val in self.vec[:-1]:
            p.name = name + '[%d:%d]' % (i, i + self.bsize - 1)
            p.value = val
            p.display()
            i += self.bsize

        p.name = name + '[%d:%d]' % (i, self.max)
        p.value = self.vec[-1]
        p.display()


        p.name = name + '.overflow'
        p.value = self.over
        p.display()

        SimpleDist.display(self, name, desc, flags, precision)

    def comparable(self, other):
        return self.min == other.min and self.max == other.max and \
               self.bsize == other.bsize and self.size == other.size

    def __eq__(self, other):
        return self.sums == other.sums and self.squares == other.squares and \
               self.samples == other.samples

    def __isub__(self, other):
        self.sums -= other.sums
        self.squares -= other.squares
        self.samples -= other.samples

        if other.samples:
            self.minval = min(self.minval, other.minval)
            self.maxval = max(self.maxval, other.maxval)
            self.under -= under
            self.vec = map(lambda x,y: x - y, self.vec, other.vec)
            self.over -= over
        return self

    def __iadd__(self, other):
        if not self.samples and other.samples:
            self = other
            return self

        self.sums += other.sums
        self.squares += other.squares
        self.samples += other.samples

        if other.samples:
            self.minval = min(self.minval, other.minval)
            self.maxval = max(self.maxval, other.maxval)
            self.under += other.under
            self.vec = map(lambda x,y: x + y, self.vec, other.vec)
            self.over += other.over
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        self.sums /= other
        self.squares /= other
        self.samples /= other

        if self.samples:
            self.under /= other
            for i in xrange(len(self.vec)):
                self.vec[i] /= other
            self.over /= other
        return self

class Dist(Statistic):
    def display(self):
        import display
        if not display.all and not (self.flags & flags.printable):
            return

        self.dist.display(self.name, self.desc, self.flags, self.precision)

    def comparable(self, other):
        return self.name == other.name and \
               self.dist.compareable(other.dist)

    def __eq__(self, other):
        return self.dist == other.dist

    def __isub__(self, other):
        self.dist -= other.dist
        return self

    def __iadd__(self, other):
        self.dist += other.dist
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        self.dist /= other
        return self

class VectorDist(Statistic):
    def display(self):
        import display
        if not display.all and not (self.flags & flags.printable):
            return

        if isinstance(self.dist, SimpleDist):
            return

        for dist,sn,sd,i in map(None, self.dist, self.subnames, self.subdescs,
                                range(len(self.dist))):
            if len(sn) > 0:
                name = '%s.%s' % (self.name, sn)
            else:
                name = '%s[%d]' % (self.name, i)

            if len(sd) > 0:
                desc = sd
            else:
                desc = self.desc

            dist.display(name, desc, self.flags, self.precision)

        if (self.flags & flags.total) or 1:
            if isinstance(self.dist[0], SimpleDist):
                disttotal = SimpleDist( \
                    reduce(sums, [d.sums for d in self.dist]),
                    reduce(sums, [d.squares for d in self.dist]),
                    reduce(sums, [d.samples for d in self.dist]))
            else:
                disttotal = FullDist( \
                    reduce(sums, [d.sums for d in self.dist]),
                    reduce(sums, [d.squares for d in self.dist]),
                    reduce(sums, [d.samples for d in self.dist]),
                    min([d.minval for d in self.dist]),
                    max([d.maxval for d in self.dist]),
                    reduce(sums, [d.under for d in self.dist]),
                    reduce(sums, [d.vec for d in self.dist]),
                    reduce(sums, [d.over for d in self.dist]),
                    dist[0].min,
                    dist[0].max,
                    dist[0].bsize,
                    dist[0].size)

            name = '%s.total' % (self.name)
            desc = self.desc
            disttotal.display(name, desc, self.flags, self.precision)

    def comparable(self, other):
        return self.name == other.name and \
               alltrue(map(lambda x, y : x.comparable(y),
                           self.dist,
                           other.dist))

    def __eq__(self, other):
        return alltrue(map(lambda x, y : x == y, self.dist, other.dist))

    def __isub__(self, other):
        if isinstance(self.dist, (list, tuple)) and \
               isinstance(other.dist, (list, tuple)):
            for sd,od in zip(self.dist, other.dist):
                sd -= od
        else:
            self.dist -= other.dist
        return self

    def __iadd__(self, other):
        if isinstance(self.dist, (list, tuple)) and \
               isinstance(other.dist, (list, tuple)):
            for sd,od in zip(self.dist, other.dist):
                sd += od
        else:
            self.dist += other.dist
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        if isinstance(self.dist, (list, tuple)):
            for dist in self.dist:
                dist /= other
        else:
            self.dist /= other
        return self

class Vector2d(Statistic):
    def display(self):
        import display
        if not display.all and not (self.flags & flags.printable):
            return

        d = display.VectorDisplay()
        d.__dict__.update(self.__dict__)

        if self.__dict__.has_key('ysubnames'):
            ysubnames = list(self.ysubnames)
            slack = self.x - len(ysubnames)
            if slack > 0:
                ysubnames.extend(['']*slack)
        else:
            ysubnames = range(self.x)

        for x,sname in enumerate(ysubnames):
            o = x * self.y
            d.value = self.value[o:o+self.y]
            d.name = '%s[%s]' % (self.name, sname)
            d.display()

        if self.flags & flags.total:
            d.value = []
            for y in range(self.y):
                xtot = 0.0
                for x in range(self.x):
                    xtot += self.value[y + x * self.x]
                d.value.append(xtot)

            d.name = self.name + '.total'
            d.display()

    def comparable(self, other):
        return self.name == other.name and self.x == other.x and \
               self.y == other.y

    def __eq__(self, other):
        return True

    def __isub__(self, other):
        return self

    def __iadd__(self, other):
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        return self

def NewStat(source, data):
    stat = None
    if data.type == 'SCALAR':
        stat = ScalarStat()
    elif data.type == 'VECTOR':
        stat = VectorStat()
    elif data.type == 'DIST':
        stat = Dist()
    elif data.type == 'VECTORDIST':
        stat = VectorDist()
    elif data.type == 'VECTOR2D':
        stat = Vector2d()
    elif data.type == 'FORMULA':
        stat = Formula()

    stat.__dict__['source'] = source
    stat.__dict__['ticks'] = None
    stat.__dict__.update(data.__dict__)

    return stat

