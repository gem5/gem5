from __future__ import division
import operator, re, types

source = None
display_run = 0
global globalTicks
globalTicks = None

def issequence(t):
    return isinstance(t, types.TupleType) or isinstance(t, types.ListType)

def total(f):
    if isinstance(f, FormulaStat):
        v = f.value
    else:
        v = f

    f = FormulaStat()
    if issequence(v):
        f.value = reduce(operator.add, v)
    else:
        f.value = v

    return f

def unaryop(op, f):
    if isinstance(f, FormulaStat):
        v = f.value
    else:
        v = f

    if issequence(v):
        return map(op, v)
    else:
        return op(v)

def zerodiv(lv, rv):
    if rv == 0.0:
        return 0.0
    else:
        return operator.truediv(lv, rv)

def wrapop(op, lv, rv):
    if isinstance(lv, str):
        return lv

    if isinstance(rv, str):
        return rv

    return op(lv, rv)

def same(lrun, rrun):
    for lx,rx in zip(lrun.keys(),rrun.keys()):
        if lx != rx:
            print 'lx != rx'
            print lx, rx
            print lrun.keys()
            print rrun.keys()
            return False
        for ly,ry in zip(lrun[lx].keys(),rrun[rx].keys()):
            if ly != ry:
                print 'ly != ry'
                print ly, ry
                print lrun[lx].keys()
                print rrun[rx].keys()
                return False
    return True


def binaryop(op, lf, rf):
    result = {}

    if isinstance(lf, FormulaStat) and isinstance(rf, FormulaStat):
        lv = lf.value
        rv = rf.value

        theruns = []
        for r in lv.keys():
            if rv.has_key(r):
                if same(lv[r], rv[r]):
                    theruns.append(r)
                else:
                    raise AttributeError

        for run in theruns:
            result[run] = {}
            for x in lv[run].keys():
                result[run][x] = {}
                for y in lv[run][x].keys():
                    result[run][x][y] = wrapop(op, lv[run][x][y],
                                               rv[run][x][y])
    elif isinstance(lf, FormulaStat):
        lv = lf.value
        for run in lv.keys():
            result[run] = {}
            for x in lv[run].keys():
                result[run][x] = {}
                for y in lv[run][x].keys():
                    result[run][x][y] = wrapop(op, lv[run][x][y], rf)
    elif isinstance(rf, FormulaStat):
        rv = rf.value
        for run in rv.keys():
            result[run] = {}
            for x in rv[run].keys():
                result[run][x] = {}
                for y in rv[run][x].keys():
                    result[run][x][y] = wrapop(op, lf, rv[run][x][y])

    return result

def sums(x, y):
    if issequence(x):
        return map(lambda x, y: x + y, x, y)
    else:
        return x + y

def alltrue(list):
    return reduce(lambda x, y: x and y, list)

def allfalse(list):
    return not reduce(lambda x, y: x or y, list)

def enumerate(list):
    return map(None, range(len(list)), list)

def cmp(a, b):
    if a < b:
        return -1
    elif a == b:
        return 0
    else:
        return 1

class Statistic(object):

    def __init__(self, data):
        self.__dict__.update(data.__dict__)
        if not self.__dict__.has_key('value'):
            self.__dict__['value'] = None
        if not self.__dict__.has_key('bins'):
            self.__dict__['bins'] = None
        if not self.__dict__.has_key('ticks'):
            self.__dict__['ticks'] = None
        if 'vc' not in self.__dict__:
            self.vc = {}

    def __getattribute__(self, attr):
        if attr == 'ticks':
            if self.__dict__['ticks'] != globalTicks:
                self.__dict__['value'] = None
                self.__dict__['ticks'] = globalTicks
            return self.__dict__['ticks']
        if attr == 'value':
            if self.__dict__['ticks'] != globalTicks:
                if self.__dict__['ticks'] != None and \
                                    len(self.__dict__['ticks']) == 1:
                    self.vc[self.__dict__['ticks'][0]] = self.__dict__['value']
                self.__dict__['ticks'] = globalTicks
                if len(globalTicks) == 1 and self.vc.has_key(globalTicks[0]):
                    self.__dict__['value'] = self.vc[globalTicks[0]]
                else:
                    self.__dict__['value'] = None
            if self.__dict__['value'] == None:
                self.__dict__['value'] = self.getValue()
            return self.__dict__['value']
        else:
            return super(Statistic, self).__getattribute__(attr)

    def __setattr__(self, attr, value):
        if attr == 'bins' or attr == 'ticks':
            if attr == 'bins':
                if value is not None:
                    value = source.getBin(value)
            #elif attr == 'ticks' and type(value) is str:
            #    value = [ int(x) for x in value.split() ]

            self.__dict__[attr] = value
            self.__dict__['value'] = None
            self.vc = {}
        else:
            super(Statistic, self).__setattr__(attr, value)

    def getValue(self):
        raise AttributeError, 'getValue() must be defined'

    def zero(self):
        return False

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return '%f' % (float(self))

class FormulaStat(object):
    def __add__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.add, self, other)
        return f
    def __sub__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.sub, self, other)
        return f
    def __mul__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.mul, self, other)
        return f
    def __truediv__(self, other):
        f = FormulaStat()
        f.value = binaryop(zerodiv, self, other)
        return f
    def __mod__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.mod, self, other)
        return f
    def __radd__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.add, other, self)
        return f
    def __rsub__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.sub, other, self)
        return f
    def __rmul__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.mul, other, self)
        return f
    def __rtruediv__(self, other):
        f = FormulaStat()
        f.value = binaryop(zerodiv, other, self)
        return f
    def __rmod__(self, other):
        f = FormulaStat()
        f.value = binaryop(operator.mod, other, self)
        return f
    def __neg__(self):
        f = FormulaStat()
        f.value = unaryop(operator.neg, self)
        return f
    def __getitem__(self, idx):
        f = FormulaStat()
        f.value = {}
        for key in self.value.keys():
            f.value[key] = {}
            f.value[key][0] = {}
            f.value[key][0][0] = self.value[key][idx][0]
        return f

    def __float__(self):
        if isinstance(self.value, FormulaStat):
            return float(self.value)
        if not self.value.has_key(display_run):
            return (1e300*1e300)
        if len(self.value[display_run]) == 1:
            return self.value[display_run][0][0]
        else:
            #print self.value[display_run]
            return self.value[display_run][4][0]
            #raise ValueError

    def display(self):
        import display
        d = display.VectorDisplay()
        d.flags = 0
        d.precision = 1
        d.name = 'formula'
        d.desc = 'formula'
        val = self.value[display_run]
        d.value = [ val[x][0] for x in val.keys() ]
        d.display()


class Scalar(Statistic,FormulaStat):
    def getValue(self):
        return source.data(self, self.bins, self.ticks)

    def display(self):
        import display
        p = display.Print()
        p.name = self.name
        p.desc = self.desc
        p.value = float(self)
        p.flags = self.flags
        p.precision = self.precision
        if display.all or (self.flags & flags.printable):
            p.display()

    def comparable(self, other):
        return self.name == other.name

    def __eq__(self, other):
        return self.value == other.value

    def __isub__(self, other):
        self.value -= other.value
        return self

    def __iadd__(self, other):
        self.value += other.value
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        self.value /= other
        return self

class Vector(Statistic,FormulaStat):
    def getValue(self):
        return source.data(self, self.bins, self.ticks);

    def display(self):
        import display
        if not display.all and not (self.flags & flags.printable):
            return

        d = display.VectorDisplay()
        d.__dict__.update(self.__dict__)
        d.display()

    def comparable(self, other):
        return self.name == other.name and \
               len(self.value) == len(other.value)

    def __eq__(self, other):
        if issequence(self.value) != issequence(other.value):
            return False

        if issequence(self.value):
            if len(self.value) != len(other.value):
                return False
            else:
                for v1,v2 in zip(self.value, other.value):
                    if v1 != v2:
                        return False
                return True
        else:
            return self.value == other.value

    def __isub__(self, other):
        self.value = binaryop(operator.sub, self.value, other.value)
        return self

    def __iadd__(self, other):
        self.value = binaryop(operator.add, self.value, other.value)
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        if issequence(self.value):
            for i in xrange(len(self.value)):
                self.value[i] /= other
        else:
            self.value /= other
        return self

class Formula(Vector):
    def getValue(self):
        formula = re.sub(':', '__', self.formula)
        x = eval(formula, source.stattop)
        return x.value

    def comparable(self, other):
        return self.name == other.name and \
               compare(self.dist, other.dist)

    def __eq__(self, other):
        return self.value == other.value

    def __isub__(self, other):
        return self

    def __iadd__(self, other):
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        return self

class SimpleDist(object):
    def __init__(self, sums, squares, samples):
        self.sums = sums
        self.squares = squares
        self.samples = samples

    def getValue(self):
        return 0.0

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

    def getValue(self):
        return 0.0

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
    def getValue(self):
        return 0.0

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
    def getValue(self):
        return 0.0

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
        if issequence(self.dist) and issequence(other.dist):
            for sd,od in zip(self.dist, other.dist):
                sd -= od
        else:
            self.dist -= other.dist
        return self

    def __iadd__(self, other):
        if issequence(self.dist) and issequence(other.dist):
            for sd,od in zip(self.dist, other.dist):
                sd += od
        else:
            self.dist += other.dist
        return self

    def __itruediv__(self, other):
        if not other:
            return self
        if issequence(self.dist):
            for dist in self.dist:
                dist /= other
        else:
            self.dist /= other
        return self

class Vector2d(Statistic):
    def getValue(self):
        return 0.0

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

def NewStat(data):
    stat = None
    if data.type == 'SCALAR':
        stat = Scalar(data)
    elif data.type == 'VECTOR':
        stat = Vector(data)
    elif data.type == 'DIST':
        stat = Dist(data)
    elif data.type == 'VECTORDIST':
        stat = VectorDist(data)
    elif data.type == 'VECTOR2D':
        stat = Vector2d(data)
    elif data.type == 'FORMULA':
        stat = Formula(data)

    return stat

