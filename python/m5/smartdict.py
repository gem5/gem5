from convert import *

class SmartDict(dict):
    class Proxy(str):
        def __int__(self):
            return int(to_integer(str(self)))
        def __long__(self):
            return long(to_integer(str(self)))
        def __float__(self):
            return float(to_integer(str(self)))
        def __nonzero__(self):
            return to_bool(str(self))
        def convert(self, other):
            t = type(other)
            if t == bool:
                return bool(self)
            if t == int:
                return int(self)
            if t == long:
                return long(self)
            if t == float:
                return float(self)
            return str(self)
        def __lt__(self, other):
            return self.convert(other) < other
        def __le__(self, other):
            return self.convert(other) <= other
        def __eq__(self, other):
            return self.convert(other) == other
        def __ne__(self, other):
            return self.convert(other) != other
        def __gt__(self, other):
            return self.convert(other) > other
        def __ge__(self, other):
            return self.convert(other) >= other

        def __add__(self, other):
            return self.convert(other) + other
        def __sub__(self, other):
            return self.convert(other) - other
        def __mul__(self, other):
            return self.convert(other) * other
        def __div__(self, other):
            return self.convert(other) / other
        def __truediv__(self, other):
            return self.convert(other) / other

        def __radd__(self, other):
            return other + self.convert(other)
        def __rsub__(self, other):
            return other - self.convert(other)
        def __rmul__(self, other):
            return other * self.convert(other)
        def __rdiv__(self, other):
            return other / self.convert(other)
        def __rtruediv__(self, other):
            return other / self.convert(other)


    def __getitem__(self, key):
        return self.Proxy(dict.__getitem__(self, key))

    def __setitem__(self, key, item):
        dict.__setitem__(self, key, str(item))

    def values(self):
        return [ self.Proxy(v) for v in dict.values(self) ]

    def itervalues(self):
        for value in dict.itervalues(self):
            yield self.Proxy(value)

    def items(self):
        return [ (k, self.Proxy(v)) for k,v in dict.items(self) ]

    def iteritems(self):
        for key,value in dict.iteritems(self):
            yield key, self.Proxy(value)

    def get(self, key, default=''):
        return self.Proxy(dict.get(self, key, str(default)))

    def setdefault(self, key, default=''):
        return self.Proxy(dict.setdefault(self, key, str(default)))

