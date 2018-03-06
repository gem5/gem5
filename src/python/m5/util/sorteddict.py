# Copyright (c) 2006-2009 Nathan Binkert <nate@binkert.org>
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

from __future__ import print_function

from bisect import bisect_left, bisect_right

class SortedDict(dict):
    def _get_sorted(self):
        return getattr(self, '_sorted', sorted)
    def _set_sorted(self, val):
        self._sorted = val
        self._del_keys()
    sorted = property(_get_sorted, _set_sorted)

    @property
    def _keys(self):
        try:
            return self._sorted_keys
        except AttributeError:
            _sorted_keys = self.sorted(dict.iterkeys(self))
            self._sorted_keys = _sorted_keys
            return _sorted_keys

    def _left_eq(self, key):
        index = self._left_ge(self, key)
        if self._keys[index] != key:
            raise KeyError(key)
        return index

    def _right_eq(self, key):
        index = self._right_le(self, key)
        if self._keys[index] != key:
            raise KeyError(key)
        return index

    def _right_lt(self, key):
        index = bisect_left(self._keys, key)
        if index:
            return index - 1
        raise KeyError(key)

    def _right_le(self, key):
        index = bisect_right(self._keys, key)
        if index:
            return index - 1
        raise KeyError(key)

    def _left_gt(self, key):
        index = bisect_right(self._keys, key)
        if index != len(self._keys):
            return index
        raise KeyError(key)

    def _left_ge(self, key):
        index = bisect_left(self._keys, key)
        if index != len(self._keys):
            return index
        raise KeyError(key)

    def _del_keys(self):
        try:
            del self._sorted_keys
        except AttributeError:
            pass

    def __repr__(self):
        return 'SortedDict({%s})' % ', '.join('%r: %r' % item
                                              for item in self.iteritems())
    def __setitem__(self, key, item):
        dict.__setitem__(self, key, item)
        self._del_keys()

    def __delitem__(self, key):
        dict.__delitem__(self, key)
        self._del_keys()

    def clear(self):
        self.data.clear()
        self._del_keys()

    def copy(self):
        t = type(self)
        return t(self)

    def keys(self):
        return self._keys[:]

    def values(self):
        return list(self.itervalues())

    def items(self):
        return list(self.iteritems())

    def iterkeys(self):
        return iter(self._keys)

    def itervalues(self):
        for k in self._keys:
            yield self[k]

    def iteritems(self):
        for k in self._keys:
            yield k, self[k]

    def keyrange(self, start=None, end=None, inclusive=False):
        if start is not None:
            start = self._left_ge(start)

        if end is not None:
            if inclusive:
                end = self._right_le(end)
            else:
                end = self._right_lt(end)

        return iter(self._keys[start:end+1])

    def valuerange(self, *args, **kwargs):
        for k in self.keyrange(*args, **kwargs):
            yield self[k]

    def itemrange(self, *args, **kwargs):
        for k in self.keyrange(*args, **kwargs):
            yield k, self[k]

    def update(self, *args, **kwargs):
        dict.update(self, *args, **kwargs)
        self._del_keys()

    def setdefault(self, key, _failobj=None):
        try:
            return self[key]
        except KeyError:
            self[key] = _failobj

    def pop(self, key, *args):
        try:
            dict.pop(self, key)
            self._del_keys()
        except KeyError:
            if not args:
                raise
            return args[0]

    def popitem(self):
        try:
            key = self._keys[0]
            self._del_keys()
        except IndexError:
            raise KeyError('popitem(): dictionary is empty')
        else:
            return key, dict.pop(self, key)

    @classmethod
    def fromkeys(cls, seq, value=None):
        d = cls()
        for key in seq:
            d[key] = value
        return d

if __name__ == '__main__':
    def display(d):
        print(d)
        print(d.keys())
        print(list(d.iterkeys()))
        print(d.values())
        print(list(d.itervalues()))
        print(d.items())
        print(list(d.iteritems()))

    d = SortedDict(x=24,e=5,j=4,b=2,z=26,d=4)
    display(d)

    print('popitem', d.popitem())
    display(d)

    print('pop j')
    d.pop('j')
    display(d)

    d.setdefault('a', 1)
    d.setdefault('g', 7)
    d.setdefault('_')
    display(d)

    d.update({'b' : 2, 'h' : 8})
    display(d)

    del d['x']
    display(d)
    d['y'] = 26
    display(d)

    print(`d`)

    print(d.copy())

    for k,v in d.itemrange('d', 'z', inclusive=True):
        print(k, v)
