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

__all__ = [ 'orderdict' ]

class orderdict(dict):
    def __init__(self, d = {}):
        self._keys = d.keys()
        super(orderdict, self).__init__(d)

    def __setitem__(self, key, item):
        super(orderdict, self).__setitem__(key, item)
        if not hasattr(self, '_keys'):
            self._keys = [key,]
        if key not in self._keys:
            self._keys.append(key)

    def __delitem__(self, key):
        super(orderdict, self).__delitem__(key)
        self._keys.remove(key)

    def clear(self):
        super(orderdict, self).clear()
        self._keys = []

    def items(self):
        for i in self._keys:
            yield i, self[i]

    def keys(self):
        return self._keys

    def popitem(self):
        if len(self._keys) == 0:
            raise KeyError('dictionary is empty')
        else:
            key = self._keys[-1]
            val = self[key]
            del self[key]
            return key, val

    def setdefault(self, key, failobj = None):
        super(orderdict, self).setdefault(key, failobj)
        if key not in self._keys:
            self._keys.append(key)

    def update(self, d):
        for key in d.keys():
            if not self.has_key(key):
                self._keys.append(key)
        super(orderdict, self).update(d)

    def values(self):
        for i in self._keys:
            yield self[i]
