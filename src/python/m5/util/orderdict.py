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

__all__ = [ 'orderdict' ]

from UserDict import DictMixin

class orderdict(dict, DictMixin):
    def __init__(self, *args, **kwargs):
        if len(args) > 1:
            raise TypeError("expected at most one argument, got %d" % \
                            len(args))
        self._keys = []
        self.update(*args, **kwargs)

    def __setitem__(self, key, item):
        if key not in self:
            self._keys.append(key)
        super(orderdict, self).__setitem__(key, item)

    def __delitem__(self, key):
        super(orderdict, self).__delitem__(key)
        self._keys.remove(key)

    def clear(self):
        super(orderdict, self).clear()
        self._keys = []

    def iterkeys(self):
        for key in self._keys:
            yield key

    def itervalues(self):
        for key in self._keys:
            yield self[key]

    def iteritems(self):
        for key in self._keys:
            yield key, self[key]

    def keys(self):
        return self._keys[:]

    def values(self):
        return [ self[key] for key in self._keys ]

    def items(self):
        return [ (self[key],key) for key in self._keys ]
