# Copyright (c) 2006 The Regents of The University of Michigan
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

__all__ = [ 'attrdict', 'multiattrdict', 'optiondict' ]

class attrdict(dict):
    """Wrap dict, so you can use attribute access to get/set elements"""
    def __getattr__(self, attr):
        if attr in self:
            return self.__getitem__(attr)
        return super().__getattribute__(attr)

    def __setattr__(self, attr, value):
        if attr in dir(self) or attr.startswith('_'):
            return super().__setattr__(attr, value)
        return self.__setitem__(attr, value)

    def __delattr__(self, attr):
        if attr in self:
            return self.__delitem__(attr)
        return super().__delattr__(attr)

    def __getstate__(self):
        return dict(self)

    def __setstate__(self, state):
        self.update(state)

class multiattrdict(attrdict):
    """Wrap attrdict so that nested attribute accesses automatically create
    nested dictionaries."""
    def __getattr__(self, attr):
        try:
            return super().__getattr__(attr)
        except AttributeError:
            if attr.startswith('_'):
                raise

            d = multiattrdict()
            setattr(self, attr, d)
            return d

class optiondict(attrdict):
    """Modify attrdict so that a missing attribute just returns None"""
    def __getattr__(self, attr):
        try:
            return super().__getattr__(attr)
        except AttributeError:
            return None

if __name__ == '__main__':
    x = attrdict()
    x.y = 1
    x['z'] = 2
    print(x['y'], x.y)
    print(x['z'], x.z)
    print(dir(x))
    print(x)

    print()

    del x['y']
    del x.z
    print(dir(x))
    print(x)

    print()
    print("multiattrdict")
    x = multiattrdict()
    x.x.x.x = 9
    x.y.z = 9
    print(x)
    print(x.y)
    print(x.y.z)
    print(x.z.z)
