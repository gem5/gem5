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

# The SmartDict class fixes a couple of issues with using the content
# of os.environ or similar dicts of strings as Python variables:
#
# 1) Undefined variables should return False rather than raising KeyError.
#
# 2) String values of 'False', '0', etc., should evaluate to False
#    (not just the empty string).
#
# #1 is solved by overriding __getitem__, and #2 is solved by using a
# proxy class for values and overriding __nonzero__ on the proxy.
# Everything else is just to (a) make proxies behave like normal
# values otherwise, (b) make sure any dict operation returns a proxy
# rather than a normal value, and (c) coerce values written to the
# dict to be strings.

from __future__ import print_function
from __future__ import absolute_import
import six
if six.PY3:
    long = int

from .convert import *
from .attrdict import attrdict

class Variable(str):
    """Intelligent proxy class for SmartDict.  Variable will use the
    various convert functions to attempt to convert values to useable
    types"""
    def __int__(self):
        return toInteger(str(self))
    def __long__(self):
        return toLong(str(self))
    def __float__(self):
        return toFloat(str(self))
    def __bool__(self):
        return toBool(str(self))
    # Python 2.7 uses __nonzero__ instead of __bool__
    __nonzero__ = __bool__
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

class UndefinedVariable(object):
    """Placeholder class to represent undefined variables.  Will
    generally cause an exception whenever it is used, but evaluates to
    zero for boolean truth testing such as in an if statement"""
    def __bool__(self):
        return False

    # Python 2.7 uses __nonzero__ instead of __bool__
    __nonzero__ = __bool__

class SmartDict(attrdict):
    """Dictionary class that holds strings, but intelligently converts
    those strings to other types depending on their usage"""

    def __getitem__(self, key):
        """returns a Variable proxy if the values exists in the database and
        returns an UndefinedVariable otherwise"""

        if key in self:
            return Variable(dict.get(self, key))
        else:
            # Note that this does *not* change the contents of the dict,
            # so that even after we call env['foo'] we still get a
            # meaningful answer from "'foo' in env" (which
            # calls dict.__contains__, which we do not override).
            return UndefinedVariable()

    def __setitem__(self, key, item):
        """intercept the setting of any variable so that we always
        store strings in the dict"""
        dict.__setitem__(self, key, str(item))

    def values(self):
        for value in dict.values(self):
            yield Variable(value)

    def items(self):
        for key,value in dict.items(self):
            yield key, Variable(value)

    def get(self, key, default='False'):
        return Variable(dict.get(self, key, str(default)))

    def setdefault(self, key, default='False'):
        return Variable(dict.setdefault(self, key, str(default)))

__all__ = [ 'SmartDict' ]
