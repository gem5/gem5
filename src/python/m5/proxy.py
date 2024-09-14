# Copyright (c) 2018 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#####################################################################
#
# Proxy object support.
#
#####################################################################

import copy


class BaseProxy:
    def __init__(self, search_self, search_up):
        self._search_self = search_self
        self._search_up = search_up
        self._ops = []

    def __str__(self):
        if self._search_self and not self._search_up:
            s = "Self"
        elif not self._search_self and self._search_up:
            s = "Parent"
        else:
            s = "ConfusedProxy"
        return s + "." + self.path()

    def __setattr__(self, attr, value):
        if not attr.startswith("_"):
            raise AttributeError(
                f"cannot set attribute '{attr}' on proxy object"
            )
        super().__setattr__(attr, value)

    @staticmethod
    def _gen_op(operation):
        def op(self, operand):
            if not (isinstance(operand, (int, float)) or isproxy(operand)):
                raise TypeError(
                    "Proxy operand must be a constant or a proxy to a param"
                )
            self._ops.append((operation, operand))
            return self

        return op

    # Support for multiplying proxies by either constants or other proxies
    __mul__ = _gen_op(lambda operand_a, operand_b: operand_a * operand_b)
    __rmul__ = __mul__

    # Support for dividing proxies by either constants or other proxies
    __truediv__ = _gen_op(lambda operand_a, operand_b: operand_a / operand_b)
    __floordiv__ = _gen_op(lambda operand_a, operand_b: operand_a // operand_b)

    # Support for dividing constants by proxies
    __rtruediv__ = _gen_op(
        lambda operand_a, operand_b: operand_b / operand_a.getValue()
    )
    __rfloordiv__ = _gen_op(
        lambda operand_a, operand_b: operand_b // operand_a.getValue()
    )

    # After all the operators and operands have been defined, this function
    # should be called to perform the actual operation
    def _opcheck(self, result, base):
        from . import params

        for operation, operand in self._ops:
            # Get the operand's value
            if isproxy(operand):
                operand = operand.unproxy(base)
                # assert that we are operating with a compatible param
                if not isinstance(operand, params.NumericParamValue):
                    raise TypeError("Proxy operand must be a numerical param")
                operand = operand.getValue()

            # Apply the operation
            result = operation(result, operand)

        return result

    def unproxy(self, base):
        obj = base
        done = False

        if self._search_self:
            result, done = self.find(obj)

        if self._search_up:
            # Search up the tree but mark ourself
            # as visited to avoid a self-reference
            self._visited = True
            obj._visited = True
            while not done:
                obj = obj._parent
                if not obj:
                    break
                result, done = self.find(obj)

            self._visited = False
            base._visited = False

        if not done:
            raise AttributeError(
                "Can't resolve proxy '%s' of type '%s' from '%s'"
                % (self.path(), self._pdesc.ptype_str, base.path())
            )

        if isinstance(result, BaseProxy):
            if result == self:
                raise RuntimeError("Cycle in unproxy")
            result = result.unproxy(obj)

        return self._opcheck(result, base)

    @staticmethod
    def getindex(obj, index):
        if index is None:
            return obj
        try:
            obj = obj[index]
        except TypeError:
            if index != 0:
                raise
            # if index is 0 and item is not subscriptable, just
            # use item itself (so cpu[0] works on uniprocessors)
        return obj

    # This method should be called once the proxy is assigned to a
    # particular parameter or port to set the expected type of the
    # resolved proxy
    def set_param_desc(self, pdesc):
        self._pdesc = pdesc


class AttrProxy(BaseProxy):
    def __init__(self, search_self, search_up, attr):
        super().__init__(search_self, search_up)
        self._attr = attr
        self._modifiers = []

    def __getattr__(self, attr):
        # python uses __bases__ internally for inheritance
        if attr.startswith("_"):
            return super().__getattr__(self, attr)
        if hasattr(self, "_pdesc"):
            raise AttributeError(
                "Attribute reference on bound proxy " f"({self}.{attr})"
            )
        # Return a copy of self rather than modifying self in place
        # since self could be an indirect reference via a variable or
        # parameter
        new_self = copy.deepcopy(self)
        new_self._modifiers.append(attr)
        return new_self

    # support indexing on proxies (e.g., Self.cpu[0])
    def __getitem__(self, key):
        if not isinstance(key, int):
            raise TypeError("Proxy object requires integer index")
        if hasattr(self, "_pdesc"):
            raise AttributeError("Index operation on bound proxy")
        new_self = copy.deepcopy(self)
        new_self._modifiers.append(key)
        return new_self

    def find(self, obj):
        try:
            val = getattr(obj, self._attr)
            visited = False
            if hasattr(val, "_visited"):
                visited = getattr(val, "_visited")

            if visited:
                return None, False

            if not isproxy(val):
                # for any additional unproxying to be done, pass the
                # current, rather than the original object so that proxy
                # has the right context
                obj = val

        except:
            return None, False
        while isproxy(val):
            val = val.unproxy(obj)
        for m in self._modifiers:
            if isinstance(m, str):
                val = getattr(val, m)
            elif isinstance(m, int):
                val = val[m]
            else:
                assert "Item must be string or integer"
            while isproxy(val):
                val = val.unproxy(obj)
        return val, True

    def path(self):
        p = self._attr
        for m in self._modifiers:
            if isinstance(m, str):
                p += f".{m}"
            elif isinstance(m, int):
                p += "[%d]" % m
            else:
                assert "Item must be string or integer"
        return p


class AnyProxy(BaseProxy):
    def find(self, obj):
        return obj.find_any(self._pdesc.ptype)

    def path(self):
        return "any"


# The AllProxy traverses the entire sub-tree (not only the children)
# and adds all objects of a specific type
class AllProxy(BaseProxy):
    def find(self, obj):
        return obj.find_all(self._pdesc.ptype)

    def path(self):
        return "all"


def isproxy(obj):
    from . import params

    if isinstance(obj, (BaseProxy, params.EthernetAddr)):
        return True
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            if isproxy(v):
                return True
    return False


class ProxyFactory:
    def __init__(self, search_self, search_up):
        self.search_self = search_self
        self.search_up = search_up

    def __getattr__(self, attr):
        if attr == "any":
            return AnyProxy(self.search_self, self.search_up)
        elif attr == "all":
            if self.search_up:
                assert "Parant.all is not supported"
            return AllProxy(self.search_self, self.search_up)
        else:
            return AttrProxy(self.search_self, self.search_up, attr)


# global objects for handling proxies
Parent = ProxyFactory(search_self=False, search_up=True)
Self = ProxyFactory(search_self=True, search_up=False)

# limit exports on 'from proxy import *'
__all__ = ["Parent", "Self"]
