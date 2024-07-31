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

__all__ = ["multidict"]


from typing import Self


class multidict:
    def __init__(self, parent: Self | None = None, **kwargs):
        self.local = dict(**kwargs)
        self.parent: Self | None = parent
        self.deleted = {}

    def __str__(self):
        return str(dict(self.items()))

    def __repr__(self):
        return repr(dict(list(self.items())))

    def __contains__(self, key):
        return key in self.local or (
            self.parent is not None and key in self.parent
        )

    def __delitem__(self, key):
        try:
            del self.local[key]
        except KeyError as e:
            if self.parent is not None and key in self.parent:
                self.deleted[key] = True
            else:
                raise KeyError(e)

    def __setitem__(self, key, value):
        self.deleted.pop(key, False)
        self.local[key] = value

    def __getitem__(self, key):
        try:
            return self.local[key]
        except KeyError as e:
            if (
                not self.deleted.get(key, False)
                and self.parent is not None
                and key in self.parent
            ):
                return self.parent[key]
            else:
                raise KeyError(e)

    def __len__(self):
        if self.parent is not None:
            return len(self.local) + len(self.parent)
        else:
            return len(self.local)

    def next(self):
        for key, value in self.local.items():
            yield key, value

        if self.parent:
            for key, value in self.parent.next():
                if key not in self.local and key not in self.deleted:
                    yield key, value

    def has_key(self, key):
        return key in self

    def items(self):
        yield from self.next()

    def keys(self):
        for key, _ in self.next():
            yield key

    def values(self):
        for _, value in self.next():
            yield value

    def get(self, key, default=None):
        try:
            return self[key]
        except KeyError as e:
            return default

    def setdefault(self, key, default):
        try:
            return self[key]
        except KeyError:
            self.deleted.pop(key, False)
            self.local[key] = default
            return default

    def _dump(self):
        print("multidict dump")
        node = self
        while isinstance(node, multidict):
            print("    ", node.local)
            node = node.parent

    def _dumpkey(self, key):
        values = []
        node = self
        while isinstance(node, multidict):
            if key in node.local:
                values.append(node.local[key])
            node = node.parent
        print(key, values)


if __name__ == "__main__":
    test1 = multidict()
    test2 = multidict(test1)
    test3 = multidict(test2)
    test4 = multidict(test3)

    test1["a"] = "test1_a"
    test1["b"] = "test1_b"
    test1["c"] = "test1_c"
    test1["d"] = "test1_d"
    test1["e"] = "test1_e"

    test2["a"] = "test2_a"
    del test2["b"]
    test2["c"] = "test2_c"
    del test1["a"]

    test2.setdefault("f", multidict)

    print("test1>", list(test1.items()))
    print("test2>", list(test2.items()))
    # print(test1['a'])
    print(test1["b"])
    print(test1["c"])
    print(test1["d"])
    print(test1["e"])

    print(test2["a"])
    # print(test2['b'])
    print(test2["c"])
    print(test2["d"])
    print(test2["e"])

    for key in test2.keys():
        print(key)

    test2.get("g", "foo")
    # test2.get('b')
    test2.get("b", "bar")
    test2.setdefault("b", "blah")
    print(test1)
    print(test2)
    print(repr(test2))

    print(len(test2))

    test3["a"] = [0, 1, 2, 3]

    print(test4)
