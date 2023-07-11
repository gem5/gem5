# Copyright (c) 2006 Nathan Binkert <nate@binkert.org>
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


class _neg_inf(object):
    """This object always compares less than any other object"""

    def __repr__(self):
        return "<neg_inf>"

    def __lt__(self, other):
        return type(self) != type(other)

    def __le__(self, other):
        return True

    def __gt__(self, other):
        return False

    def __ge__(self, other):
        return type(self) == type(other)

    def __eq__(self, other):
        return type(self) == type(other)

    def __ne__(self, other):
        return type(self) != type(other)


neg_inf = _neg_inf()


class _pos_inf(object):
    """This object always compares greater than any other object"""

    def __repr__(self):
        return "<pos_inf>"

    def __lt__(self, other):
        return False

    def __le__(self, other):
        return type(self) == type(other)

    def __gt__(self, other):
        return type(self) != type(other)

    def __ge__(self, other):
        return True

    def __eq__(self, other):
        return type(self) == type(other)

    def __ne__(self, other):
        return type(self) != type(other)


pos_inf = _pos_inf()


class Region(tuple):
    """A region (range) of [start, end).
    This includes utility functions to compare overlap of regions."""

    def __new__(cls, *args):
        if len(args) == 1:
            arg = args[0]
            if isinstance(arg, Region):
                return arg
            args = tuple(arg)

        if len(args) != 2:
            raise AttributeError(
                "Only one or two arguments allowed, %d provided" % (alen,)
            )

        return tuple.__new__(cls, args)

    def __repr__(self):
        return f"Region({self[0]}, {self[1]})"

    @property
    def start(self):
        return self[0]

    @property
    def end(self):
        return self[1]

    def __contains__(self, other):
        """other is
        region: True if self and other is fully contained within self.
        pos: True if other is within the region"""
        if isinstance(other, tuple):
            return self[0] <= other[0] and self[1] >= other[1]
        return self[0] <= other and other < self[1]

    def __eq__(self, other):
        """other is
        region: True if self and other are identical.
        pos: True if other is within the region"""
        if isinstance(other, tuple):
            return self[0] == other[0] and self[1] == other[1]
        return self[0] <= other and other < self[1]

    # @param self is a region.
    # @param other is a region.
    # @return if self and other are not identical.
    def __ne__(self, other):
        """other is
        region: true if they are not identical
        pos: True if other is not in the region"""
        if isinstance(other, tuple):
            return self[0] != other[0] or self[1] != other[1]
        return other < self[0] or self[1] <= other

    # @param self is a region.
    # @param other is a region.
    # @return if self is less than other and does not overlap self.
    def __lt__(self, other):
        "self completely left of other (cannot overlap)"
        if isinstance(other, tuple):
            return self[1] <= other[0]
        return self[1] <= other

    # @param self is a region.
    # @param other is a region.
    # @return if self is less than other.  self may overlap other,
    # but not extend beyond the _end of other.
    def __le__(self, other):
        "self extends to the left of other (can overlap)"
        if isinstance(other, tuple):
            return self[0] <= other[0]
        return self[0] <= other

    # @param self is a region.
    # @param other is a region.
    # @return if self is greater than other and does not overlap other.
    def __gt__(self, other):
        "self is completely right of other (cannot overlap)"
        if isinstance(other, tuple):
            return self[0] >= other[1]
        return self[0] > other

    # @param self is a region.
    # @param other is a region.
    # @return if self is greater than other.  self may overlap other,
    # but not extend beyond the beginning of other.
    def __ge__(self, other):
        "self ex_ends beyond other to the right (can overlap)"
        if isinstance(other, tuple):
            return self[1] >= other[1]
        return self[1] > other


class Regions(object):
    """A set of regions (ranges).  Basically a region with holes.
    Includes utility functions to merge regions and figure out if
    something is in one of the regions."""

    def __init__(self, *args):
        self.regions = []
        self.extend(*args)

    def copy(self):
        copy = Regions()
        copy.regions.extend(self.regions)
        return copy

    def append(self, *args):
        self.regions.append(Region(*args))

    def extend(self, *args):
        self.regions.extend(Region(a) for a in args)

    def __contains__(self, position):
        for region in self.regions:
            if position in region:
                return True

        return False

    def __len__(self):
        return len(self.regions)

    def __iand__(self, other):
        A = self.regions
        B = other.regions
        R = []

        i = 0
        j = 0
        while i < len(self) and j < len(other):
            a = A[i]
            b = B[j]
            if a[1] <= b[0]:
                # A is completely before B.  Skip A
                i += 1
            elif a[0] <= b[0]:
                if a[1] <= b[1]:
                    # A and B overlap with B not left of A and A not right of B
                    R.append(Region(b[0], a[1]))

                    # Advance A because nothing is left
                    i += 1

                    if a[1] == b[1]:
                        # Advance B too
                        j += 1
                else:
                    # A and B overlap with B completely within the bounds of A
                    R.append(Region(b[0], b[1]))

                    # Advance only B because some of A may still be useful
                    j += 1
            elif b[1] <= a[0]:
                # B is completely before A. Skip B.
                j += 1
            else:
                assert b[0] < a[0]
                if b[1] <= a[1]:
                    # A and B overlap with A not left of B and B not right of A
                    R.append(Region(a[0], b[1]))

                    # Advance B because nothing is left
                    j += 1

                    if a[1] == b[1]:
                        # Advance A too
                        i += 1
                else:
                    # A and B overlap with A completely within the bounds of B
                    R.append(Region(a[0], a[1]))

                    # Advance only A because some of B may still be useful
                    i += 1

        self.regions = R
        return self

    def __and__(self, other):
        result = self.copy()
        result &= other
        return result

    def __repr__(self):
        return f"Regions({[(r[0], r[1]) for r in self.regions]})"


all_regions = Regions(Region(neg_inf, pos_inf))

if __name__ == "__main__":
    x = Regions(*((i, i + 1) for i in range(0, 30, 2)))
    y = Regions(*((i, i + 4) for i in range(0, 30, 5)))
    z = Region(6, 7)
    n = Region(9, 10)

    def test(left, right):
        print(f"{left} == {right}: {left == right}")
        print(f"{left} != {right}: {left != right}")
        print(f"{left} <  {right}: {left < right}")
        print(f"{left} <= {right}: {left <= right}")
        print(f"{left} >  {right}: {left > right}")
        print(f"{left} >= {right}: {left >= right}")
        print("\n")

    test(neg_inf, neg_inf)
    test(neg_inf, pos_inf)
    test(pos_inf, neg_inf)
    test(pos_inf, pos_inf)

    test(neg_inf, 0)
    test(neg_inf, -11111)
    test(neg_inf, 11111)

    test(0, neg_inf)
    test(-11111, neg_inf)
    test(11111, neg_inf)

    test(pos_inf, 0)
    test(pos_inf, -11111)
    test(pos_inf, 11111)

    test(0, pos_inf)
    test(-11111, pos_inf)
    test(11111, pos_inf)

    print(x)
    print(y)
    print(x & y)
    print(z)

    print(4 in x)
    print(4 in z)
    print(5 not in x)
    print(6 not in z)
    print(z in y)
    print(n in y, n not in y)
