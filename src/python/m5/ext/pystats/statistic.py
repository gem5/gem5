# Copyright (c) 2021 The Regents of The University of California
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

from abc import ABC
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    List,
    Optional,
    Union,
)

from .abstract_stat import AbstractStat
from .storagetype import StorageType


class Statistic(ABC, AbstractStat):
    """
    The abstract base class for all Python statistics.
    """

    value: Any
    type: Optional[str]
    description: Optional[str]

    def __init__(
        self,
        value: Any,
        type: Optional[str] = None,
        description: Optional[str] = None,
    ):
        self.value = value
        self.type = type
        self.description = description

    def __repr__(self):
        return str(self.value)


class Scalar(Statistic):
    """
    A scalar Python statistic type.
    """

    value: Union[float, int]
    unit: Optional[str]
    datatype: Optional[StorageType]

    def __init__(
        self,
        value: Union[float, int],
        unit: Optional[str] = None,
        description: Optional[str] = None,
        datatype: Optional[StorageType] = None,
    ):
        super().__init__(
            value=value,
            type="Scalar",
            description=description,
        )
        self.unit = unit
        self.datatype = datatype


class Vector(Statistic):
    """
    An Python statistics which representing a vector of Scalar values.
    """

    def __init__(
        self,
        value: Dict[Union[str, int, float], Scalar],
        type: Optional[str] = None,
        description: Optional[str] = None,
    ):
        super().__init__(
            value=value,
            type=type,
            description=description,
        )

    def __getitem__(self, item: Union[int, str, float]) -> Scalar:
        assert self.value != None
        # In the case of string, we cast strings to integers of floats if they
        # are numeric. This avoids users having to cast strings to integers.
        if isinstance(item, str):
            if item.isdigit():
                item = int(item)
            elif item.isnumeric():
                item = float(item)
        return self.value[item]

    def __contains__(self, item) -> bool:
        assert self.value != None
        if isinstance(item, str):
            if item.isdigit():
                item = int(item)
            elif item.isnumeric():
                item = float(item)
        return item in self.value

    def __iner__(self) -> None:
        return iter(self.value)

    def __len__(self) -> int:
        assert self.value != None
        return len(self.value.values())

    def size(self) -> int:
        """
        Returns the size of the vector.

        :returns: The size of the vector.
        """
        assert self.value != None
        return len(self.value)

    def mean(self) -> float:
        """
        Returns the mean of the value vector.

        :returns: The mean value across all values in the vector.
        """
        assert self.value != None

        return self.count() / self.size()

    def count(self) -> float:
        """
        Returns the count (sum) of all values in the vector.

        :returns: The sum of all vector values.
        """
        assert self.value != None
        return sum(float(self.value[key]) for key in self.values)

    def children(
        self,
        predicate: Optional[Callable[[str], bool]] = None,
        recursive: bool = False,
    ) -> List["AbstractStat"]:
        to_return = []
        for attr in self.value.keys():
            obj = self.value[attr]
            if isinstance(obj, AbstractStat):
                if (
                    isinstance(attr, str)
                    and (predicate and predicate(attr))
                    or not predicate
                ):
                    to_return.append(obj)
                to_return = to_return + obj.children(
                    predicate=predicate, recursive=True
                )
        return to_return


class Vector2d(Statistic):
    """
    A 2D vector of scalar values.
    """

    value: Dict[Union[str, int, float], Vector]

    def __init__(
        self,
        value: Dict[Union[str, int, float], Vector],
        type: Optional[str] = None,
        description: Optional[str] = None,
    ):
        assert (
            len({vector.size() for vector in value.values()}) == 1
        ), "All the Vectors in the 2d Vector are not of equal length."

        super().__init__(
            value=value,
            type=type,
            description=description,
        )

    def x_size(self) -> int:
        """Returns the number of elements in the x dimension."""
        assert self.value is not None
        return len(self.value)

    def y_size(self) -> int:
        """Returns the number of elements in the y dimension."""
        assert self.value is not None
        return len(self.value[0])

    def size(self) -> int:
        """Returns the total number of elements."""
        return self.x_size() * self.y_size()

    def __len__(self) -> int:
        return self.x_size()

    def __iter__(self):
        return iter(self.keys())

    def total(self) -> int:
        """The total (sum) of all the entries in the 2d vector/"""
        assert self.value is not None
        total = 0
        for vector in self.value.values():
            for scalar in vector.values():
                total += scalar.value
        return total

    def __getitem__(self, index: Union[str, int, float]) -> Vector:
        assert self.value is not None
        # In the case of string, we cast strings to integers of floats if they
        # are numeric. This avoids users having to cast strings to integers.
        if isinstance(index, str):
            if index.isindex():
                index = int(index)
            elif index.isnumeric():
                index = float(index)
        return self.value[index]

    def children(
        self,
        predicate: Optional[Callable[[str], bool]] = None,
        recursive: bool = False,
    ) -> List["AbstractStat"]:
        to_return = []
        for attr in self.value.keys():
            obj = self.value[attr]
            if (
                isinstance(attr, str)
                and (predicate and predicate(attr))
                or not predicate
            ):
                to_return.append(obj)
            to_return = to_return + obj.children(
                predicate=predicate, recursive=True
            )
        return to_return

    def __contains__(self, item) -> bool:
        assert self.value is not None
        if isinstance(item, str):
            if item.isdigit():
                item = int(item)
            elif item.isnumeric():
                item = float(item)
        return item in self.value


class Distribution(Vector):
    """
    A statistic type that stores information relating to distributions. Each
    distribution has a number of bins (>=1)
    between this range. The values correspond to the value of each bin.
    E.g., ``value[3]`` is the value of the 4th bin.

    It is assumed each bucket is of equal size.
    """

    min: Union[float, int]
    max: Union[float, int]
    num_bins: int
    bin_size: Union[float, int]
    sum: Optional[int]
    sum_squared: Optional[int]
    underflow: Optional[int]
    overflow: Optional[int]
    logs: Optional[float]

    def __init__(
        self,
        value: Dict[Union[int, float], Scalar],
        min: Union[float, int],
        max: Union[float, int],
        num_bins: int,
        bin_size: Union[float, int],
        sum: Optional[int] = None,
        sum_squared: Optional[int] = None,
        underflow: Optional[int] = None,
        overflow: Optional[int] = None,
        logs: Optional[float] = None,
        description: Optional[str] = None,
    ):
        super().__init__(
            value=value,
            type="Distribution",
            description=description,
        )

        self.min = min
        self.max = max
        self.num_bins = num_bins
        self.bin_size = bin_size
        self.sum = sum
        self.underflow = underflow
        self.overflow = overflow
        self.logs = logs
        self.sum_squared = sum_squared

        # These check some basic conditions of a distribution.
        assert self.bin_size >= 0
        assert self.num_bins >= 1


class SparseHist(Vector):
    """A Sparse Histogram of values. A sparse histogram simply counts the "
    frequency of each value in a sample. Ergo, it is, ineffect an disctionary
    of values mapped to their count"""

    def __init__(
        self,
        value: Dict[float, Scalar],
        description: Optional[str] = None,
    ):
        super().__init__(
            value=value,
            type="SparseHist",
            description=description,
        )

    def size(self) -> int:
        """The number of unique sampled values."""
        return len(self.value)

    def count(self) -> int:
        """
        Returns the total number of samples.
        """
        assert self.value != None
        return sum(self.value.values())
