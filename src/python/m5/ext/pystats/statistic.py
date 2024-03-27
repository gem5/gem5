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
    unit: Optional[str]
    description: Optional[str]
    datatype: Optional[StorageType]

    def __init__(
        self,
        value: Any,
        type: Optional[str] = None,
        unit: Optional[str] = None,
        description: Optional[str] = None,
        datatype: Optional[StorageType] = None,
    ):
        self.value = value
        self.type = type
        self.unit = unit
        self.description = description
        self.datatype = datatype

    def __repr__(self):
        return str(self.value)


class Scalar(Statistic):
    """
    A scalar Python statistic type.
    """

    value: Union[float, int]

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
            unit=unit,
            description=description,
            datatype=datatype,
        )


class BaseScalarVector(Statistic):
    """
    An abstract base class for classes containing a vector of Scalar values.
    """

    value: List[Union[int, float]]

    def __init__(
        self,
        value: Iterable[Union[int, float]],
        type: Optional[str] = None,
        unit: Optional[str] = None,
        description: Optional[str] = None,
        datatype: Optional[StorageType] = None,
    ):
        super().__init__(
            value=list(value),
            type=type,
            unit=unit,
            description=description,
            datatype=datatype,
        )

    def mean(self) -> float:
        """
        Returns the mean of the value vector.

        :returns: The mean value across all bins.
        """
        assert self.value != None
        assert isinstance(self.value, List)

        from statistics import mean as statistics_mean

        return statistics_mean(self.value)

    def count(self) -> float:
        """
        Returns the count across all the bins.

        :returns: The sum of all bin values.
        """
        assert self.value != None
        return sum(self.value)


class Distribution(BaseScalarVector):
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
        value: Iterable[int],
        min: Union[float, int],
        max: Union[float, int],
        num_bins: int,
        bin_size: Union[float, int],
        sum: Optional[int] = None,
        sum_squared: Optional[int] = None,
        underflow: Optional[int] = None,
        overflow: Optional[int] = None,
        logs: Optional[float] = None,
        unit: Optional[str] = None,
        description: Optional[str] = None,
        datatype: Optional[StorageType] = None,
    ):
        super().__init__(
            value=value,
            type="Distribution",
            unit=unit,
            description=description,
            datatype=datatype,
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


class Accumulator(BaseScalarVector):
    """
    A statistical type representing an accumulator.
    """

    _count: int
    min: Union[int, float]
    max: Union[int, float]
    sum_squared: Optional[int]

    def __init__(
        self,
        value: Iterable[Union[int, float]],
        count: int,
        min: Union[int, float],
        max: Union[int, float],
        sum_squared: Optional[int] = None,
        unit: Optional[str] = None,
        description: Optional[str] = None,
        datatype: Optional[StorageType] = None,
    ):
        super().__init__(
            value=value,
            type="Accumulator",
            unit=unit,
            description=description,
            datatype=datatype,
        )

        self._count = count
        self.min = min
        self.max = max
        self.sum_squared = sum_squared

    def count(self) -> int:
        return self._count
