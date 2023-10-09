# Copyright (c) 2021 The Regents of the University of California
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
from enum import Enum
from typing import Any
from typing import Iterator

from m5.objects import Port
from m5.objects import PyTrafficGen
from m5.ticks import fromSeconds
from m5.util.convert import toLatency
from m5.util.convert import toMemoryBandwidth

from ...utils.override import overrides
from .abstract_core import AbstractCore
from .abstract_generator_core import AbstractGeneratorCore


class TrafficModes(Enum):
    """The traffic mode class
    This class is an enum to store traffic mode in a more meaningful way
    """

    linear = 0
    random = 1


class ComplexTrafficParams:
    def __init__(
        self,
        mode: TrafficModes,
        duration: str,
        rate: str,
        block_size: int,
        min_addr: int,
        max_addr: int,
        rd_perc: int,
        data_limit: int,
    ):
        """The complex traffic params class
        This class is a container for parameters to create either a linear or
        random traffic. The complex generator core stores a list of complex
        traffic params for resolution after m5.instantiate is called.
        """
        self._mode = mode
        self._duration = duration
        self._rate = rate
        self._block_size = block_size
        self._min_addr = min_addr
        self._max_addr = max_addr
        self._rd_perc = rd_perc
        self._data_limit = data_limit


class ComplexGeneratorCore(AbstractGeneratorCore):
    def __init__(self):
        """The complex generator core interface.

        This class defines the interface for a generator core that will create
        a series of different types of traffic. This core uses PyTrafficGen to
        create and inject the synthetic traffic. This generator could be used
        to create more complex traffics that consist of linear and random
        traffic in different phases.
        """
        super().__init__()
        self.generator = PyTrafficGen()
        self._traffic_params = []
        self._traffic = []
        self._traffic_set = False

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.generator.port = port

    def add_linear(
        self,
        duration: str,
        rate: str,
        block_size: int,
        min_addr: int,
        max_addr: int,
        rd_perc: int,
        data_limit: int,
    ) -> None:
        """
        This function will add the params for a linear traffic to the list of
        traffic params in this generator core. These params will be later
        resolved by the start_traffic call. This core uses a PyTrafficGen to
        create the traffic based on the specified params below.

        :param duration: The number of ticks for the generator core to generate
        traffic.
        :param rate: The rate at which the synthetic data is read/written.
        :param block_size: The number of bytes to be read/written with each
        request.
        :param min_addr: The lower bound of the address range the generator
        will read/write from/to.
        :param max_addr: The upper bound of the address range the generator
        will read/write from/to.
        :param rd_perc: The percentage of read requests among all the generated
        requests. The write percentage would be equal to 100 - rd_perc.
        :param data_limit: The amount of data in bytes to read/write by the
        generator before stopping generation.
        """
        param = ComplexTrafficParams(
            TrafficModes.linear,
            duration,
            rate,
            block_size,
            min_addr,
            max_addr,
            rd_perc,
            data_limit,
        )
        self._traffic_params = self._traffic_params + [param]
        self._traffic_set = False

    def add_random(
        self,
        duration: str,
        rate: str,
        block_size: int,
        min_addr: int,
        max_addr: int,
        rd_perc: int,
        data_limit: int,
    ) -> None:
        """
        This function will add the params for a random traffic to the list of
        traffic params in this generator core. These params will be later
        resolved by the start_traffic call. This core uses a PyTrafficGen to
        create the traffic based on the specified params below.

        :param duration: The number of ticks for the generator core to generate
        traffic.
        :param rate: The rate at which the synthetic data is read/written.
        :param block_size: The number of bytes to be read/written with each
        request.
        :param min_addr: The lower bound of the address range the generator
        will read/write from/to.
        :param max_addr: The upper bound of the address range the generator
        will read/write from/to.
        :param rd_perc: The percentage of read requests among all the generated
        requests. The write percentage would be equal to 100 - rd_perc.
        :param data_limit: The amount of data in bytes to read/write by the
        generator before stopping generation.
        """
        param = ComplexTrafficParams(
            TrafficModes.random,
            duration,
            rate,
            block_size,
            min_addr,
            max_addr,
            rd_perc,
            data_limit,
        )
        self._traffic_params = self._traffic_params + [param]
        self._traffic_set = False

    @overrides(AbstractGeneratorCore)
    def start_traffic(self) -> None:
        """
        This function first checks if there are any pending traffics that
        require creation, if so it will create the pending traffics based on
        traffic_params list and adds  them to the traffic list, then it starts
        generating the traffic at the top of the traffic list. It also pops the
        first element in the list so that every time this function is called a
        new traffic is generated, each instance of a call to this function
        should happen before each instance of the call to m5.simulate(). All
        the instances of calls to this function should happen after
        m5.instantiate()
        """
        if not self._traffic_set:
            self._set_traffic()
        if self._traffic:
            self.generator.start(self._traffic.pop(0))
        else:
            print("No phases left to generate!")

    def _set_traffic(self) -> None:
        """
        This function will pop params from traffic params list and create their
        respective traffic and adds the traffic to the core's traffic list.
        """
        while self._traffic_params:
            param = self._traffic_params.pop(0)
            mode = param._mode
            duration = param._duration
            rate = param._rate
            block_size = param._block_size
            min_addr = param._min_addr
            max_addr = param._max_addr
            rd_perc = param._rd_perc
            data_limit = param._data_limit

            if mode == TrafficModes.linear:
                traffic = self._create_linear_traffic(
                    duration,
                    rate,
                    block_size,
                    min_addr,
                    max_addr,
                    rd_perc,
                    data_limit,
                )
                self._traffic = self._traffic + [traffic]

            if mode == TrafficModes.random:
                traffic = self._create_random_traffic(
                    duration,
                    rate,
                    block_size,
                    min_addr,
                    max_addr,
                    rd_perc,
                    data_limit,
                )
                self._traffic = self._traffic + [traffic]

        self._traffic_set = True

    def set_traffic_from_python_generator(
        self, python_generator: Iterator[Any]
    ) -> None:
        """
        Function to set the traffic from a user defined python generator.
        The generator should only only assume one input argument (positional)
        for the actual PyTrafficGen object to create the traffic. This is possible
        either through using a generator with hardcoded parameters in the
        function calls to PyTrafficGen methods or by compiling a flexible
        python generator into a generator object with only one
        input argument (positional) using functools.partial.

        :param generator: A python generator object that creates traffic
        patterns through calls to methods of PyTrafficGen.
        """
        if not self._traffic_set:
            self._set_traffic()
        self._traffic.append(python_generator(self.generator))

    def _create_linear_traffic(
        self,
        duration: str,
        rate: str,
        block_size: int,
        min_addr: int,
        max_addr: int,
        rd_perc: int,
        data_limit: int,
    ) -> None:
        """
        This function yields (creates) a linear traffic based on the input
        params. Then it will yield (create) an exit traffic (exit traffic is
        used to exit the simulation).

        :param duration: The number of ticks for the generator core to generate
        traffic.
        :param rate: The rate at which the synthetic data is read/written.
        :param block_size: The number of bytes to be read/written with each
        request.
        :param min_addr: The lower bound of the address range the generator
        will read/write from/to.
        :param max_addr: The upper bound of the address range the generator
        will read/write from/to.
        :param rd_perc: The percentage of read requests among all the generated
        requests. The write percentage would be equal to 100 - rd_perc.
        :param data_limit: The amount of data in bytes to read/write by the
        generator before stopping generation.
        """
        duration = fromSeconds(toLatency(duration))
        rate = toMemoryBandwidth(rate)
        period = fromSeconds(block_size / rate)
        min_period = period
        max_period = period
        yield self.generator.createLinear(
            duration,
            min_addr,
            max_addr,
            block_size,
            min_period,
            max_period,
            rd_perc,
            data_limit,
        )
        yield self.generator.createExit(0)

    def _create_random_traffic(
        self,
        duration: str,
        rate: str,
        block_size: int,
        min_addr: int,
        max_addr: int,
        rd_perc: int,
        data_limit: int,
    ) -> None:
        """
        This function yields (creates) a random traffic based on the input
        params. Then it will yield (create) an exit traffic (exit traffic is
        used to exit the simulation).

        :param duration: The number of ticks for the generator core to generate
        traffic.
        :param rate: The rate at which the synthetic data is read/written.
        :param block_size: The number of bytes to be read/written with each
        request.
        :param min_addr: The lower bound of the address range the generator
        will read/write from/to.
        :param max_addr: The upper bound of the address range the generator
        will read/write from/to.
        :param rd_perc: The percentage of read requests among all the generated
        requests. The write percentage would be equal to 100 - rd_perc.
        :param data_limit: The amount of data in bytes to read/write by the
        generator before stopping generation.
        """
        duration = fromSeconds(toLatency(duration))
        rate = toMemoryBandwidth(rate)
        period = fromSeconds(block_size / rate)
        min_period = period
        max_period = period
        yield self.generator.createRandom(
            duration,
            min_addr,
            max_addr,
            block_size,
            min_period,
            max_period,
            rd_perc,
            data_limit,
        )
        yield self.generator.createExit(0)
