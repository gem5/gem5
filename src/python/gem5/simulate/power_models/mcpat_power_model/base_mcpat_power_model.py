# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from typing import (
    Dict,
    Union,
)

from m5.objects import (
    Root,
    SimObject,
)

from ..abstract_power_model import AbstractPowerModel

ActEnergyType = Dict[str, Union[float, Dict[str, float]]]


class BaseMcPATPowerModel(AbstractPowerModel):
    def __init__(self, simobj: SimObject, act_energies: ActEnergyType):
        super().__init__(simobj)
        self.name = "BaseMcPATPowerModel"
        self._act_energies = act_energies

    """
    BaseMcPATPowerModel takes a SimObject and an activation energy
    dictionary, which is the number of joules a given operation consumes
    mapped such as addition or cache hits.

    Objects modeled using the McPAT Power Model inherit from this class.
    Each class which inherits from this class must implement/describe
    how dynamic and static energy are modeled.
    """

    def getExecutionTime(self):
        """
        Obtains the execution time of a given program,
        assuming only one region of interest/workload

        :returns: the number of seconds your simulation ran for
        """
        return Root.getInstance().resolveStat("simSeconds").total

    def convert_to_watts(self, value: float) -> float:
        """
        Obtains the number of watts, given the number of joules

        :param value: A given number of Joules

        :returns: value converted into Watts
        """
        time = self.getExecutionTime()
        return value / time
