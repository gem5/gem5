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

"""
This serves as the bridge between the gem5 statistics exposed via PyBind11 and
the Python Stats model.
"""

from datetime import datetime
from typing import IO, List, Union

import _m5.stats
from m5.objects import *
from m5.ext.pystats.group import *
from m5.ext.pystats.simstat import *
from m5.ext.pystats.statistic import *
from m5.ext.pystats.storagetype import *


class JsonOutputVistor:
    """
    This is a helper vistor class used to include a JSON output via the stats
    API (`src/python/m5/stats/__init__.py`).
    """

    file: str
    json_args: Dict

    def __init__(self, file: str, **kwargs):
        """
        Parameters
        ----------

        file: str
            The output file location in which the JSON will be dumped.

        kwargs: Dict[str, Any]
            Additional parameters to be passed to the `json.dumps` method.
        """

        self.file = file
        self.json_args = kwargs

    def dump(self, roots: Union[List[SimObject], Root]) -> None:
        """
        Dumps the stats of a simulation root (or list of roots) to the output
        JSON file specified in the JsonOutput constructor.

        WARNING: This dump assumes the statistics have already been prepared
        for the target root.

        Parameters
        ----------

        roots: Union[List[Root], Root]]
            The Root, or List of roots, whose stats are are to be dumped JSON.
        """

        with open(self.file, "w") as fp:
            simstat = get_simstat(root=roots, prepare_stats=False)
            simstat.dump(fp=fp, **self.json_args)


def get_stats_group(group: _m5.stats.Group) -> Group:
    """
    Translates a gem5 Group object into a Python stats Group object. A Python
    statistic Group object is a dictionary of labeled Statistic objects. Any
    gem5 object passed to this will have its `getStats()` and `getStatGroups`
    function called, and all the stats translated (inclusive of the stats
    further down the hierarchy).

    Parameters
    ----------
    group: _m5.stats.Group
        The gem5 _m5.stats.Group object to be translated to be a Python stats
        Group object. Typically this will be a gem5 SimObject.

    Returns
    -------
    Group
        The stats group object translated from the input gem5 object.
    """

    stats_dict = {}

    for stat in group.getStats():
        statistic = __get_statistic(stat)
        if statistic is not None:
            stats_dict[stat.name] = statistic

    for key in group.getStatGroups():
        stats_dict[key] = get_stats_group(group.getStatGroups()[key])

    return Group(**stats_dict)


def __get_statistic(statistic: _m5.stats.Info) -> Optional[Statistic]:
    """
    Translates a _m5.stats.Info object into a Statistic object, to process
    statistics at the Python level.

    Parameters
    ----------
    statistic: Info
        The Info object to be translated to a Statistic object.

    Returns
    -------
    Optional[Statistic]
        The Statistic object of the Info object. Returns None if Info object
        cannot be translated.
    """

    assert isinstance(statistic, _m5.stats.Info)
    statistic.prepare()

    if isinstance(statistic, _m5.stats.ScalarInfo):
        return __get_scaler(statistic)
    elif isinstance(statistic, _m5.stats.DistInfo):
        return __get_distribution(statistic)
    elif isinstance(statistic, _m5.stats.FormulaInfo):
        # We don't do anything with Formula's right now.
        # We may never do so, see https://gem5.atlassian.net/browse/GEM5-868.
        pass
    elif isinstance(statistic, _m5.stats.VectorInfo):
        return __get_vector(statistic)

    return None


def __get_scaler(statistic: _m5.stats.ScalarInfo) -> Scalar:
    value = statistic.value
    unit = statistic.unit
    description = statistic.desc
    # ScalarInfo uses the C++ `double`.
    datatype = StorageType["f64"]

    return Scalar(
        value=value, unit=unit, description=description, datatype=datatype
    )


def __get_distribution(statistic: _m5.stats.DistInfo) -> Distribution:
    unit = statistic.unit
    description = statistic.desc
    value = statistic.values
    bin_size = statistic.bucket_size
    min = statistic.min_val
    max = statistic.max_val
    num_bins = len(value)
    sum_val = statistic.sum
    sum_squared = statistic.squares
    underflow = statistic.underflow
    overflow = statistic.overflow
    logs = statistic.logs
    # DistInfo uses the C++ `double`.
    datatype = StorageType["f64"]

    return Distribution(
        value=value,
        min=min,
        max=max,
        num_bins=num_bins,
        bin_size=bin_size,
        sum=sum_val,
        sum_squared=sum_squared,
        underflow=underflow,
        overflow=overflow,
        logs=logs,
        unit=unit,
        description=description,
        datatype=datatype,
    )


def __get_vector(statistic: _m5.stats.VectorInfo) -> Vector:
    to_add = dict()

    for index in range(statistic.size):
        # All the values in a Vector are Scalar values
        value = statistic.value[index]
        unit = statistic.unit
        description = statistic.subdescs[index]
        # ScalarInfo uses the C++ `double`.
        datatype = StorageType["f64"]

        # Sometimes elements within a vector are defined by their name. Other
        # times they have no name. When a name is not available, we name the
        # stat the index value.
        if str(statistic.subnames[index]):
            index_string = str(statistic.subnames[index])
        else:
            index_string = str(index)

        to_add[index_string] = Scalar(
            value=value, unit=unit, description=description, datatype=datatype
        )

    return Vector(scalar_map=to_add)


def _prepare_stats(group: _m5.stats.Group):
    """
    Prepares the statistics for dumping.
    """

    group.preDumpStats()

    for stat in group.getStats():
        stat.prepare()

    for child in group.getStatGroups().values():
        _prepare_stats(child)


def get_simstat(
    root: Union[SimObject, List[SimObject]], prepare_stats: bool = True
) -> SimStat:
    """
    This function will return the SimStat object for a simulation given a
    SimObject (typically a Root SimObject), or list of SimObjects. The returned
    SimStat object will contain all the stats for all the SimObjects contained
    within the "root", inclusive of the "root" SimObject/SimObjects.

    Parameters
    ----------
    root: Union[SimObject, List[SimObject]]
        A SimObject, or list of SimObjects, of the simulation for translation
        into a SimStat object. Typically this is the simulation's Root
        SimObject as this will obtain the entirety of a run's statistics in a
        single SimStat object.

    prepare_stats: bool
        Dictates whether the stats are to be prepared prior to creating the
        SimStat object. By default this is 'True'.

    Returns
    -------
    SimStat
        The SimStat Object of the current simulation.

    """
    stats_map = {}
    creation_time = datetime.now()
    time_converstion = None  # TODO https://gem5.atlassian.net/browse/GEM5-846
    final_tick = Root.getInstance().resolveStat("finalTick").value
    sim_ticks = Root.getInstance().resolveStat("simTicks").value
    simulated_begin_time = int(final_tick - sim_ticks)
    simulated_end_time = int(final_tick)

    if prepare_stats:
        _m5.stats.processDumpQueue()

    for r in root:
        if isinstance(r, Root):
            # The Root is a special case, we jump directly into adding its
            # constituent Groups.
            if prepare_stats:
                _prepare_stats(r)
            for key in r.getStatGroups():
                stats_map[key] = get_stats_group(r.getStatGroups()[key])
        elif isinstance(r, SimObject):
            if prepare_stats:
                _prepare_stats(r)
            stats_map[r.get_name()] = get_stats_group(r)
        else:
            raise TypeError(
                "Object (" + str(r) + ") passed is not a "
                "SimObject. " + __name__ + " only processes "
                "SimObjects, or a list of  SimObjects."
            )

    return SimStat(
        creation_time=creation_time,
        time_conversion=time_converstion,
        simulated_begin_time=simulated_begin_time,
        simulated_end_time=simulated_end_time,
        **stats_map,
    )
