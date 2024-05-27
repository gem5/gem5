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
from typing import (
    IO,
    List,
    Union,
)

from m5.ext.pystats.group import *
from m5.ext.pystats.simstat import *
from m5.ext.pystats.statistic import *
from m5.ext.pystats.storagetype import *
from m5.objects import *
from m5.params import SimObjectVector

import _m5.stats


class JsonOutputVistor:
    """
    This is a helper vistor class used to include a JSON output via the stats
    API (``src/python/m5/stats/__init__.py``).
    """

    file: str
    json_args: Dict

    def __init__(self, file: str, **kwargs):
        """
        :param file: The output file location in which the JSON will be dumped.

        :param kwargs: Additional parameters to be passed to the ``json.dumps`` method.
        """

        self.file = file
        self.json_args = kwargs

    def dump(self, roots: Union[List[SimObject], Root]) -> None:
        """
        Dumps the stats of a simulation root (or list of roots) to the output
        JSON file specified in the JsonOutput constructor.

        .. warning::

            This dump assumes the statistics have already been prepared
            for the target root.


        :param roots: The Root, or List of roots, whose stats are are to be dumped JSON.
        """

        with open(self.file, "w") as fp:
            simstat = get_simstat(root=roots, prepare_stats=False)
            simstat.dump(fp=fp, **self.json_args)


def __get_statistic(statistic: _m5.stats.Info) -> Optional[Statistic]:
    """
    Translates a _m5.stats.Info object into a Statistic object, to process
    statistics at the Python level.

    :param statistic: The Info object to be translated to a Statistic object.

    :returns: The Statistic object of the Info object. Returns ``None`` if
              Info object cannot, or should not, be translated.
    """

    assert isinstance(statistic, _m5.stats.Info)
    statistic.prepare()

    if isinstance(statistic, _m5.stats.ScalarInfo):
        if statistic.is_nozero and statistic.value == 0.0:
            # In the case where the "nozero" flag is set, and the value is
            # zero, we don't want to include this statistic so return None.
            return None
        return __get_scaler(statistic)
    elif isinstance(statistic, _m5.stats.DistInfo):
        return __get_distribution(statistic)
    elif isinstance(statistic, _m5.stats.FormulaInfo):
        # We don't do anything with Formula's right now.
        # We may never do so, see https://gem5.atlassian.net/browse/GEM5-868.
        pass
    elif isinstance(statistic, _m5.stats.VectorInfo):
        return __get_vector(statistic)
    elif isinstance(statistic, _m5.stats.Vector2dInfo):
        return __get_vector2d(statistic)
    elif isinstance(statistic, _m5.stats.SparseHistInfo):
        return __get_sparse_hist(statistic)

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

    parsed_values = {}
    for index in range(len(value)):
        parsed_values[index] = Scalar(
            value=value[index],
            unit=statistic.unit,
            datatype=StorageType["f64"],
        )

    return Distribution(
        value=parsed_values,
        min=min,
        max=max,
        num_bins=num_bins,
        bin_size=bin_size,
        sum=sum_val,
        sum_squared=sum_squared,
        underflow=underflow,
        overflow=overflow,
        logs=logs,
        description=description,
    )


def __get_vector(statistic: _m5.stats.VectorInfo) -> Vector:
    vec: Dict[Union[str, int, float], Scalar] = {}

    for index in range(statistic.size):
        # All the values in a Vector are Scalar values
        value = statistic.value[index]
        assert isinstance(value, float) or isinstance(value, int)

        # Sometimes elements within a vector are defined by their name. Other
        # times they have no name. When a name is not available, we name the
        # stat the index value.
        if len(statistic.subnames) > index and statistic.subnames[index]:
            index_subname = str(statistic.subnames[index])
            if index_subname.isdigit():
                index_subname = int(index_subname)
            elif index_subname.isnumeric():
                index_subname = float(index_subname)
        else:
            index_subname = index

        index_subdesc = None
        if len(statistic.subdescs) > index and statistic.subdescs[index]:
            index_subdesc = str(statistic.subdescs[index])
        else:
            index_subdesc = statistic.desc

        vec[index_subname] = Scalar(
            value=value,
            unit=statistic.unit,
            description=index_subdesc,
            datatype=StorageType["f64"],
        )

    return Vector(
        vec,
        type="Vector",
        description=statistic.desc,
    )


def __get_vector2d(statistic: _m5.stats.Vector2dInfo) -> Vector2d:
    # All the values in a 2D Vector are Scalar values
    description = statistic.desc
    x_size = statistic.x_size
    y_size = statistic.y_size

    vector_rep: Dict[Union[str, int, float], Vector] = {}
    for x_index in range(x_size):
        x_index_string = x_index
        if x_index in statistic.subnames:
            x_index_string = str(statistic.subnames[x_index])

        x_desc = description
        if x_index in statistic.subdescs:
            x_desc = str(statistic.subdescs[x_index])
        x_vec: Dict[str, Scalar] = {}
        for y_index in range(y_size):
            y_index_val = y_index
            if y_index in statistic.ysubnames:
                y_index_val = str(statistic.subnames[y_index])

            x_vec[y_index_val] = Scalar(
                value=statistic.value[x_index * y_size + y_index],
                unit=statistic.unit,
                datatype=StorageType["f64"],
            )

        vector_rep[x_index_string] = Vector(
            x_vec,
            type="Vector",
            description=x_desc,
        )

    return Vector2d(value=vector_rep, type="Vector2d", description=description)


def __get_sparse_hist(statistic: _m5.stats.SparseHistInfo) -> SparseHist:
    description = statistic.desc
    value = statistic.values

    parsed_values = {}
    for val in value:
        parsed_values[val] = Scalar(
            value=value[val],
            unit=statistic.unit,
            datatype=StorageType["f64"],
        )

    return SparseHist(
        value=parsed_values,
        description=description,
    )


def _prepare_stats(group: _m5.stats.Group):
    """
    Prepares the statistics for dumping.
    """

    group.preDumpStats()

    for stat in group.getStats():
        stat.prepare()

    for child in group.getStatGroups().values():
        _prepare_stats(child)


def _process_simobject_object(simobject: SimObject) -> SimObjectGroup:
    """
    Processes the stats of a SimObject, and returns a dictionary of the stats
    for the SimObject with PyStats objects when appropriate.

    :param simobject: The SimObject to process the stats for.

    :returns: A dictionary of the PyStats stats for the SimObject.
    """

    assert isinstance(
        simobject, SimObject
    ), "simobject param must be a SimObject."

    stats = (
        {
            "name": simobject.get_name(),
        }
        if simobject.get_name()
        else {}
    )

    for stat in simobject.getStats():
        val = __get_statistic(stat)
        if val:
            stats[stat.name] = val

    for name, child in simobject._children.items():
        to_add = _process_simobject_stats(child)
        if to_add:
            stats[name] = to_add

    for name, child in sorted(simobject.getStatGroups().items()):
        # Note: We are using the name of the group to determine if we have
        # already processed the group as a child simobject or a statistic.
        # This is to avoid SimObjectVector's being processed twice. It is far
        # from an ideal solution, but it works for now.
        if not any(
            re.compile(f"{to_match}" + r"\d*").search(name)
            for to_match in stats.keys()
        ):
            stats[name] = Group(**_process_simobject_stats(child))

    return SimObjectGroup(**stats)


def _process_simobject_stats(
    simobject: Union[
        SimObject, SimObjectVector, List[Union[SimObject, SimObjectVector]]
    ]
) -> Union[List[Dict], Dict]:
    """
    Processes the stats of a SimObject, SimObjectVector, or List of either, and
    returns a dictionary of the PySqtats for the SimObject.

    :param simobject: The SimObject to process the stats for.

    :returns: A dictionary of the stats for the SimObject.
    """

    if isinstance(simobject, SimObject):
        return _process_simobject_object(simobject)

    if isinstance(simobject, Union[List, SimObjectVector]):
        stats_list = []
        for obj in simobject:
            stats_list.append(_process_simobject_stats(obj))
        return SimObjectVectorGroup(value=stats_list)

    return {}


def get_simstat(
    root: Union[
        Union[SimObject, SimObjectVector],
        List[Union[SimObject, SimObjectVector]],
    ],
    prepare_stats: bool = True,
) -> SimStat:
    """
    This function will return the SimStat object for a simulation given a
    SimObject (typically a Root SimObject), or list of SimObjects. The returned
    SimStat object will contain all the stats for all the SimObjects contained
    within the "root", inclusive of the "root" SimObject/SimObjects.

    :param root: A SimObject, or list of SimObjects, of the simulation for
                 translation into a SimStat object. Typically this is the
                 simulation's Root SimObject as this will obtain the entirety
                 of a run's statistics in a single SimStat object.

    :param prepare_stats: Dictates whether the stats are to be prepared prior
                          to creating the SimStat object. By default this is
                          ``True``.

    :Returns: The SimStat Object of the current simulation.

    """

    if prepare_stats:
        _m5.stats.processDumpQueue()

    stats_map = {}
    for r in root:
        creation_time = datetime.now()
        time_converstion = (
            None  # TODO https://gem5.atlassian.net/browse/GEM5-846
        )
        final_tick = Root.getInstance().resolveStat("finalTick").value
        sim_ticks = Root.getInstance().resolveStat("simTicks").value
        simulated_begin_time = int(final_tick - sim_ticks)
        simulated_end_time = int(final_tick)

        if prepare_stats:
            if isinstance(r, list):
                for obj in r:
                    _prepare_stats(obj)
            else:
                _prepare_stats(r)

        stats = _process_simobject_stats(r).__dict__
        stats["name"] = r.get_name() if r.get_name() else "root"
        stats_map[stats["name"]] = stats

    if len(stats_map) == 1:
        stats_map = stats_map[next(iter(stats_map))]

    return SimStat(
        creation_time=creation_time,
        simulated_begin_time=simulated_begin_time,
        simulated_end_time=simulated_end_time,
        **stats_map,
    )
