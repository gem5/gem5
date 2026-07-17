# Copyright (c) 2025 Arm Limited
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

import csv
import json
import re
from abc import (
    ABC,
    abstractmethod,
)
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

from _m5 import stats as _m5_stats


class Visitor(ABC):
    """
    Generic Visitor
    """

    @abstractmethod
    def visit_scalar(self, element: Scalar) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_distribution(self, element: Distribution) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_vector(self, element: Vector) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_vector2d(self, element: Vector2d) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_sparse_hist(self, element: SparseHist) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_simobject_group(self, element: SimObjectGroup) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_simstat(self, element: SimStat) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def visit_group(self, element: Group) -> Dict:
        raise NotImplementedError

    @abstractmethod
    def dump(self, roots: Union[List[SimObject], Root], **kwargs) -> None:
        raise NotImplementedError

    def _acceptable_type(self, element):
        if type(element) in [
            Scalar,
            Distribution,
            Vector,
            Vector2d,
            SparseHist,
            SimObjectGroup,
            SimObjectVectorGroup,
            SimStat,
            Group,
        ]:
            return True
        return False


class CsvOutputVisitor(Visitor):
    """
    This is a helper vistor class used to include a CSV output via the stats
    API (``src/python/m5/stats/__init__.py``).

    It outputs a CSV file with a single header row followed by data rows.

    Format:
         key1,key2,key3
         val1_dump1,val2_dump1,val3_dump1
         val1_dump2,val2_dump2,val3_dump2

    The first row defines column names with each subsequent row
    representing the values of one stats dump.
    """

    file: str

    file_created = False

    def __init__(self, file: str):
        """
        :param file: The output file location in which the CSV file will be dumped.

        """

        self.file = file

    def visit_scalar(self, element: Scalar):
        return {
            "value": element.value,
        }

    def visit_distribution(self, element: Distribution):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
            "min": element.min,
            "max": element.max,
            "num_bins": element.num_bins,
            "bin_size": element.bin_size,
            "sum": element.sum,
            "underflow": element.underflow,
            "overflow": element.overflow,
            "logs": element.logs,
            "sum_squared": element.sum_squared,
        }

    def visit_vector(self, element: Vector):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
        }

    def visit_vector2d(self, element: Vector2d):
        return {
            "value": {
                key: self.visit_vector(value)
                for key, value in element.value.items()
            },
        }

    def visit_sparse_hist(self, element: SparseHist):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
        }

    def visit_group(self, element: Group):
        values = {}
        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)
        return values

    def visit_simobject_group(self, element: SimObjectGroup):
        values = {}
        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)
        return values

    def visit_simobject_vector_group(self, element: SimObjectVectorGroup):
        values = {}
        for key, value in element.values.items():
            vlist = []
            for v in value:
                assert self._acceptable_type(v), "Unexpected value type"
                vlist.append(v.accept(self))
            values[key] = vlist
        return values

    def visit_simstat(self, element: SimStat):
        values = {
            "time_conversion": element.time_conversion,
            "creation_time": element.creation_time.isoformat(
                timespec="seconds"
            ),
            "simulated_begin_time": element.simulated_begin_time,
            "simulated_end_time": element.simulated_end_time,
        }

        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)

        return values

    def flatten_dict(self, d, parent_key="", sep="."):
        items = {}
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k

            if isinstance(v, dict):
                # Recursively flatten nested dicts
                items.update(self.flatten_dict(v, new_key, sep=sep))
            elif isinstance(v, (list, tuple)):
                # Flatten arrays by indexing elements
                for i, item in enumerate(v):
                    if isinstance(item, dict):
                        # Recurse if element is a dict
                        items.update(
                            self.flatten_dict(item, f"{new_key}{i}", sep=sep)
                        )
                    else:
                        items[f"{new_key}{i}"] = item
            else:
                # Primitive value
                items[new_key] = v

        return items

    def dump(self, roots: Union[List[SimObject], Root], **kwargs) -> None:
        """
        Dumps the stats of a simulation root (or list of roots) to the output
        CSV file specified in the constructor.


        :param roots: The Root, or List of roots, whose stats are are to be dumped JSON.
        """

        simstat = get_simstat(root=roots, prepare_stats=False)
        vals = self.visit_simstat(simstat)
        flat_dict = self.flatten_dict(vals)

        if not self.file_created:
            with open(self.file, "w") as fp:
                writer = csv.writer(fp)
                writer.writerow(flat_dict.keys())
                writer.writerow(flat_dict.values())
            self.file_created = True
        else:
            with open(self.file, newline="") as fp:
                reader = csv.reader(fp)
                header = next(reader)

            row = [flat_dict.get(key, "") for key in header]

            with open(self.file, "a", newline="") as fp:
                writer = csv.writer(fp)
                writer.writerow(row)


class JsonOutputVistor(Visitor):
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

    def visit_scalar(self, element: Scalar):
        return {
            "value": element.value,
            "type": element.type,
            "description": element.description,
            "unit": element.unit,
            "datatype": element.datatype.name,
        }

    def visit_distribution(self, element: Distribution):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
            "type": element.type,
            "description": element.description,
            "min": element.min,
            "max": element.max,
            "num_bins": element.num_bins,
            "bin_size": element.bin_size,
            "sum": element.sum,
            "underflow": element.underflow,
            "overflow": element.overflow,
            "logs": element.logs,
            "sum_squared": element.sum_squared,
        }

    def visit_vector(self, element: Vector):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
            "type": element.type,
            "description": element.description,
        }

    def visit_vector2d(self, element: Vector2d):
        return {
            "value": {
                key: self.visit_vector(value)
                for key, value in element.value.items()
            },
            "type": element.type,
            "description": element.description,
        }

    def visit_sparse_hist(self, element: SparseHist):
        return {
            "value": {
                key: self.visit_scalar(value)
                for key, value in element.value.items()
            },
            "type": element.type,
            "description": element.description,
        }

    def visit_group(self, element: Group):
        values = {
            "type": element.type,
            "time_conversion": element.time_conversion,
        }
        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)
        return values

    def visit_simobject_group(self, element: SimObjectGroup):
        values = {
            "type": element.type,
            "time_conversion": element.time_conversion,
            "name": element.name,
        }
        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)
        return values

    def visit_simobject_vector_group(self, element: SimObjectVectorGroup):
        values = {
            "type": element.type,
            "time_conversion": element.time_conversion,
        }
        for key, value in element.values.items():
            vlist = []
            for v in value:
                assert self._acceptable_type(v), "Unexpected value type"
                vlist.append(v.accept(self))
            values[key] = vlist
        return values

    def visit_simstat(self, element: SimStat):
        values = {
            "type": element.type,
            "time_conversion": element.time_conversion,
            "creation_time": element.creation_time.isoformat(
                timespec="seconds"
            ),
            "simulated_begin_time": element.simulated_begin_time,
            "simulated_end_time": element.simulated_end_time,
        }

        for key, value in element.values.items():
            assert self._acceptable_type(value), "Unexpected value type"
            values[key] = value.accept(self)

        values["name"] = element.name
        return values

    def dump(self, roots: Union[List[SimObject], Root], **kwargs) -> None:
        """
        Dumps the stats of a simulation root (or list of roots) to the output
        JSON file specified in the JsonOutput constructor.

        .. warning::

            This dump assumes the statistics have already been prepared
            for the target root.


        :param roots: The Root, or List of roots, whose stats are are to be dumped JSON.
        """

        if "indent" not in kwargs:
            kwargs["indent"] = 4

        with open(self.file, "w") as fp:
            simstat = get_simstat(root=roots, prepare_stats=False)
            json.dump(obj=self.visit_simstat(simstat), fp=fp, **kwargs)


def __get_statistic(statistic: _m5_stats.Info) -> Optional[Statistic]:
    """
    Translates a _m5.stats.Info object into a Statistic object, to process
    statistics at the Python level.

    :param statistic: The Info object to be translated to a Statistic object.

    :returns: The Statistic object of the Info object. Returns ``None`` if
              Info object cannot, or should not, be translated.
    """

    assert isinstance(statistic, _m5_stats.Info)
    statistic.prepare()

    if isinstance(statistic, _m5_stats.ScalarInfo):
        if statistic.is_nozero and statistic.value == 0.0:
            # In the case where the "nozero" flag is set, and the value is
            # zero, we don't want to include this statistic so return None.
            return None
        return __get_scaler(statistic)
    elif isinstance(statistic, _m5_stats.DistInfo):
        return __get_distribution(statistic)
    elif isinstance(statistic, _m5_stats.FormulaInfo):
        # We don't do anything with Formula's right now.
        # We may never do so, see https://gem5.atlassian.net/browse/GEM5-868.
        pass
    elif isinstance(statistic, _m5_stats.VectorInfo):
        return __get_vector(statistic)
    elif isinstance(statistic, _m5_stats.Vector2dInfo):
        return __get_vector2d(statistic)
    elif isinstance(statistic, _m5_stats.SparseHistInfo):
        return __get_sparse_hist(statistic)

    return None


def __get_scaler(statistic: _m5_stats.ScalarInfo) -> Scalar:
    value = statistic.value
    unit = statistic.unit
    description = statistic.desc
    # ScalarInfo uses the C++ `double`.
    datatype = StorageType["f64"]

    return Scalar(
        value=value, unit=unit, description=description, datatype=datatype
    )


def __get_distribution(statistic: _m5_stats.DistInfo) -> Distribution:
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


def __get_vector(statistic: _m5_stats.VectorInfo) -> Vector:
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


def __get_vector2d(statistic: _m5_stats.Vector2dInfo) -> Vector2d:
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


def __get_sparse_hist(statistic: _m5_stats.SparseHistInfo) -> SparseHist:
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


def _prepare_stats(group: _m5_stats.Group):
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

    stats = {}

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
            stats[name] = Group(
                type="Group", **_process_simobject_stats(child)
            )

    return SimObjectGroup(name=simobject.get_name(), **stats)


def _process_group(group: _m5_stats.Group) -> dict:
    out = {}
    for stat in group.getStats():
        val = __get_statistic(stat)
        if val is not None:
            out[stat.name] = val
    for name, sub in group.getStatGroups().items():
        out[name] = Group(type="Group", **_process_group(sub))
    return out


def _process_simobject_stats(
    simobject: Union[
        _m5_stats.Group,
        SimObject,
        SimObjectVector,
        List[Union[SimObject, SimObjectVector]],
    ],
) -> Union[List[Dict], Dict]:
    """
    Processes the stats of a SimObject, SimObjectVector, or List of either, and
    returns a dictionary of the PySqtats for the SimObject.

    :param simobject: The SimObject to process the stats for.

    :returns: A dictionary of the stats for the SimObject.
    """

    if isinstance(simobject, SimObject):
        return _process_simobject_object(simobject)

    if isinstance(simobject, (list, SimObjectVector)):
        stats_list = []
        for obj in simobject:
            stats_list.append(_process_simobject_stats(obj))
        return SimObjectVectorGroup(children=stats_list)

    if isinstance(simobject, _m5_stats.Group):
        return _process_group(simobject)

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
        _m5_stats.processDumpQueue()

    stats_map = {}
    for r in root:
        if prepare_stats:
            if isinstance(r, list):
                for obj in r:
                    _prepare_stats(obj)
            else:
                _prepare_stats(r)

        stats = _process_simobject_stats(r).values
        stats["name"] = r.get_name() if r.get_name() else "root"
        stats_map[stats["name"]] = stats

    if len(stats_map) == 1:
        stats_map = stats_map[next(iter(stats_map))]

    creation_time = datetime.now()
    time_converstion = None  # TODO https://gem5.atlassian.net/browse/GEM5-846
    final_tick = Root.getInstance().resolveStat("finalTick").value
    sim_ticks = Root.getInstance().resolveStat("simTicks").value
    simulated_begin_time = int(final_tick - sim_ticks)
    simulated_end_time = int(final_tick)

    return SimStat(
        creation_time=creation_time,
        simulated_begin_time=simulated_begin_time,
        simulated_end_time=simulated_end_time,
        **stats_map,
    )
