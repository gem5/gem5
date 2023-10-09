# Copyright (c) 2023 The Regents of the University of California
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
import csv
import json
import os
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional
from typing import Union

import m5
from m5.objects import PcCountTrackerManager
from m5.params import PcCountPair


class LooppointRegionPC:
    """A data structure for storing the Looppoint region's PC information.

    **Note**: This is not intended to be a user-facing class. The classes
    `LooppointJsonLoader` and `LooppointCSVLoader` can be used to load
    and restore Simpoint data.
    """

    def __init__(self, pc: int, globl: int, relative: Optional[int] = None):
        """
        :param pc: The Program Counter value of this region.
        :param globl: The global value of this region.
        :param relative: The relative program counter value. Optional.
        """
        self._pc = pc
        self._global = globl
        self._relative = relative

    def get_pc(self) -> int:
        """Returns the Program counter value."""
        return self._pc

    def get_global(self) -> int:
        """Returns the global value."""
        return self._global

    def get_relative(self) -> Optional[int]:
        """If specified, returns the relative Program counter value, otherwise
        returns None."""
        return self._relative

    def get_pc_count_pair(self) -> PcCountPair:
        """Returns the PcCountPair for this Region PC value."""
        return PcCountPair(self.get_pc(), self.get_global())

    def update_relative_count(self, manager: PcCountTrackerManager) -> None:
        """Updates the relative count."""
        self._relative = int(
            self.get_global() - manager.getPcCount(self.get_pc())
        )

    def to_json(self) -> Dict[str, int]:
        """Returns this class in a JSON structure which can then be serialized
        and later be restored from."""
        to_return = {
            "pc": self.get_pc(),
            "global": self.get_global(),
        }
        if self._relative:
            to_return["relative"] = self.get_relative()

        return to_return


class LooppointRegionWarmup:
    """A data structure for storing a Looppoint region's warmup data.

    **Note**: This is not intended to be a user-facing class. The classes
    `LooppointJsonLoader` and `LooppointCSVLoader` can be used to load
    and restore Simpoint data.
    """

    def __init__(self, start: PcCountPair, end: PcCountPair):
        """
        :param start: The starting PcCountPair.
        :param end: The ending PcCountPair.
        """
        self._start = start
        self._end = end

    def get_start(self) -> PcCountPair:
        """Returns the PcCountPair for the start of the region warmup."""
        return self._start

    def get_end(self) -> PcCountPair:
        """Returns the PcCountPair for the end of the region warmup."""
        return self._end

    def get_pc_count_pairs(self) -> List[PcCountPair]:
        """Returns the start and end PC count pairs."""
        return [self.get_start(), self.get_end()]

    def to_json(self) -> Dict[str, Dict[str, int]]:
        """Returns this class in a JSON structure which can then be
        serialized."""
        return {
            "start": {
                "pc": self.get_start().pc,
                "count": self.get_start().count,
            },
            "end": {
                "pc": self.get_end().pc,
                "count": self.get_end().count,
            },
        }


class LooppointSimulation:
    """A data structure to store the simulation region start and end region.

    **Note**: This is not intended to be a user-facing class. The classes
    `LooppointJsonLoader` and `LooppointCSVLoader` can be used to load
    and restore Simpoint data.
    """

    def __init__(self, start: LooppointRegionPC, end: LooppointRegionPC):
        """
        :param start: The starting LooppointRegionPC.
        :param end: The ending LoopppointRegionPC.
        """
        self._start = start
        self._end = end

    def get_start(self) -> LooppointRegionPC:
        """Returns the starting LooppointRegionPC data structure."""
        return self._start

    def get_end(self) -> LooppointRegionPC:
        """Returns the ending LooppointRegionPC data structure."""
        return self._end

    def get_pc_count_pairs(self) -> List[PcCountPair]:
        """Returns the PC count pairs for the start and end
        LoopointRegionPCs."""
        return [
            self.get_start().get_pc_count_pair(),
            self.get_end().get_pc_count_pair(),
        ]

    def update_relatives_counts(
        self, manager: PcCountTrackerManager, include_start: bool = False
    ) -> None:
        """Updates the relative counts for this simulation region."""
        if include_start:
            # if this region has a warmup interval,
            # then update the relative count for the
            # start of the simulation region
            self.get_start().update_relative_count(manager=manager)

        self.get_end().update_relative_count(manager=manager)

    def to_json(self) -> Dict:
        """Returns this class in a JSON structure which can then be serialized
        and later be restored from."""
        return {
            "start": self.get_start().to_json(),
            "end": self.get_end().to_json(),
        }


class LooppointRegion:
    """A data structure to store Looppoint region information.

    **Note**: This is not intended to be a user-facing class. The classes
    `LooppointJsonLoader` and `LooppointCSVLoader` can be used to load
    and restore Simpoint data.
    """

    def __init__(
        self,
        simulation: LooppointSimulation,
        multiplier: float,
        warmup: Optional[LooppointRegionWarmup] = None,
    ):
        """
        :param simulation: The simulation information for this Looppoint
        region.
        :param multiplier: The multiplier for this Looppoint region.
        :param warmup: The warmup information for this Looppoint region.
        Optional.
        """
        self._simulation = simulation
        self._multiplier = multiplier
        self._warmup = warmup

    def get_simulation(self) -> LooppointSimulation:
        """Returns the simulation region information."""
        return self._simulation

    def get_multiplier(self) -> float:
        """Returns the multiplier."""
        return self._multiplier

    def get_warmup(self) -> Optional[LooppointRegionWarmup]:
        """If set, returns the warmup region information. Otherwise None."""
        return self._warmup

    def get_pc_count_pairs(self) -> List[PcCountPair]:
        """Returns the PC count pairs for this Looppoint region."""
        pc_count_pairs = self.get_simulation().get_pc_count_pairs()
        if self.get_warmup():
            pc_count_pairs.extend(self.get_warmup().get_pc_count_pairs())
        return pc_count_pairs

    def update_relatives_counts(self, manager: PcCountTrackerManager) -> None:
        """Updates the relative counds of this Looppoint region."""
        self.get_simulation().update_relatives_counts(
            manager=manager, include_start=bool(self.get_warmup())
        )

    def get_start(self) -> PcCountPair:
        """Returns the correct starting PcCountPair for this Looppoint
        region."""
        if self.get_warmup():
            return self.get_warmup().get_start()
        return self.get_simulation().get_start().get_pc_count_pair()

    def to_json(self) -> Dict:
        """Returns this class in a JSON structure which can then be serialized
        and later be restored from."""
        to_return = {
            "simulation": self.get_simulation().to_json(),
            "multiplier": self.get_multiplier(),
        }
        if self.get_warmup():
            to_return["warmup"] = self.get_warmup().to_json()
        return to_return


class Looppoint:
    """Stores all the Looppoint information for a gem5 workload."""

    def __init__(self, regions: Dict[Union[str, int], LooppointRegion]):
        """
        :param regions: A dictionary mapping the region_ids with the
        LooppointRegions.
        """
        self._regions = regions
        self._manager = PcCountTrackerManager()
        self._manager.targets = self.get_targets()

    def set_target_region_id(self, region_id: Union[str, int]) -> None:
        """There are use-cases where we want to obtain a looppoint data
        structure containing a single target region via its ID. This function
        will remove all irrelevant regions."""

        if region_id not in self._regions:
            raise Exception(f"Region ID '{region_id}' cannot be found.")

        to_remove = [rid for rid in self._regions if rid is not region_id]
        for rid in to_remove:
            del self._regions[rid]

        self._manager.targets = self.get_targets()

    def get_manager(self) -> PcCountTrackerManager:
        """Returns the PcCountTrackerManager for this Looppoint data
        structure."""
        return self._manager

    def get_regions(self) -> Dict[Union[int, str], LooppointRegion]:
        """Returns the regions for this Looppoint data structure."""
        return self._regions

    def setup_processor(
        self,
        processor: "AbstractProcessor",
    ) -> None:
        """
        A function is used to setup a PC tracker in all the cores and
        connect all the tracker to the PC tracker manager to perform
        multithread PC tracking.

        :param processor: The processor used in the simulation configuration.
        """
        for core in processor.get_cores():
            core.add_pc_tracker_probe(self.get_targets(), self.get_manager())

    def update_relatives_counts(self) -> None:
        """
        Updates the relative count for restore usage. The new relative count
        will be stored in relevant data structures.
        """
        current_pair = self.get_current_pair()
        region_start_map = self.get_region_start_id_map()
        if current_pair in region_start_map:
            region_id = region_start_map[current_pair]
            self.get_regions()[region_id].update_relatives_counts(
                manager=self.get_manager()
            )

    def get_current_region(self) -> Optional[Union[str, int]]:
        """Returns the region id if the current PC Count pair if significant
        (e.g. beginning of the checkpoint), otherwise, it returns None to
        indicate the current PC Count pair is not significant.
        """
        current_pair = self.get_current_pair()
        region_start_map = self.get_region_start_id_map()
        if current_pair in region_start_map:
            return region_start_map[current_pair]
        return None

    def get_current_pair(self) -> PcCountPair:
        """This function returns the current PC Count pair."""
        return self.get_manager().getCurrentPcCountPair()

    def get_region_start_id_map(self) -> Dict[PcCountPair, Union[int, str]]:
        """Returns the starting PcCountPairs mapped to the corresponding region
        IDs. This is a helper function for quick mapping of PcCountPairs to
        region IDs."""

        regions = {}
        for rid in self.get_regions():
            regions[self.get_regions()[rid].get_start()] = rid

        return regions

    def get_targets(self) -> List[PcCountPair]:
        """Returns the complete list of target PcCountPairs. That is, the
        PcCountPairs each region starts with as well as the relevant warmup
        intervals."""
        targets = []
        for rid in self.get_regions():
            targets.extend(self.get_regions()[rid].get_pc_count_pairs())

        return targets

    def to_json(self) -> Dict[Union[int, str], Dict]:
        """Returns this data-structure as a dictionary for serialization via
        the `output_json_file` function."""
        to_return = {}
        for region_id in self.get_regions():
            to_return[region_id] = self.get_regions()[region_id].to_json()
        return to_return

    def output_json_file(
        self,
        input_indent: int = 4,
        filepath: str = os.path.join(m5.options.outdir, "looppoint.json"),
    ) -> Dict[int, Dict]:
        """
        This function is used to output the _json_file into a json file

        :param input_indent: the indent value of the json file
        :param filepath: the path of the output json file
        """
        with open(filepath, "w") as file:
            json.dump(self.to_json(), file, indent=input_indent)


class LooppointCsvLoader(Looppoint):
    """This class will create a Looppoint data structure from data extracted
    from a Looppoint pinpoints file."""

    def __init__(
        self,
        pinpoints_file: Union[Path, str],
        region_id: Optional[Union[str, int]] = None,
    ):
        """
        :params pinpoints_file: The pinpoints file in which the data is to be
        expected.
        :params region_id: If set, will only load the specified region data.
        Otherwise, all region info is loaded. Is used when restoring to a
        particular region.
        """

        regions = {}
        warmups = {}

        _path = (
            pinpoints_file
            if isinstance(pinpoints_file, Path)
            else Path(pinpoints_file)
        )

        # This section is hard-coded to parse the data in the csv file.
        # The csv file is assumed to have a constant format.
        with open(_path, newline="") as csvfile:
            reader = csv.reader(csvfile, delimiter=" ", quotechar="|")
            for row in reader:
                if len(row) > 1:
                    if row[0] == "cluster":
                        # if it is a simulation region
                        line = row[4].split(",")

                        rid = int(line[2])

                        region_start = LooppointRegionPC(
                            pc=int(line[3], 16),
                            globl=int(line[6]),
                            # From the CSV's I've observed, the start relative
                            # value is never set, while the end is always set.
                            # Given limited information, I can only determine
                            # this is a rule of how the CSV is setup.
                            relative=None,
                        )

                        region_end = LooppointRegionPC(
                            pc=int(line[7], 16),
                            globl=int(line[10]),
                            relative=int(line[11]),
                        )

                        simulation = LooppointSimulation(
                            start=region_start, end=region_end
                        )

                        multiplier = float(line[14])

                        region = LooppointRegion(
                            simulation=simulation, multiplier=multiplier
                        )

                        regions[rid] = region

                    elif row[0] == "Warmup":
                        line = row[3].split(",")
                        rid = int(line[0])
                        start = PcCountPair(int(line[3], 16), int(line[6]))
                        end = PcCountPair(int(line[7], 16), int(line[10]))

                        warmup = LooppointRegionWarmup(start=start, end=end)
                        warmups[rid] = warmup

        for rid in warmups:
            if rid not in regions:
                raise Exception(
                    "Warmup region ID '{rid}' does not have a "
                    "corresponding region."
                )
            regions[rid]._warmup = warmups[rid]

        super().__init__(regions=regions)

        if region_id:
            self.set_target_region_id(region_id=region_id)


class LooppointJsonLoader(Looppoint):
    """This class will create a generate a Looppoint data structure from data
    extracted from a Looppoint json file."""

    def __init__(
        self,
        looppoint_file: Union[str, Path],
        region_id: Optional[Union[str, int]] = None,
    ) -> None:
        """
        :param looppoint_file: a json file generated by gem5 that has all the
        LoopPoint data information
        :params region_id: If set, will only load the specified region data.
        Otherwise, all region info is loaded. Is used when restoring to a
        particular region.
        """

        _path = (
            looppoint_file
            if isinstance(looppoint_file, Path)
            else Path(looppoint_file)
        )

        regions = {}
        with open(_path) as file:
            json_contents = json.load(file)
            for rid in json_contents:

                start_pc = int(json_contents[rid]["simulation"]["start"]["pc"])
                start_globl = int(
                    json_contents[rid]["simulation"]["start"]["global"]
                )
                start_relative = (
                    int(json_contents[rid]["simulation"]["start"]["relative"])
                    if "relative" in json_contents[rid]["simulation"]["start"]
                    else None
                )
                start = LooppointRegionPC(
                    pc=start_pc,
                    globl=start_globl,
                    relative=start_relative,
                )

                end_pc = int(json_contents[rid]["simulation"]["end"]["pc"])
                end_globl = int(
                    json_contents[rid]["simulation"]["end"]["global"]
                )
                end_relative = (
                    int(json_contents[rid]["simulation"]["end"]["relative"])
                    if "relative" in json_contents[rid]["simulation"]["end"]
                    else None
                )
                end = LooppointRegionPC(
                    pc=end_pc,
                    globl=end_globl,
                    relative=end_relative,
                )
                simulation = LooppointSimulation(start=start, end=end)
                multiplier = float(json_contents[rid]["multiplier"])
                warmup = None
                if "warmup" in json_contents[rid]:
                    start = PcCountPair(
                        json_contents[rid]["warmup"]["start"]["pc"],
                        json_contents[rid]["warmup"]["start"]["count"],
                    )
                    end = PcCountPair(
                        json_contents[rid]["warmup"]["end"]["pc"],
                        json_contents[rid]["warmup"]["end"]["count"],
                    )
                    warmup = LooppointRegionWarmup(start=start, end=end)

                regions[rid] = LooppointRegion(
                    simulation=simulation, multiplier=multiplier, warmup=warmup
                )

        super().__init__(regions=regions)
        if region_id:
            self.set_target_region_id(region_id=region_id)
