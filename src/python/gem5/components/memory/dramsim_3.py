import configparser
import os
from typing import (
    List,
    Optional,
    Sequence,
    Tuple,
)

import m5
from m5.objects import (
    AddrRange,
    DRAMsim3,
    MemCtrl,
    Port,
)
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem


def config_ds3(mem_type: str, num_chnls: int) -> Tuple[str, str]:
    """
    This function creates a config file that will be used to create a memory
    controller of type DRAMSim3. It stores the config file in ``/tmp/`` directory.

    :param mem_type: The name for the type of the memory to be configured.

    :param num_chnls: The number of channels to configure for the memory

    :returns: A tuple containing the output file and the output directory.
    """
    config = configparser.ConfigParser()

    # TODO: We need a better solution to this. This hard-coding is not
    # an acceptable solution.
    dramsim_3_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.pardir,
        os.pardir,
        os.pardir,
        "ext",
        "dramsim3",
        "DRAMsim3",
    )

    dramsim_3_mem_configs = os.path.join(dramsim_3_dir, "configs")

    input_file = os.path.join(dramsim_3_mem_configs, mem_type + ".ini")

    # Run checks to ensure the `ext/DRAMsim3` directory is present, contains
    # the configs directory, and the configuration file we require.
    if not os.path.isdir(dramsim_3_dir):
        raise Exception(
            "The `ext/DRAMsim3` directory cannot be found.\n"
            "Please navigate to `ext` and run:\n"
            "git clone git@github.com:umd-memsys/DRAMsim3.git"
        )
    elif not os.path.isdir(dramsim_3_mem_configs):
        raise Exception(
            "The `ext/DRAMsim3/configs` directory cannot be found."
        )
    elif not os.path.isfile(input_file):
        raise Exception(
            "The configuration file '" + input_file + "' cannot  be found."
        )

    output_file = "/tmp/" + mem_type + "_chnls" + str(num_chnls) + ".ini"
    new_config = open(output_file, "w")
    config.read(input_file)
    config.set("system", "channels", str(num_chnls))
    config.write(new_config)
    new_config.close()
    return output_file, m5.options.outdir


class DRAMSim3MemCtrl(DRAMsim3):
    """
    A DRAMSim3 Memory Controller.

    The class serves as a SimObject object wrapper, utiliszing the DRAMSim3
    configuratons.
    """

    def __init__(self, mem_name: str, num_chnls: int) -> None:
        """
        :param mem_name: The name of the type  of memory to be configured.
        :param num_chnls: The number of channels.
        """
        super().__init__()
        ini_path, outdir = config_ds3(mem_name, num_chnls)
        self.configFile = ini_path
        self.filePath = outdir


class SingleChannel(AbstractMemorySystem):
    """
    A Single Channel Memory system.
    """

    def __init__(self, mem_type: str, size: Optional[str]):
        """
        :param mem_name: The name of the type  of memory to be configured.
        :param num_chnls: The number of channels.
        """
        super().__init__()
        self.mem_ctrl = DRAMSim3MemCtrl(mem_type, 1)
        self._size = toMemorySize(size)
        if not size:
            raise NotImplementedError(
                "DRAMSim3 memory controller requires a size parameter."
            )

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Tuple[Sequence[AddrRange], Port]:
        return [(self.mem_ctrl.range, self.mem_ctrl.port)]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return [self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        if len(ranges != 1) or ranges[0].size != self._size:
            raise Exception(
                "Single channel DRAMSim memory controller requires a single "
                "range which matches the memory's size."
            )
        self.mem_ctrl.range = ranges[0]


def SingleChannelDDR3_1600(
    size: Optional[str] = "2048MiB",
) -> SingleChannel:
    """
    A single channel DDR3_1600.

    :param size: The size of the memory system. Default value of 2048MiB.
    """
    return SingleChannel("DDR3_8Gb_x8_1600", size)


def SingleChannelDDR4_2400(size: Optional[str] = "1024MiB") -> SingleChannel:
    """
    A single channel DDR3_2400.

    :param size: The size of the memory system. Default value of 1024MiB.
    """
    return SingleChannel("DDR4_4Gb_x8_2400", size)


def SingleChannelLPDDR3_1600(size: Optional[str] = "256MiB") -> SingleChannel:
    """
    A single channel LPDDR3_1600.

    :param size: The size of the memory system. Default value of 256MiB.
    """
    return SingleChannel("LPDDR3_8Gb_x32_1600", size)


def SingleChannelHBM(size: Optional[str] = "64MiB") -> SingleChannel:
    """
    A single channel HBM.

    :param size: The size of the memory system. Default value of 64MiB.
    """
    return SingleChannel("HBM1_4Gb_x128", size)
