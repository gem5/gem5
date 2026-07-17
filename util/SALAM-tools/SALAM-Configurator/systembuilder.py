#!/usr/bin/env python3

# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

import argparse
import os
import shutil
from http.client import FOUND
from unittest import expectedFailure

import config_parser
import yaml

# Define the imports of the gem5 script
imports = """import m5
from m5.objects import *
from m5.util import *

from configparser import ConfigParser
from HWAccConfig import *

"""

# L1 Cache defined here for now, need to add some more configurability to this
l1Cache = """class L1Cache(Cache):
\tassoc = 2
\ttag_latency = 2
\tdata_latency = 2
\tresponse_latency = 2
\tmshrs = 4
\ttgts_per_mshr = 20\n\t
def __init__(self, size, options=None):
\t\tself.size = size
\t\tsuper(L1Cache, self).__init__()
\t\tpass\n\n"""


def parse_cur_args():
    argparser = argparse.ArgumentParser(description="SALAM System Builder")
    argparser.add_argument(
        "--bench-path",
        help="Path to the benchmark directory relative to ACC_BENCH_PATH "
        "(e.g. bfs → $ACC_BENCH_PATH/bfs)",
        required=True,
    )
    argparser.add_argument(
        "--sys-name",
        help="Name of the generated python files. For a sys_name of gemm, the"
        " configurator generates both fs_gemm.py & gemm.py in configs/SALAM).",
        required=True,
        default=None,
    )
    argparser.add_argument(
        "--config-name",
        help="Name of the configuration file in the root of the bench."
        " Defaults to config.yml",
        required=False,
        default="config.yml",
    )
    argparser.add_argument(
        "--m5-path", help="Path to M5 Directory", required=False, default=None
    )
    return argparser.parse_args()


def parse_yaml(
    parent_config,
    base_address,
    working_dir: str,
    parent_path: str = None,
    hw_path: str = None,
):
    clusters = []
    # Load in each acc cluster and add it to the list
    for cluster_dict in parent_config:
        for list_type, params in cluster_dict.items():
            FOUND_SYS_PATH = False
            FOUND_HW_PATH = False
            FOUND_DEVICE = False
            if list_type == "acc_cluster":
                for param in params:
                    if "SysPath" in param:
                        FOUND_SYS_PATH = True
                        cur_config = open_yaml(
                            yml_path=(working_dir + param["SysPath"])
                        )
                        cur_path = working_dir + param["SysPath"]
                    elif "HWPath" in param:
                        FOUND_HW_PATH = True
                        hw_path = working_dir + param["HWPath"]
                    else:
                        FOUND_DEVICE = True
                        cur_config = parent_config
                        # Use the parent YAML path as the hardware profile
                        # path unless this cluster overrides it with HWPath.
                        if not FOUND_HW_PATH and hw_path is None:
                            hw_path = parent_path
            if FOUND_SYS_PATH and FOUND_DEVICE:
                raise Exception(
                    "Found device definitions in a cluster with a"
                    " path to another YAML file."
                )
            if FOUND_SYS_PATH:
                # Recursion Alert!
                base_address, temp_cluster = parse_yaml(
                    parent_config=cur_config,
                    base_address=base_address,
                    working_dir=working_dir,
                    parent_path=cur_path,
                    hw_path=hw_path,
                )
                clusters.extend(temp_cluster)
            elif FOUND_HW_PATH:
                raise Exception(
                    "HW Path should be defined with a System Config file"
                )

        cluster_name = None
        dmas = []
        accs = []
        for list_type, devices in cluster_dict.items():
            if list_type == "acc_cluster":
                for device in devices:
                    if "Name" in device:
                        cluster_name = device["Name"]
                    if "DMA" in device:
                        dmas.append(device)
                    if "Accelerator" in device:
                        accs.append(device)
        if cluster_name is None:
            continue
        clusters.append(
            config_parser.AccCluster(
                name=cluster_name,
                dmas=dmas,
                accs=accs,
                base_address=base_address,
                working_dir=working_dir,
                config_path=parent_path,
                hw_config_path=hw_path,
            )
        )
        base_address = clusters[-1].top_address + (
            64 - (int(clusters[-1].top_address) % 64)
        )
        if (int(base_address) % 64) != 0:
            print("Address Alignment Error: " + hex(base_address))
    return base_address, clusters


def open_yaml(yml_path: str):
    stream = open(yml_path)
    config = yaml.safe_load_all(stream)
    return config


def gen_config(clusters, config_path: str, file_name: str):
    # Write out config file
    with open(config_path + file_name + ".py", "w") as writer:
        writer.write(imports)
        writer.write(l1Cache)
        for cluster in clusters:
            for line in cluster.genConfig():
                writer.write(line + "\n")
            # Add cluster definitions here
            for dma in cluster.dmas:
                writeLines(writer, dma.genConfig())
            # Come back here later and see if you can get away with doing
            # these at the same time (probably not)
            for acc in cluster.accs:
                writeLines(writer, acc.genDefinition())
            for acc in cluster.accs:
                writeLines(writer, acc.genConfig())
        # Write cluster creation
        writer.write("def makeHWAcc(args, system):\n\n")
        for i in clusters:
            writer.write(
                "	system." + i.name.lower() + " = AccCluster()" + "\n"
            )
            writer.write(
                "	build"
                + i.name
                + "(args, system, system."
                + i.name.lower()
                + ")\n\n"
            )


def load_og_header(clusters, working_dir: str):
    begin = None
    end = None
    # Read in existing header
    header_list = []
    for i in clusters:
        try:
            # f = open(working_dir + i.name  + "_" + args.headerName +
            #     ".h", 'r')
            f = open(working_dir + i.name + "_hw_defines.h")
            oldHeader = f.readlines()
            for i in range(0, len(oldHeader)):
                if oldHeader[i] == "//BEGIN GENERATED CODE\n":
                    begin = i
                elif (
                    oldHeader[i] == "//END GENERATED CODE\n"
                    or oldHeader[i] == "//END GENERATED CODE"
                ):
                    end = i
            del oldHeader[begin : end + 1]
            header_list.append(oldHeader)
        except:
            print("No Header Found")
            emptyList = []
            header_list.append(emptyList)
    return header_list


def gen_header(header_list, clusters, working_dir: str):
    # Write out headers
    for current_header in header_list:
        for cluster in clusters:
            with open(working_dir + cluster.name + "_hw_defines.h", "w") as f:
                current_header.append("//BEGIN GENERATED CODE\n")
                current_header.append(
                    "//Cluster: " + cluster.name.upper() + "\n"
                )
                for dma in cluster.dmas:
                    if dma.dmaType == "NonCoherent":
                        current_header.append(
                            "//" + dma.dmaType + "DMA" + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Flags "
                            + hex(dma.address)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdAddr "
                            + hex(dma.address + 1)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrAddr "
                            + hex(dma.address + 9)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_CopyLen "
                            + hex(dma.address + 17)
                            + "\n"
                        )
                    elif dma.dmaType == "Stream":
                        current_header.append(
                            "//" + dma.dmaType + "DMA" + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Flags "
                            + hex(dma.address)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdAddr "
                            + hex(dma.address + 4)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrAddr "
                            + hex(dma.address + 12)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdFrameSize "
                            + hex(dma.address + 20)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_NumRdFrames "
                            + hex(dma.address + 24)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_RdFrameBufSize "
                            + hex(dma.address + 25)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrFrameSize "
                            + hex(dma.address + 26)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_NumWrFrames "
                            + hex(dma.address + 30)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_WrFrameBufSize "
                            + hex(dma.address + 31)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Stream "
                            + hex(dma.address + 32)
                            + "\n"
                        )
                        current_header.append(
                            "#define "
                            + dma.name.upper()
                            + "_Status "
                            + hex(dma.statusAddress)
                            + "\n"
                        )
                for acc in cluster.accs:
                    current_header.append(
                        "//Accelerator: " + acc.name.upper() + "\n"
                    )
                    current_header.append(
                        "#define "
                        + acc.name.upper()
                        + " "
                        + hex(acc.address)
                        + "\n"
                    )
                    for var in acc.variables:
                        if "Cache" in var.type:
                            continue
                        elif "Stream" in var.type:
                            current_header.append(
                                "#define "
                                + var.name
                                + " "
                                + hex(var.address)
                                + "\n"
                            )
                            current_header.append(
                                "#define "
                                + var.name
                                + "_Status "
                                + hex(var.statusAddress)
                                + "\n"
                            )
                        else:
                            current_header.append(
                                "#define "
                                + var.name
                                + " "
                                + hex(var.address)
                                + "\n"
                            )
                current_header.append("//END GENERATED CODE")
                f.writelines(current_header)
                current_header = []


def writeLines(writer, lines):
    for line in lines:
        writer.write("	" + line + "\n")


def main():

    args = parse_cur_args()
    # This requires M5_PATH to point to your gem5-SALAM directory
    M5_Path = os.getenv("M5_PATH")
    acc_bench_path = os.getenv("ACC_BENCH_PATH")

    if M5_Path is None:
        print("Looking for Path Argument from Command Line")
        if args.m5_path is None:
            raise Exception("Path argument required when M5_PATH not set")
        M5_Path = args.path
        if M5_Path is None:
            raise Exception("M5_PATH Not Found")

    if acc_bench_path is None:
        print("Looking for Path Argument from Command Line")
        if args.m5_path is None:
            raise Exception(
                "Path argument required when ACC_BENCH_PATH not set"
            )
        acc_bench_path = args.path
        if acc_bench_path is None:
            raise Exception("ACC_BENCH_PATH Not Found")

    # Set file information
    if args.sys_name == None:
        file_name = os.path.basename(os.path.normpath(args.sys_path))
    else:
        file_name = args.sys_name
    config_path = M5_Path + "/configs/SALAM/"
    working_dir = acc_bench_path + "/" + args.bench_path + "/"
    main_yml_path = working_dir + args.config_name

    # Set base addresses
    base_address = 0x2F000000  # 0x10020000
    max_address = 0x2FFFFFFF  # 0x13FFFFFF
    # Load in the YAML file
    config = open_yaml(yml_path=main_yml_path)
    # Parse YAML File
    base_address, clusters = parse_yaml(
        parent_config=config,
        base_address=base_address,
        working_dir=working_dir,
        parent_path=main_yml_path,
    )
    # Generate SALAM Config
    gen_config(clusters=clusters, config_path=config_path, file_name=file_name)
    # Parse original header for custom code
    header_list = load_og_header(clusters=clusters, working_dir=working_dir)
    # Make the header files with custom code
    gen_header(
        header_list=header_list, clusters=clusters, working_dir=working_dir
    )
    # Generate full system file
    shutil.copyfile(
        M5_Path + "/util/SALAM-tools/SALAM-Configurator/fs_template.py",
        config_path + "fs_" + file_name + ".py",
    )
    f = open(config_path + "fs_" + file_name + ".py")
    fullSystem = f.readlines()

    for i, ln in enumerate(fullSystem):
        if ln.strip() == "import TEMPLATE":
            fullSystem[i] = f"import {file_name}\n"
        elif "TEMPLATE.makeHWAcc(" in ln:
            fullSystem[i] = ln.replace("TEMPLATE.", f"{file_name}.")

    f = open(config_path + "fs_" + file_name + ".py", "w")
    f.writelines(fullSystem)
    # Warn if the size is greater than allowed
    if clusters[-1].top_address > max_address:
        print("WARNING: Address range is greater than defined for gem5")


if __name__ == "__main__":
    main()
