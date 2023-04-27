# Copyright (c) 2022 The Regents of the University of California
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

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

# The following lists the RISCV binaries. Those commented out presently result
# in a test failure. This is outlined in the following Jira issue:
# https://gem5.atlassian.net/browse/GEM5-496
binaries = (
    "rv64samt-ps-sysclone_d",
    "rv64samt-ps-sysfutex1_d",
    #    'rv64samt-ps-sysfutex2_d',
    "rv64samt-ps-sysfutex3_d",
    #    'rv64samt-ps-sysfutex_d',
    "rv64ua-ps-amoadd_d",
    "rv64ua-ps-amoadd_w",
    "rv64ua-ps-amoand_d",
    "rv64ua-ps-amoand_w",
    "rv64ua-ps-amomax_d",
    "rv64ua-ps-amomax_w",
    "rv64ua-ps-amomaxu_d",
    "rv64ua-ps-amomaxu_w",
    "rv64ua-ps-amomin_d",
    "rv64ua-ps-amomin_w",
    "rv64ua-ps-amominu_d",
    "rv64ua-ps-amominu_w",
    "rv64ua-ps-amoor_d",
    "rv64ua-ps-amoor_w",
    "rv64ua-ps-amoswap_d",
    "rv64ua-ps-amoswap_w",
    "rv64ua-ps-amoxor_d",
    "rv64ua-ps-amoxor_w",
    "rv64ua-ps-lrsc",
    "rv64uamt-ps-amoadd_d",
    "rv64uamt-ps-amoand_d",
    "rv64uamt-ps-amomax_d",
    "rv64uamt-ps-amomaxu_d",
    "rv64uamt-ps-amomin_d",
    "rv64uamt-ps-amominu_d",
    "rv64uamt-ps-amoor_d",
    "rv64uamt-ps-amoswap_d",
    "rv64uamt-ps-amoxor_d",
    "rv64uamt-ps-lrsc_d",
    "rv64ud-ps-fadd",
    "rv64ud-ps-fclass",
    "rv64ud-ps-fcmp",
    "rv64ud-ps-fcvt",
    "rv64ud-ps-fcvt_w",
    "rv64ud-ps-fdiv",
    "rv64ud-ps-fmadd",
    "rv64ud-ps-fmin",
    "rv64ud-ps-ldst",
    "rv64ud-ps-move",
    "rv64ud-ps-recoding",
    "rv64ud-ps-structural",
    "rv64uf-ps-fadd",
    "rv64uf-ps-fclass",
    "rv64uf-ps-fcmp",
    "rv64uf-ps-fcvt",
    "rv64uf-ps-fcvt_w",
    "rv64uf-ps-fdiv",
    "rv64uf-ps-fmadd",
    "rv64uf-ps-fmin",
    "rv64uf-ps-ldst",
    "rv64uf-ps-move",
    "rv64uf-ps-recoding",
    "rv64ui-ps-add",
    "rv64ui-ps-addi",
    "rv64ui-ps-addiw",
    "rv64ui-ps-addw",
    "rv64ui-ps-and",
    "rv64ui-ps-andi",
    "rv64ui-ps-auipc",
    "rv64ui-ps-beq",
    "rv64ui-ps-bge",
    "rv64ui-ps-bgeu",
    "rv64ui-ps-blt",
    "rv64ui-ps-bltu",
    "rv64ui-ps-bne",
    "rv64ui-ps-fence_i",
    "rv64ui-ps-jal",
    "rv64ui-ps-jalr",
    "rv64ui-ps-lb",
    "rv64ui-ps-lbu",
    "rv64ui-ps-ld",
    "rv64ui-ps-lh",
    "rv64ui-ps-lhu",
    "rv64ui-ps-lui",
    "rv64ui-ps-lw",
    "rv64ui-ps-lwu",
    "rv64ui-ps-or",
    "rv64ui-ps-ori",
    "rv64ui-ps-sb",
    "rv64ui-ps-sd",
    "rv64ui-ps-sh",
    "rv64ui-ps-simple",
    "rv64ui-ps-sll",
    "rv64ui-ps-slli",
    "rv64ui-ps-slliw",
    "rv64ui-ps-sllw",
    "rv64ui-ps-slt",
    "rv64ui-ps-slti",
    "rv64ui-ps-sltiu",
    "rv64ui-ps-sltu",
    "rv64ui-ps-sra",
    "rv64ui-ps-srai",
    "rv64ui-ps-sraiw",
    "rv64ui-ps-sraw",
    "rv64ui-ps-srl",
    "rv64ui-ps-srli",
    "rv64ui-ps-srliw",
    "rv64ui-ps-srlw",
    "rv64ui-ps-sub",
    "rv64ui-ps-subw",
    "rv64ui-ps-sw",
    "rv64ui-ps-xor",
    "rv64ui-ps-xori",
    "rv64um-ps-div",
    "rv64um-ps-divu",
    "rv64um-ps-divuw",
    "rv64um-ps-divw",
    "rv64um-ps-mul",
    "rv64um-ps-mulh",
    "rv64um-ps-mulhsu",
    "rv64um-ps-mulhu",
    "rv64um-ps-mulw",
    "rv64um-ps-rem",
    "rv64um-ps-remu",
    "rv64um-ps-remuw",
    "rv64um-ps-remw",
    "rv64uzfh-ps-fadd",
    "rv64uzfh-ps-fclass",
    "rv64uzfh-ps-fcmp",
    "rv64uzfh-ps-fcvt",
    "rv64uzfh-ps-fcvt_w",
    "rv64uzfh-ps-fdiv",
    "rv64uzfh-ps-fmadd",
    "rv64uzfh-ps-fmin",
    "rv64uzfh-ps-ldst",
    "rv64uzfh-ps-move",
    "rv64uzfh-ps-recoding",
)

cpu_types = ("atomic", "timing", "minor", "o3")

for cpu_type in cpu_types:
    for binary in binaries:
        gem5_verify_config(
            name=f"asm-riscv-{binary}-{cpu_type}",
            verifiers=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "configs",
                "simple_binary_run.py",
            ),
            config_args=[
                binary,
                cpu_type,
                "riscv",
                "--num-cores",
                "4",
                "--resource-directory",
                resource_path,
            ],
            valid_isas=(constants.all_compiled_tag,),
            valid_hosts=constants.supported_hosts,
        )
