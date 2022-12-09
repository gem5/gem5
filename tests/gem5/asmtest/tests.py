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
binary_configs = (
    ("rv{}samt-ps-sysclone_d", (64,)),
    ("rv{}samt-ps-sysfutex1_d", (64,)),
    #    'rv64samt-ps-sysfutex2_d',
    ("rv{}samt-ps-sysfutex3_d", (64,)),
    #    'rv64samt-ps-sysfutex_d',
    ("rv{}ua-ps-amoadd_d", (64,)),
    ("rv{}ua-ps-amoadd_w", (32, 64)),
    ("rv{}ua-ps-amoand_d", (64,)),
    ("rv{}ua-ps-amoand_w", (32, 64)),
    ("rv{}ua-ps-amomax_d", (64,)),
    ("rv{}ua-ps-amomax_w", (32, 64)),
    ("rv{}ua-ps-amomaxu_d", (64,)),
    ("rv{}ua-ps-amomaxu_w", (32, 64)),
    ("rv{}ua-ps-amomin_d", (64,)),
    ("rv{}ua-ps-amomin_w", (32, 64)),
    ("rv{}ua-ps-amominu_d", (64,)),
    ("rv{}ua-ps-amominu_w", (32, 64)),
    ("rv{}ua-ps-amoor_d", (64,)),
    ("rv{}ua-ps-amoor_w", (32, 64)),
    ("rv{}ua-ps-amoswap_d", (64,)),
    ("rv{}ua-ps-amoswap_w", (32, 64)),
    ("rv{}ua-ps-amoxor_d", (64,)),
    ("rv{}ua-ps-amoxor_w", (32, 64)),
    ("rv{}ua-ps-lrsc", (32, 64)),
    ("rv{}uamt-ps-amoadd_d", (64,)),
    ("rv{}uamt-ps-amoand_d", (64,)),
    ("rv{}uamt-ps-amomax_d", (64,)),
    ("rv{}uamt-ps-amomaxu_d", (64,)),
    ("rv{}uamt-ps-amomin_d", (64,)),
    ("rv{}uamt-ps-amominu_d", (64,)),
    ("rv{}uamt-ps-amoor_d", (64,)),
    ("rv{}uamt-ps-amoswap_d", (64,)),
    ("rv{}uamt-ps-amoxor_d", (64,)),
    ("rv{}uamt-ps-lrsc_d", (64,)),
    ("rv{}uamt-ps-amoadd_w", (32,)),
    ("rv{}uamt-ps-amoand_w", (32,)),
    ("rv{}uamt-ps-amomax_w", (32,)),
    ("rv{}uamt-ps-amomaxu_w", (32,)),
    ("rv{}uamt-ps-amomin_w", (32,)),
    ("rv{}uamt-ps-amominu_w", (32,)),
    ("rv{}uamt-ps-amoor_w", (32,)),
    ("rv{}uamt-ps-amoswap_w", (32,)),
    ("rv{}uamt-ps-amoxor_w", (32,)),
    ("rv{}uamt-ps-lrsc_w", (32,)),
    ("rv{}ud-ps-fadd", (32, 64)),
    ("rv{}ud-ps-fclass", (32, 64)),
    ("rv{}ud-ps-fcmp", (32, 64)),
    ("rv{}ud-ps-fcvt", (32, 64)),
    ("rv{}ud-ps-fcvt_w", (32, 64)),
    ("rv{}ud-ps-fdiv", (32, 64)),
    ("rv{}ud-ps-fmadd", (32, 64)),
    ("rv{}ud-ps-fmin", (32, 64)),
    ("rv{}ud-ps-ldst", (32, 64)),
    ("rv{}ud-ps-move", (64,)),
    ("rv{}ud-ps-recoding", (32, 64)),
    ("rv{}ud-ps-structural", (64,)),
    ("rv{}uf-ps-fadd", (32, 64)),
    ("rv{}uf-ps-fclass", (32, 64)),
    ("rv{}uf-ps-fcmp", (32, 64)),
    ("rv{}uf-ps-fcvt", (32, 64)),
    ("rv{}uf-ps-fcvt_w", (32, 64)),
    ("rv{}uf-ps-fdiv", (32, 64)),
    ("rv{}uf-ps-fmadd", (32, 64)),
    ("rv{}uf-ps-fmin", (32, 64)),
    ("rv{}uf-ps-ldst", (32, 64)),
    ("rv{}uf-ps-move", (32, 64)),
    ("rv{}uf-ps-recoding", (32, 64)),
    ("rv{}ui-ps-add", (32, 64)),
    ("rv{}ui-ps-addi", (32, 64)),
    ("rv{}ui-ps-addiw", (64,)),
    ("rv{}ui-ps-addw", (64,)),
    ("rv{}ui-ps-and", (32, 64)),
    ("rv{}ui-ps-andi", (32, 64)),
    ("rv{}ui-ps-auipc", (32, 64)),
    ("rv{}ui-ps-beq", (32, 64)),
    ("rv{}ui-ps-bge", (32, 64)),
    ("rv{}ui-ps-bgeu", (32, 64)),
    ("rv{}ui-ps-blt", (32, 64)),
    ("rv{}ui-ps-bltu", (32, 64)),
    ("rv{}ui-ps-bne", (32, 64)),
    ("rv{}ui-ps-fence_i", (32, 64)),
    ("rv{}ui-ps-jal", (32, 64)),
    ("rv{}ui-ps-jalr", (32, 64)),
    ("rv{}ui-ps-lb", (32, 64)),
    ("rv{}ui-ps-lbu", (32, 64)),
    ("rv{}ui-ps-ld", (64,)),
    ("rv{}ui-ps-lh", (32, 64)),
    ("rv{}ui-ps-lhu", (32, 64)),
    ("rv{}ui-ps-lui", (32, 64)),
    ("rv{}ui-ps-lw", (32, 64)),
    ("rv{}ui-ps-lwu", (64,)),
    ("rv{}ui-ps-or", (32, 64)),
    ("rv{}ui-ps-ori", (32, 64)),
    ("rv{}ui-ps-sb", (32, 64)),
    ("rv{}ui-ps-sd", (64,)),
    ("rv{}ui-ps-sh", (32, 64)),
    ("rv{}ui-ps-simple", (32, 64)),
    ("rv{}ui-ps-sll", (32, 64)),
    ("rv{}ui-ps-slli", (32, 64)),
    ("rv{}ui-ps-slliw", (64,)),
    ("rv{}ui-ps-sllw", (64,)),
    ("rv{}ui-ps-slt", (32, 64)),
    ("rv{}ui-ps-slti", (32, 64)),
    ("rv{}ui-ps-sltiu", (32, 64)),
    ("rv{}ui-ps-sltu", (32, 64)),
    ("rv{}ui-ps-sra", (32, 64)),
    ("rv{}ui-ps-srai", (32, 64)),
    ("rv{}ui-ps-sraiw", (64,)),
    ("rv{}ui-ps-sraw", (64,)),
    ("rv{}ui-ps-srl", (32, 64)),
    ("rv{}ui-ps-srli", (32, 64)),
    ("rv{}ui-ps-srliw", (64,)),
    ("rv{}ui-ps-srlw", (64,)),
    ("rv{}ui-ps-sub", (32, 64)),
    ("rv{}ui-ps-subw", (64,)),
    ("rv{}ui-ps-sw", (32, 64)),
    ("rv{}ui-ps-xor", (32, 64)),
    ("rv{}ui-ps-xori", (32, 64)),
    ("rv{}um-ps-div", (32, 64)),
    ("rv{}um-ps-divu", (32, 64)),
    ("rv{}um-ps-divuw", (64,)),
    ("rv{}um-ps-divw", (64,)),
    ("rv{}um-ps-mul", (32, 64)),
    ("rv{}um-ps-mulh", (32, 64)),
    ("rv{}um-ps-mulhsu", (32, 64)),
    ("rv{}um-ps-mulhu", (32, 64)),
    ("rv{}um-ps-mulw", (64,)),
    ("rv{}um-ps-rem", (32, 64)),
    ("rv{}um-ps-remu", (32, 64)),
    ("rv{}um-ps-remuw", (64,)),
    ("rv{}um-ps-remw", (64,)),
    ("rv{}uzfh-ps-fadd", (32, 64)),
    ("rv{}uzfh-ps-fclass", (32, 64)),
    ("rv{}uzfh-ps-fcmp", (32, 64)),
    ("rv{}uzfh-ps-fcvt", (32, 64)),
    ("rv{}uzfh-ps-fcvt_w", (32, 64)),
    ("rv{}uzfh-ps-fdiv", (32, 64)),
    ("rv{}uzfh-ps-fmadd", (32, 64)),
    ("rv{}uzfh-ps-fmin", (32, 64)),
    ("rv{}uzfh-ps-ldst", (32, 64)),
    ("rv{}uzfh-ps-move", (32, 64)),
    ("rv{}uzfh-ps-recoding", (32, 64)),
)

cpu_types = ("atomic", "timing", "minor", "o3")

for cpu_type in cpu_types:
    for cfg in binary_configs:
        template_bin, all_bits = cfg
        for bits in all_bits:
            binary = template_bin.format(bits)
            config_args = [
                binary,
                cpu_type,
                "riscv",
                "--num-cores",
                "4",
                "--resource-directory",
                resource_path,
            ]
            if bits == 32:
                config_args.extend(["-b", "--riscv-32bits"])
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
                config_args=config_args,
                valid_isas=(constants.all_compiled_tag,),
                valid_hosts=constants.supported_hosts,
            )
