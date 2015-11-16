#!/usr/bin/python

#
# A Python script to generate McPAT XML and result files for the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from optparse import OptionParser
import collections
import os
import json
from objectpath import *
from pyparsing import Word, Optional, ParseException, printables, nums, restOfLine
from lxml import etree
from yattag import Doc

parser = OptionParser()
parser.add_option('--config', type='string', default='m5out/config.json',
                  help='GEM5\'s system configuration JSON file name')
parser.add_option('--stats', type='string', default='m5out/stats.txt', help='GEM5\'s output statistics file name')
parser.add_option('--mcpat_in', type='string', default='m5out/mcpat_in.xml', help='McPAT\'s input XML file name')
parser.add_option('--mcpat_out', type='string', default='m5out/mcpat_out.txt', help='GEM5\'s output file name')

(option, arg) = parser.parse_args()

config_json_file_name = option.config
stats_file_name = option.stats
mcpat_in_xml_file_name = option.mcpat_in
mcpat_out_file_name = option.mcpat_out

with open(config_json_file_name) as config_json_file:
    config = json.load(config_json_file)

config_tree = Tree(config)

num_cpus = config_tree.execute('len($.system.cpu)')
num_l2caches = config_tree.execute('len($.system.l2cache)')

numa = config_tree.execute('len($.system.numa_caches_upward)') > 0

stat_rule = Word(printables) + Word('nan.%' + nums) + Optional(restOfLine)

stats = collections.OrderedDict()

with open(stats_file_name) as stats_file:
    for stat_line in stats_file:
        try:
            stat = stat_rule.parseString(stat_line)
            key = stat[0]
            value = stat[1]
            stats[key] = value
        except ParseException, err:
            pass


def cpu_id(i, l1=False):
    return config_tree.execute('$.system.' + ('switch_cpus' if not l1 and numa else 'cpu') + '[' + str(i) + '].name')


def l2_id(i=None):
    return 'l2' if i is None else config_tree.execute('$.system.l2cache[' + str(i) + '].name')


def gen_core(tag, i):
    def gen_predictor():
        with tag('component', id='system.core' + str(i) + '.predictor', name='PBT', type='BranchPredictor'):
            with tag('param', name='assoc', value='1'):
                pass
            with tag('param', name='nbanks', value='1'):
                pass
            with tag('param', name='local_l1_predictor_size', value='12'):
                pass
            with tag('param', name='local_l2_predictor_size', value='4'):
                pass
            with tag('param', name='local_predictor_entries', value='8192'):
                pass
            with tag('param', name='global_predictor_entries', value='8192'):
                pass
            with tag('param', name='global_predictor_bits', value='4'):
                pass
            with tag('param', name='chooser_predictor_entries', value='8192'):
                pass
            with tag('param', name='chooser_predictor_bits', value='4'):
                pass

    def gen_itlb():
        with tag('component', id='system.core' + str(i) + '.itlb', name='itlb', type='InstructionTLB'):
            with tag('param', name='number_entries',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].itb.size')):
                pass
            with tag('param', name='latency', value='8'):  # TODO
                pass
            with tag('param', name='throughput', value='3'):
                pass
            with tag('param', name='assoc', value='0'):
                pass
            with tag('param', name='nbanks', value='1'):
                pass
            with tag('stat', name='total_accesses',
                     value=str(int(stats['system.' + cpu_id(i) + '.itb.fetch_accesses']))):
                pass
            with tag('stat', name='total_misses',
                     value=str(int(stats['system.' + cpu_id(i) + '.itb.fetch_misses']))):
                pass
            with tag('stat', name='conflicts', value='0'):  # TODO
                pass

    def gen_l1i():
        with tag('component', id='system.core' + str(i) + '.icache', name='Instruction Cache', type='CacheUnit'):
            with tag('param', name='level', value='1'):
                pass
            with tag('param', name='size', value=config_tree.execute('$.system.cpu[' + str(i) + '].icache.size')):
                pass
            with tag('param', name='block_size',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].icache.tags.block_size')):
                pass
            with tag('param', name='assoc', value=config_tree.execute('$.system.cpu[' + str(i) + '].icache.assoc')):
                pass
            with tag('param', name='num_banks', value='1'):
                pass
            with tag('param', name='latency',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].icache.hit_latency')):
                pass
            with tag('param', name='throughput', value='3'):
                pass
            with tag('param', name='miss_buffer_size', value='2'):
                pass
            with tag('param', name='fetch_buffer_size', value='2'):
                pass
            with tag('param', name='prefetch_buffer_size', value='2'):
                pass
            with tag('param', name='writeback_buffer_size', value='0'):
                pass
            with tag('param', name='clockrate', value='0'):
                pass
            with tag('param', name='tech_type', value='0'):
                pass
            with tag('param', name='Directory_type', value='2'):
                pass
            with tag('param', name='device_type', value='0'):
                pass
            with tag('param', name='core_type', value='1'):
                pass
            with tag('param', name='wire_mat_type', value='2'):
                pass
            with tag('param', name='wire_type', value='0'):
                pass
            with tag('param', name='miss_buffer_assoc', value='0'):
                pass
            with tag('param', name='fetch_buffer_assoc', value='0'):
                pass
            with tag('param', name='prefetch_buffer_assoc', value='0'):
                pass
            with tag('param', name='writeback_buffer_assoc', value='0'):
                pass
            with tag('param', name='miss_buffer_banks', value='1'):
                pass
            with tag('param', name='fetch_buffer_banks', value='1'):
                pass
            with tag('param', name='prefetch_buffer_banks', value='1'):
                pass
            with tag('param', name='writeback_buffer_banks', value='1'):
                pass
            with tag('param', name='cache_access_mode', value='0'):
                pass
            with tag('param', name='miss_buff_access_mode', value='2'):
                pass
            with tag('param', name='fetch_buff_access_mode', value='2'):
                pass
            with tag('param', name='prefetch_buff_access_mode', value='2'):
                pass
            with tag('param', name='writeback_buff_access_mode', value='2'):
                pass
            with tag('param', name='cache_rw_ports', value='1'):
                pass
            with tag('param', name='cache_rd_ports', value='0'):
                pass
            with tag('param', name='cache_wr_ports', value='0'):
                pass
            with tag('param', name='cache_se_rd_ports', value='0'):
                pass
            with tag('param', name='cache_search_ports', value='0'):
                pass
            with tag('param', name='miss_buff_rw_ports', value='1'):
                pass
            with tag('param', name='miss_buff_rd_ports', value='0'):
                pass
            with tag('param', name='miss_buff_wr_ports', value='0'):
                pass
            with tag('param', name='miss_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='miss_buff_search_ports', value='1'):
                pass
            with tag('param', name='fetch_buff_rw_ports', value='1'):
                pass
            with tag('param', name='fetch_buff_rd_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_wr_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_search_ports', value='1'):
                pass
            with tag('param', name='pf_buff_rw_ports', value='1'):
                pass
            with tag('param', name='pf_buff_rd_ports', value='0'):
                pass
            with tag('param', name='pf_buff_wr_ports', value='0'):
                pass
            with tag('param', name='pf_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='pf_buff_search_ports', value='1'):
                pass
            with tag('param', name='wb_buff_rw_ports', value='1'):
                pass
            with tag('param', name='wb_buff_rd_ports', value='0'):
                pass
            with tag('param', name='wb_buff_wr_ports', value='0'):
                pass
            with tag('param', name='wb_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='wb_buff_search_ports', value='1'):
                pass
            with tag('param', name='pure_ram', value='0'):
                pass
            with tag('stat', name='read_accesses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.icache.ReadReq_accesses::total']))):
                pass
            with tag('stat', name='read_misses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.icache.ReadReq_misses::total']))):
                pass
            with tag('stat', name='conflicts', value='0'):  # TODO
                pass
            with tag('stat', name='duty_cycle', value='0'):  # TODO
                pass

    def gen_dtlb():
        with tag('component', id='system.core' + str(i) + '.dtlb', name='dtlb', type='DataTLB'):
            with tag('param', name='number_entries',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].dtb.size')):
                pass
            with tag('param', name='latency', value='8'):  # TODO
                pass
            with tag('param', name='throughput', value='3'):
                pass
            with tag('param', name='assoc', value='0'):
                pass
            with tag('param', name='nbanks', value='1'):
                pass
            with tag('stat', name='read_accesses',
                     value=str(int(stats['system.' + cpu_id(i) + '.dtb.read_accesses']))):
                pass
            with tag('stat', name='read_misses', value=str(int(stats['system.' + cpu_id(i) + '.dtb.read_misses']))):
                pass
            with tag('stat', name='conflicts', value='0'):  # TODO
                pass

    def gen_l1d():
        with tag('component', id='system.core' + str(i) + '.dcache', name='Data Cache', type='CacheUnit'):
            with tag('param', name='level', value='1'):
                pass
            with tag('param', name='size', value=config_tree.execute('$.system.cpu[' + str(i) + '].dcache.size')):
                pass
            with tag('param', name='block_size',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].dcache.tags.block_size')):
                pass
            with tag('param', name='assoc', value=config_tree.execute('$.system.cpu[' + str(i) + '].dcache.assoc')):
                pass
            with tag('param', name='num_banks', value='1'):
                pass
            with tag('param', name='latency',
                     value=config_tree.execute('$.system.cpu[' + str(i) + '].dcache.hit_latency')):
                pass
            with tag('param', name='throughput', value='3'):
                pass
            with tag('param', name='miss_buffer_size', value='2'):
                pass
            with tag('param', name='fetch_buffer_size', value='2'):
                pass
            with tag('param', name='prefetch_buffer_size', value='2'):
                pass
            with tag('param', name='writeback_buffer_size', value='0'):
                pass
            with tag('param', name='clockrate', value='0'):
                pass
            with tag('param', name='tech_type', value='0'):
                pass
            with tag('param', name='Directory_type', value='2'):
                pass
            with tag('param', name='device_type', value='0'):
                pass
            with tag('param', name='core_type', value='1'):
                pass
            with tag('param', name='wire_mat_type', value='2'):
                pass
            with tag('param', name='wire_type', value='0'):
                pass
            with tag('param', name='miss_buffer_assoc', value='0'):
                pass
            with tag('param', name='fetch_buffer_assoc', value='0'):
                pass
            with tag('param', name='prefetch_buffer_assoc', value='0'):
                pass
            with tag('param', name='writeback_buffer_assoc', value='0'):
                pass
            with tag('param', name='miss_buffer_banks', value='1'):
                pass
            with tag('param', name='fetch_buffer_banks', value='1'):
                pass
            with tag('param', name='prefetch_buffer_banks', value='1'):
                pass
            with tag('param', name='writeback_buffer_banks', value='1'):
                pass
            with tag('param', name='cache_access_mode', value='0'):
                pass
            with tag('param', name='miss_buff_access_mode', value='2'):
                pass
            with tag('param', name='fetch_buff_access_mode', value='2'):
                pass
            with tag('param', name='prefetch_buff_access_mode', value='2'):
                pass
            with tag('param', name='writeback_buff_access_mode', value='2'):
                pass
            with tag('param', name='cache_rw_ports', value='1'):
                pass
            with tag('param', name='cache_rd_ports', value='0'):
                pass
            with tag('param', name='cache_wr_ports', value='0'):
                pass
            with tag('param', name='cache_se_rd_ports', value='0'):
                pass
            with tag('param', name='cache_search_ports', value='0'):
                pass
            with tag('param', name='miss_buff_rw_ports', value='1'):
                pass
            with tag('param', name='miss_buff_rd_ports', value='0'):
                pass
            with tag('param', name='miss_buff_wr_ports', value='0'):
                pass
            with tag('param', name='miss_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='miss_buff_search_ports', value='1'):
                pass
            with tag('param', name='fetch_buff_rw_ports', value='1'):
                pass
            with tag('param', name='fetch_buff_rd_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_wr_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='fetch_buff_search_ports', value='1'):
                pass
            with tag('param', name='pf_buff_rw_ports', value='1'):
                pass
            with tag('param', name='pf_buff_rd_ports', value='0'):
                pass
            with tag('param', name='pf_buff_wr_ports', value='0'):
                pass
            with tag('param', name='pf_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='pf_buff_search_ports', value='1'):
                pass
            with tag('param', name='wb_buff_rw_ports', value='1'):
                pass
            with tag('param', name='wb_buff_rd_ports', value='0'):
                pass
            with tag('param', name='wb_buff_wr_ports', value='0'):
                pass
            with tag('param', name='wb_buff_se_rd_ports', value='0'):
                pass
            with tag('param', name='wb_buff_search_ports', value='1'):
                pass
            with tag('param', name='pure_ram', value='0'):
                pass
            with tag('stat', name='read_accesses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.dcache.ReadReq_accesses::total']))):
                pass
            with tag('stat', name='write_accesses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.dcache.WriteReq_accesses::total']))):
                pass
            with tag('stat', name='read_misses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.dcache.ReadReq_misses::total']))):
                pass
            with tag('stat', name='write_misses',
                     value=str(int(stats['system.' + cpu_id(i, True) + '.dcache.WriteReq_misses::total']))):
                pass
            with tag('stat', name='conflicts', value='1'):  # TODO
                pass
            with tag('stat', name='duty_cycle', value='1'):  # TODO
                pass

    def gen_btargetbuf():
        with tag('component', id='system.core' + str(i) + '.btargetbuf', name='btargetbuf', type='BranchTargetBuffer'):
            with tag('param', name='size', value='8192'):
                pass
            with tag('param', name='block_size', value='4'):
                pass
            with tag('param', name='assoc', value='2'):
                pass
            with tag('param', name='num_banks', value='1'):
                pass
            with tag('param', name='latency', value='1'):
                pass
            with tag('param', name='throughput', value='3'):
                pass
            with tag('param', name='rw_ports', value='1'):
                pass
            with tag('stat', name='read_accesses', value='25'):  # TODO
                pass
            with tag('stat', name='write_accesses', value='25'):  # TODO
                pass

    with tag('component', id='system.core' + str(i), name='core' + str(i), type='Core'):
        with tag('param', name='clock_rate', value=str(int(stats['sim_freq']) / 1000 / 10 ** 6)):
            pass
        with tag('param', name='opt_local', value='0'):
            pass
        with tag('param', name='instruction_length', value='32'):
            pass
        with tag('param', name='opcode_width', value='7'):
            pass
        with tag('param', name='x86', value='0'):
            pass
        with tag('param', name='micro_opcode_width', value='8'):
            pass
        with tag('param', name='machine_type', value='1'):
            pass
        with tag('param', name='number_hardware_threads',
                 value=config_tree.execute('$.system.cpu[' + str(i) + '].numThreads')):
            pass
        with tag('param', name='fetch_width', value='1'):
            pass
        with tag('param', name='number_instruction_fetch_ports', value='1'):
            pass
        with tag('param', name='decode_width', value='1'):
            pass
        with tag('param', name='issue_width', value='1'):
            pass
        with tag('param', name='peak_issue_width', value='1'):
            pass
        with tag('param', name='commit_width', value='1'):
            pass
        with tag('param', name='fp_issue_width', value='1'):
            pass
        with tag('param', name='prediction_width', value='0'):
            pass
        with tag('param', name='int_pipelines', value='2'):
            pass
        with tag('param', name='fp_pipelines', value='1'):
            pass
        with tag('param', name='int_pipeline_depth', value='12'):
            pass
        with tag('param', name='fp_pipeline_depth', value='13'):
            pass
        with tag('param', name='ALU_per_core', value='2'):
            pass
        with tag('param', name='MUL_per_core', value='1'):
            pass
        with tag('param', name='FPU_per_core', value='1'):
            pass
        with tag('param', name='instruction_buffer_size', value='32'):
            pass
        with tag('param', name='instruction_window_scheme', value='0'):
            pass
        with tag('param', name='instruction_window_size', value='7'):
            pass
        with tag('param', name='fp_instruction_window_size', value='18'):
            pass
        with tag('param', name='ROB_size', value='56'):
            pass
        with tag('param', name='archi_Regs_IRF_size', value='30'):
            pass
        with tag('param', name='archi_Regs_FRF_size', value='48'):
            pass
        with tag('param', name='phy_Regs_IRF_size', value='34'):
            pass
        with tag('param', name='phy_Regs_FRF_size', value='40'):
            pass
        with tag('param', name='rename_scheme', value='0'):
            pass
        with tag('param', name='register_window_size', value='0'):
            pass
        with tag('param', name='register_window_throughput', value='4'):
            pass
        with tag('param', name='register_window_latency', value='4'):
            pass
        with tag('param', name='store_buffer_size', value='32'):
            pass
        with tag('param', name='load_buffer_size', value='32'):
            pass
        with tag('param', name='memory_ports', value='1'):
            pass
        with tag('param', name='RAS_size', value='16'):
            pass
        with tag('param', name='execu_wire_mat_type', value='2'):
            pass
        with tag('param', name='execu_bypass_base_width', value='1'):
            pass
        with tag('param', name='execu_bypass_base_height', value='1'):
            pass
        with tag('param', name='execu_bypass_start_wiring_level', value='3'):
            pass
        with tag('param', name='execu_bypass_route_over_perc', value='1'):
            pass
        with tag('param', name='globalCheckpoint', value='32'):
            pass
        with tag('param', name='perThreadState', value='8'):
            pass
        with tag('param', name='ROB_assoc', value='1'):
            pass
        with tag('param', name='ROB_nbanks', value='1'):
            pass
        with tag('param', name='ROB_tag_width', value='0'):
            pass
        with tag('param', name='scheduler_assoc', value='0'):
            pass
        with tag('param', name='scheduler_nbanks', value='1'):
            pass
        with tag('param', name='register_window_assoc', value='1'):
            pass
        with tag('param', name='register_window_nbanks', value='1'):
            pass
        with tag('param', name='register_window_tag_width', value='0'):
            pass
        with tag('param', name='register_window_rw_ports', value='1'):
            pass
        with tag('param', name='phy_Regs_IRF_assoc', value='1'):
            pass
        with tag('param', name='phy_Regs_IRF_nbanks', value='1'):
            pass
        with tag('param', name='phy_Regs_IRF_tag_width', value='0'):
            pass
        with tag('param', name='phy_Regs_IRF_rd_ports', value='1'):
            pass
        with tag('param', name='phy_Regs_IRF_wr_ports', value='1'):
            pass
        with tag('param', name='phy_Regs_FRF_assoc', value='1'):
            pass
        with tag('param', name='phy_Regs_FRF_nbanks', value='1'):
            pass
        with tag('param', name='phy_Regs_FRF_tag_width', value='0'):
            pass
        with tag('param', name='phy_Regs_FRF_rd_ports', value='1'):
            pass
        with tag('param', name='phy_Regs_FRF_wr_ports', value='1'):
            pass
        with tag('param', name='front_rat_nbanks', value='1'):
            pass
        with tag('param', name='front_rat_rw_ports', value='1'):
            pass
        with tag('param', name='retire_rat_nbanks', value='1'):
            pass
        with tag('param', name='retire_rat_rw_ports', value='0'):
            pass
        with tag('param', name='freelist_nbanks', value='1'):
            pass
        with tag('param', name='freelist_rw_ports', value='1'):
            pass
        with tag('param', name='load_buffer_assoc', value='0'):
            pass
        with tag('param', name='load_buffer_nbanks', value='1'):
            pass
        with tag('param', name='store_buffer_assoc', value='0'):
            pass
        with tag('param', name='store_buffer_nbanks', value='1'):
            pass
        with tag('param', name='instruction_buffer_assoc', value='1'):
            pass
        with tag('param', name='instruction_buffer_nbanks', value='1'):
            pass
        with tag('param', name='instruction_buffer_tag_width', value='0'):
            pass
        with tag('stat', name='total_instructions', value=str(
                        int(stats['system.' + cpu_id(i) + '.num_int_insts']) + int(
                    stats['system.' + cpu_id(i) + '.num_fp_insts']))):
            pass
        with tag('stat', name='int_instructions', value=str(int(stats['system.' + cpu_id(i) + '.num_int_insts']))):
            pass
        with tag('stat', name='fp_instructions', value=str(int(stats['system.' + cpu_id(i) + '.num_fp_insts']))):
            pass
        with tag('stat', name='branch_instructions', value='25'):  # TODO
            pass
        with tag('stat', name='branch_mispredictions', value='2'):  # TODO
            pass
        with tag('stat', name='load_instructions', value=str(int(stats['system.' + cpu_id(i) + '.num_load_insts']))):
            pass
        with tag('stat', name='store_instructions',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_store_insts']))):
            pass
        with tag('stat', name='committed_instructions', value=str(
                        int(stats['system.' + cpu_id(i) + '.num_int_insts']) + int(
                    stats['system.' + cpu_id(i) + '.num_fp_insts']))):
            pass
        with tag('stat', name='committed_int_instructions',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_int_insts']))):
            pass
        with tag('stat', name='committed_fp_instructions',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_fp_insts']))):
            pass
        with tag('stat', name='pipeline_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='total_cycles', value=str(int(stats['system.' + cpu_id(i) + '.numCycles']))):  # TODO
            pass
        with tag('stat', name='ROB_reads', value='100'):  # TODO
            pass
        with tag('stat', name='ROB_writes', value='100'):  # TODO
            pass
        with tag('stat', name='rename_reads', value='100'):  # TODO
            pass
        with tag('stat', name='rename_writes', value='100'):  # TODO
            pass
        with tag('stat', name='fp_rename_reads', value='100'):  # TODO
            pass
        with tag('stat', name='fp_rename_writes', value='100'):  # TODO
            pass
        with tag('stat', name='inst_window_reads', value='80'):  # TODO
            pass
        with tag('stat', name='inst_window_writes', value='80'):  # TODO
            pass
        with tag('stat', name='inst_window_wakeup_accesses', value='80'):  # TODO
            pass
        with tag('stat', name='fp_inst_window_reads', value='20'):  # TODO
            pass
        with tag('stat', name='fp_inst_window_writes', value='20'):  # TODO
            pass
        with tag('stat', name='fp_inst_window_wakeup_accesses', value='20'):  # TODO
            pass
        with tag('stat', name='int_regfile_reads', value='160'):  # TODO
            pass
        with tag('stat', name='float_regfile_reads', value='40'):  # TODO
            pass
        with tag('stat', name='int_regfile_writes', value='80'):  # TODO
            pass
        with tag('stat', name='float_regfile_writes', value='20'):  # TODO
            pass
        with tag('stat', name='function_calls',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_func_calls']))):  # TODO
            pass
        with tag('stat', name='context_switches', value='0'):  # TODO
            pass
        with tag('stat', name='ialu_accesses',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_int_alu_accesses']))):  # TODO
            pass
        with tag('stat', name='fpu_accesses',
                 value=str(int(stats['system.' + cpu_id(i) + '.num_fp_alu_accesses']))):  # TODO
            pass
        with tag('stat', name='mul_accesses', value='10'):  # TODO
            pass
        with tag('stat', name='cdb_alu_accesses', value='70'):  # TODO
            pass
        with tag('stat', name='cdb_mul_accesses', value='10'):  # TODO
            pass
        with tag('stat', name='cdb_fpu_accesses', value='20'):  # TODO
            pass
        with tag('stat', name='IFU_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='LSU_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='MemManU_I_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='MemManU_D_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='ALU_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='MUL_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='FPU_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='ALU_cdb_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='MUL_cdb_duty_cycle', value='1'):  # TODO
            pass
        with tag('stat', name='FPU_cdb_duty_cycle', value='1'):  # TODO
            pass

        gen_predictor()
        gen_itlb()
        gen_l1i()
        gen_dtlb()
        gen_l1d()
        gen_btargetbuf()


def gen_l2(tag, i=None):
    with tag('component', id='system.L2' + ('' if i is None else str(i)), name='L2 Cache', type='CacheUnit'):
        with tag('param', name='level', value='2'):
            pass
        with tag('param', name='size',
                 value=config_tree.execute('$.system.' + ('l2' if i is None else 'l2cache[' + str(i) + ']') + '.size')):
            pass
        with tag('param', name='block_size', value=config_tree.execute(
                                '$.system.' + ('l2' if i is None else 'l2cache[' + str(i) + ']') + '.tags.block_size')):
            pass
        with tag('param', name='assoc',
                 value=config_tree.execute('$.system.' + ('l2' if i is None else 'l2cache[' + str(i) + ']') + '.assoc')):
            pass
        with tag('param', name='num_banks', value='1'):
            pass
        with tag('param', name='latency',
                 value=config_tree.execute('$.system.' + ('l2' if i is None else 'l2cache[' + str(i) + ']') + '.hit_latency')):
            pass
        with tag('param', name='throughput', value='3'):
            pass
        with tag('param', name='miss_buffer_size', value='2'):
            pass
        with tag('param', name='fetch_buffer_size', value='2'):
            pass
        with tag('param', name='prefetch_buffer_size', value='2'):
            pass
        with tag('param', name='writeback_buffer_size', value='0'):
            pass
        with tag('param', name='clockrate', value=str(int(stats['sim_freq']) / 1000 / 10 ** 6)):
            pass
        with tag('param', name='tech_type', value='0'):
            pass
        with tag('param', name='Directory_type', value='2'):
            pass
        with tag('param', name='device_type', value='0'):
            pass
        with tag('param', name='core_type', value='1'):
            pass
        with tag('param', name='wire_mat_type', value='2'):
            pass
        with tag('param', name='wire_type', value='0'):
            pass
        with tag('param', name='miss_buffer_assoc', value='0'):
            pass
        with tag('param', name='fetch_buffer_assoc', value='0'):
            pass
        with tag('param', name='prefetch_buffer_assoc', value='0'):
            pass
        with tag('param', name='writeback_buffer_assoc', value='0'):
            pass
        with tag('param', name='miss_buffer_banks', value='1'):
            pass
        with tag('param', name='fetch_buffer_banks', value='1'):
            pass
        with tag('param', name='prefetch_buffer_banks', value='1'):
            pass
        with tag('param', name='writeback_buffer_banks', value='1'):
            pass
        with tag('param', name='cache_access_mode', value='0'):
            pass
        with tag('param', name='miss_buff_access_mode', value='2'):
            pass
        with tag('param', name='fetch_buff_access_mode', value='2'):
            pass
        with tag('param', name='prefetch_buff_access_mode', value='2'):
            pass
        with tag('param', name='writeback_buff_access_mode', value='2'):
            pass
        with tag('param', name='cache_rw_ports', value='1'):
            pass
        with tag('param', name='cache_rd_ports', value='0'):
            pass
        with tag('param', name='cache_wr_ports', value='0'):
            pass
        with tag('param', name='cache_se_rd_ports', value='0'):
            pass
        with tag('param', name='cache_search_ports', value='0'):
            pass
        with tag('param', name='miss_buff_rw_ports', value='1'):
            pass
        with tag('param', name='miss_buff_rd_ports', value='0'):
            pass
        with tag('param', name='miss_buff_wr_ports', value='0'):
            pass
        with tag('param', name='miss_buff_se_rd_ports', value='0'):
            pass
        with tag('param', name='miss_buff_search_ports', value='1'):
            pass
        with tag('param', name='fetch_buff_rw_ports', value='1'):
            pass
        with tag('param', name='fetch_buff_rd_ports', value='0'):
            pass
        with tag('param', name='fetch_buff_wr_ports', value='0'):
            pass
        with tag('param', name='fetch_buff_se_rd_ports', value='0'):
            pass
        with tag('param', name='fetch_buff_search_ports', value='1'):
            pass
        with tag('param', name='pf_buff_rw_ports', value='1'):
            pass
        with tag('param', name='pf_buff_rd_ports', value='0'):
            pass
        with tag('param', name='pf_buff_wr_ports', value='0'):
            pass
        with tag('param', name='pf_buff_se_rd_ports', value='0'):
            pass
        with tag('param', name='pf_buff_search_ports', value='1'):
            pass
        with tag('param', name='wb_buff_rw_ports', value='1'):
            pass
        with tag('param', name='wb_buff_rd_ports', value='0'):
            pass
        with tag('param', name='wb_buff_wr_ports', value='0'):
            pass
        with tag('param', name='wb_buff_se_rd_ports', value='0'):
            pass
        with tag('param', name='wb_buff_search_ports', value='1'):
            pass
        with tag('param', name='pure_ram', value='0'):
            pass
        with tag('stat', name='read_accesses',
                 value=str(int(stats['system.' + l2_id(i) + '.ReadExReq_accesses::total']) + int(
                         stats['system.' + l2_id(i) + '.ReadCleanReq_accesses::total']) + int(
                     stats['system.' + l2_id(i) + '.ReadSharedReq_accesses::total']))):
            pass
        with tag('stat', name='write_accesses',
                 value=str(int(stats['system.' + l2_id(i) + '.overall_accesses::total']) - int(
                         stats['system.' + l2_id(i) + '.ReadExReq_accesses::total']) - int(
                     stats['system.' + l2_id(i) + '.ReadSharedReq_accesses::total']) - int(
                     stats['system.' + l2_id(i) + '.ReadCleanReq_accesses::total']))):
            pass
        with tag('stat', name='read_misses',
                 value=str(int(stats['system.' + l2_id(i) + '.ReadExReq_misses::total']) + int(
                         stats['system.' + l2_id(i) + '.ReadSharedReq_misses::total']) + int(
                     stats['system.' + l2_id(i) + '.ReadCleanReq_misses::total']))):
            pass
        with tag('stat', name='write_misses', value=str(int(stats['system.' + l2_id(i) + '.overall_misses::total']) - int(
                stats['system.' + l2_id(i) + '.ReadExReq_misses::total']) - int(
            stats['system.' + l2_id(i) + '.ReadSharedReq_misses::total']) - int(
            stats['system.' + l2_id(i) + '.ReadCleanReq_misses::total']))):
            pass
        with tag('stat', name='conflicts', value='1'):  # TODO
            pass
        with tag('stat', name='duty_cycle', value='1'):  # TODO
            pass


def gen_nocs(tag):
    # TODO
    pass


def gen_mcs(tag):
    # TODO
    pass


def gen_misc(tag):
    # TODO
    pass


def gen_system(tag):
    with tag('param', name='core_tech_node', value='40'):
        pass
    with tag('param', name='target_core_clockrate', value=str(int(stats['sim_freq']) / 1000 / 10 ** 6)):
        pass
    with tag('param', name='temperature', value='380'):
        pass
    with tag('param', name='interconnect_projection_type', value='1'):
        pass
    with tag('param', name='device_type', value='0'):
        pass
    with tag('param', name='longer_channel_device', value='1'):
        pass
    with tag('param', name='machine_bits', value='64'):
        pass
    with tag('param', name='virtual_address_width', value='64'):
        pass
    with tag('param', name='physical_address_width', value='64'):
        pass
    with tag('param', name='virtual_memory_page_size', value='4096'):
        pass
    with tag('param', name='wire_is_mat_type', value='2'):
        pass
    with tag('param', name='wire_os_mat_type', value='2'):
        pass
    with tag('param', name='delay_wt', value='100'):
        pass
    with tag('param', name='area_wt', value='0'):
        pass
    with tag('param', name='dynamic_power_wt', value='100'):
        pass
    with tag('param', name='leakage_power_wt', value='0'):
        pass
    with tag('param', name='cycle_time_wt', value='0'):
        pass
    with tag('param', name='delay_dev', value='10000'):
        pass
    with tag('param', name='area_dev', value='10000'):
        pass
    with tag('param', name='dynamic_power_dev', value='10000'):
        pass
    with tag('param', name='leakage_power_dev', value='10000'):
        pass
    with tag('param', name='cycle_time_dev', value='10000'):
        pass
    with tag('param', name='ed', value='2'):
        pass
    with tag('param', name='burst_len', value='1'):
        pass
    with tag('param', name='int_prefetch_w', value='1'):
        pass
    with tag('param', name='page_sz_bits', value='0'):
        pass
    with tag('param', name='rpters_in_htree', value='1'):
        pass
    with tag('param', name='ver_htree_wires_over_array', value='0'):
        pass
    with tag('param', name='nuca', value='0'):
        pass
    with tag('param', name='nuca_bank_count', value='0'):
        pass
    with tag('param', name='force_cache_config', value='0'):
        pass
    with tag('param', name='wt', value='0'):
        pass
    with tag('param', name='force_wiretype', value='0'):
        pass
    with tag('param', name='print_detail', value='1'):
        pass
    with tag('param', name='add_ecc_b_', value='1'):
        pass
    with tag('param', name='broadcast_addr_din_over_ver_htrees', value='0'):
        pass
    with tag('stat', name='total_cycles', value=str(int(stats['system.' + cpu_id(0) + '.numCycles']))):
        pass

    for i in range(num_cpus):
        gen_core(tag, i)

    if numa:
        for i in range(num_l2caches):
            gen_l2(tag, i)
    else:
        gen_l2(tag)

    gen_nocs(tag)
    gen_mcs(tag)
    gen_misc(tag)


def gen_document(tag):
    with tag('component', id='root', name='root'):
        with tag('component', id='system', name='system', type='System'):
            gen_system(tag)


(doc, root_tag, text) = Doc().tagtext()

gen_document(root_tag)

mcpat_xml = etree.tostring(etree.fromstring(doc.getvalue()), pretty_print=True).rstrip()

with open(mcpat_in_xml_file_name, 'w') as mcpat_xml_file:
    mcpat_xml_file.write(mcpat_xml)

os.system('build/mcpat/mcpat -infile ' + mcpat_in_xml_file_name + ' -print_level 5 > ' + mcpat_out_file_name)
