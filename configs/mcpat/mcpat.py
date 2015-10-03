#!/usr/bin/python

#
# A Python script to generate McPAT XML files for the specified GEM5 config and stat files.
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

from optparse import OptionParser

parser = OptionParser()
parser.add_option('--config', type='string', default='../../m5out/config.json', help = 'GEM5\'s system configuration JSON file name')
parser.add_option('--stats', type='string', default='../../m5out/stats.txt', help = 'GEM5\'s output statistics file name')

(option, arg) = parser.parse_args()

config_json_file_name = option.config
stats_file_name = option.stats

with open(config_json_file_name) as config_json_file:
    config = json.load(config_json_file)

config_tree = Tree(config)

stat_rule = Word(printables) + Word('nan.%' + nums) + Optional(restOfLine)

stats = collections.OrderedDict({})

with open(stats_file_name) as stats_file:
    for stat_line in stats_file:
        try:
            stat = stat_rule.parseString(stat_line)
            key = stat[0]
            value = stat[1]
            stats[key] = value
        except ParseException, err:
            pass

def gen_system(tag):
    with tag('param', name = 'core_tech_node', value = '40'):
        pass
    with tag('param', name = 'target_core_clockrate', value = str(int(stats['sim_freq']) / 1000 / 10**6)):
        pass
    with tag('param', name = 'temperature', value = '380'):
        pass
    with tag('param', name = 'device_type', value = '0'):
        pass
    with tag('param', name = 'longer_channel_device', value = '1'):
        pass
    with tag('param', name = 'machine_bits', value = '64'):
        pass
    with tag('param', name = 'virtual_address_width', value = '64'):
        pass
    with tag('param', name = 'physical_address_width', value = '64'):
        pass
    with tag('param', name = 'virtual_memory_page_size', value = '4096'):
        pass

    np = config_tree.execute('len($.system.cpu)')

    for i in range(np):
        gen_core(tag, i)

    gen_l2()
    gen_nocs()
    gen_mcs()
    gen_misc()

def gen_core(tag, i):
    def gen_l1i():
        pass
    def gen_l1d():
        pass

    with tag('component', id = 'system.core' + str(i), name = 'core' + str(i), type = 'Core'):
        with tag('param', name = 'instruction_length', value = '32'):
            pass
        with tag('param', name = 'number_hardware_threads', value = config_tree.execute('$.system.cpu[' + str(i) + '].numThreads')):
            pass
        with tag('param', name = 'opcode_width', value = '16'):
            pass
        with tag('param', name = 'instruction_buffer_size', value = '32'):
            pass
        with tag('param', name = 'number_instruction_fetch_ports', value = '1'):
            pass
        with tag('param', name = 'peak_issue_width', value = '1'):
            pass
        # TODO

def gen_l2():
    # TODO
    pass

def gen_nocs():
    # TODO
    pass

def gen_mcs():
    # TODO
    pass

def gen_misc():
    # TODO
    pass

def gen_document(tag):
    with tag('document'):
        with tag('component', id = 'root', name = 'root'):
            with tag('component', id = 'system', name = 'system', type = 'System'):
                gen_system(tag)

(doc, tag, text) = Doc().tagtext()

gen_document(tag)

mcpat_xml_file_name = 'mcpat.xml'

mcpat_xml = etree.tostring(etree.fromstring(doc.getvalue()), pretty_print = True).rstrip()

with open(mcpat_xml_file_name, 'w') as mcpat_xml_file:
    mcpat_xml_file.write(mcpat_xml)

os.system('../../build/mcpat/mcpat -infile ' + mcpat_xml_file_name + ' -print_level 0')




