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

from pprint import pprint

parser = OptionParser()
parser.add_option('--config', type='string', default='../../m5out/config.json', help = 'GEM5\'s system configuration JSON file name')
parser.add_option('--stats', type='string', default='../../m5out/stats.txt', help = 'GEM5\'s output statistics file name')

(option, arg) = parser.parse_args()

np = 4 # TODO

config_json_file_name = option.config
stats_file_name = option.stats

with open(config_json_file_name) as config_json_file:
    config = json.load(config_json_file)

config_tree = Tree(config)

pprint(config_tree.execute('$.system.kernel'))

stat_rule = Word(printables) + Word('.' + nums) + Optional(restOfLine)

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

for key, value in stats.iteritems():
    print '{}: {}'.format(key, value)

(doc, tag, text) = Doc().tagtext()

with tag('document'):
    with tag('component', id = 'root', name = 'root'):
        with tag('component', id = 'system', name = 'system', type = 'System'):
            with tag('param', name = 'core_tech_node', value = '40'):
                pass
            for i in range(np):
                with tag('component', id = 'system.core' + str(i), name = 'core' + str(i), type = 'Core'):
                    pass

mcpat_xml = doc.getvalue()

mcpat_xml_file_name = 'mcpat.xml'

with open(mcpat_xml_file_name, 'w') as mcpat_xml_file:
    mcpat_xml_file.write(mcpat_xml)

mcpat_xml = etree.tostring(etree.parse(mcpat_xml_file_name), pretty_print = True).rstrip()

with open(mcpat_xml_file_name, 'w') as mcpat_xml_file:
    mcpat_xml_file.write(mcpat_xml)

os.system('../../build/mcpat/mcpat -infile ' + mcpat_xml_file_name + ' -print_level 0')




