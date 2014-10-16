# Copyright (c) 2014 ARM Limited
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
#
# Author: Andrew Bardsley

# This script allows .ini and .json system config file generated from a
# previous gem5 run to be read in and instantiated.
#
# This may be useful as a way of allowing variant run scripts (say,
# with more complicated than usual checkpointing/stats dumping/
# simulation control) to read pre-described systems from config scripts
# with better system-description capabilities.  Splitting scripts
# between system construction and run control may allow better
# debugging.

import argparse
import ConfigParser
import inspect
import json
import re
import sys

import m5
import m5.ticks as ticks

sim_object_classes_by_name = {
    cls.__name__: cls for cls in m5.objects.__dict__.itervalues()
    if inspect.isclass(cls) and issubclass(cls, m5.objects.SimObject) }

# Add some parsing functions to Param classes to handle reading in .ini
#   file elements.  This could be moved into src/python/m5/params.py if
#   reading .ini files from Python proves to be useful

def no_parser(cls, flags, param):
    raise Exception('Can\'t parse string: %s for parameter'
        ' class: %s' % (str(param), cls.__name__))

def simple_parser(suffix='', cast=lambda i: i):
    def body(cls, flags, param):
        return cls(cast(param + suffix))
    return body

# def tick_parser(cast=m5.objects.Latency): # lambda i: i):
def tick_parser(cast=lambda i: i):
    def body(cls, flags, param):
        old_param = param
        ret = cls(cast(str(param) + 't'))
        return ret
    return body

def addr_range_parser(cls, flags, param):
    sys.stdout.flush()
    low, high = param.split(':')
    return m5.objects.AddrRange(long(low), long(high))

def memory_bandwidth_parser(cls, flags, param):
    # The string will be in tick/byte
    # Convert to byte/tick
    value = 1.0 / float(param)
    # Convert to byte/s
    value = ticks.fromSeconds(value)
    return cls('%fB/s' % value)

# These parameters have trickier parsing from .ini files than might be
#   expected
param_parsers = {
    'Bool': simple_parser(),
    'ParamValue': no_parser,
    'NumericParamValue': simple_parser(cast=long),
    'TickParamValue': tick_parser(),
    'Frequency': tick_parser(cast=m5.objects.Latency),
    'Voltage': simple_parser(suffix='V'),
    'Enum': simple_parser(),
    'MemorySize': simple_parser(suffix='B'),
    'MemorySize32': simple_parser(suffix='B'),
    'AddrRange': addr_range_parser,
    'String': simple_parser(),
    'MemoryBandwidth': memory_bandwidth_parser,
    'Time': simple_parser()
    }

for name, parser in param_parsers.iteritems():
    setattr(m5.params.__dict__[name], 'parse_ini', classmethod(parser))

class PortConnection(object):
    """This class is similar to m5.params.PortRef but with just enough
    information for ConfigManager"""

    def __init__(self, object_name, port_name, index):
        self.object_name = object_name
        self.port_name = port_name
        self.index = index

    @classmethod
    def from_string(cls, str):
        m = re.match('(.*)\.([^.\[]+)(\[(\d+)\])?', str)
        object_name, port_name, whole_index, index = m.groups()
        if index is not None:
            index = int(index)
        else:
            index = 0

        return PortConnection(object_name, port_name, index)

    def __str__(self):
        return '%s.%s[%d]' % (self.object_name, self.port_name, self.index)

    def __cmp__(self, right):
        return cmp((self.object_name, self.port_name, self.index),
            (right.object_name, right.port_name, right.index))

def to_list(v):
    """Convert any non list to a singleton list"""
    if isinstance(v, list):
        return v
    else:
        return [v]

class ConfigManager(object):
    """Manager for parsing a Root configuration from a config file"""
    def __init__(self, config):
        self.config = config
        self.objects_by_name = {}
        self.flags = config.get_flags()

    def find_object(self, object_name):
        """Find and configure (with just non-SimObject parameters)
        a single object"""

        if object_name == 'Null':
            return NULL

        if object_name in self.objects_by_name:
            return self.objects_by_name[object_name]

        object_type = self.config.get_param(object_name, 'type')

        if object_type not in sim_object_classes_by_name:
            raise Exception('No SimObject type %s is available to'
                ' build: %s' % (object_type, object_name))

        object_class = sim_object_classes_by_name[object_type]

        parsed_params = {}

        for param_name, param in object_class._params.iteritems():
            if issubclass(param.ptype, m5.params.ParamValue):
                if isinstance(param, m5.params.VectorParamDesc):
                    param_values = self.config.get_param_vector(object_name,
                        param_name)

                    param_value = [ param.ptype.parse_ini(self.flags, value)
                        for value in param_values ]
                else:
                    param_value = param.ptype.parse_ini(
                        self.flags, self.config.get_param(object_name,
                        param_name))

                parsed_params[param_name] = param_value

        obj = object_class(**parsed_params)
        self.objects_by_name[object_name] = obj

        return obj

    def fill_in_simobj_parameters(self, object_name, obj):
        """Fill in all references to other SimObjects in an objects
        parameters.  This relies on all referenced objects having been
        created"""

        if object_name == 'Null':
            return NULL

        for param_name, param in obj.__class__._params.iteritems():
            if issubclass(param.ptype, m5.objects.SimObject):
                if isinstance(param, m5.params.VectorParamDesc):
                    param_values = self.config.get_param_vector(object_name,
                        param_name)

                    setattr(obj, param_name, [ self.objects_by_name[name]
                        for name in param_values ])
                else:
                    param_value = self.config.get_param(object_name,
                        param_name)

                    if param_value != 'Null':
                        setattr(obj, param_name, self.objects_by_name[
                            param_value])

        return obj

    def fill_in_children(self, object_name, obj):
        """Fill in the children of this object.  This relies on all the
        referenced objects having been created"""

        children = self.config.get_object_children(object_name)

        for child_name, child_paths in children:
            param = obj.__class__._params.get(child_name, None)

            if isinstance(child_paths, list):
                child_list = [ self.objects_by_name[path]
                    for path in child_paths ]
            else:
                child_list = self.objects_by_name[child_paths]

            obj.add_child(child_name, child_list)

            for path in to_list(child_paths):
                self.fill_in_children(path, self.objects_by_name[path])

        return obj

    def parse_port_name(self, port):
        """Parse the name of a port"""

        m = re.match('(.*)\.([^.\[]+)(\[(\d+)\])?', port)
        peer, peer_port, whole_index, index = m.groups()
        if index is not None:
            index = int(index)
        else:
            index = 0

        return (peer, self.objects_by_name[peer], peer_port, index)

    def gather_port_connections(self, object_name, obj):
        """Gather all the port-to-port connections from the named object.
        Returns a list of (PortConnection, PortConnection) with unordered
        (wrt. master/slave) connection information"""

        if object_name == 'Null':
            return NULL

        parsed_ports = []
        for port_name, port in obj.__class__._ports.iteritems():
            # Assume that unnamed ports are unconnected
            peers = self.config.get_port_peers(object_name, port_name)

            for index, peer in zip(xrange(0, len(peers)), peers):
                parsed_ports.append((
                    PortConnection(object_name, port.name, index),
                    PortConnection.from_string(peer)))

        return parsed_ports

    def bind_ports(self, connections):
        """Bind all ports from the given connection list.  Note that the
        connection list *must* list all connections with both (slave,master)
        and (master,slave) orderings"""

        # Markup a dict of how many connections are made to each port.
        #   This will be used to check that the next-to-be-made connection
        #   has a suitable port index
        port_bind_indices = {}
        for from_port, to_port in connections:
            port_bind_indices[
                (from_port.object_name, from_port.port_name)] = 0

        def port_has_correct_index(port):
            return port_bind_indices[
                (port.object_name, port.port_name)] == port.index

        def increment_port_index(port):
            port_bind_indices[
                (port.object_name, port.port_name)] += 1

        # Step through the sorted connections.  Exactly one of
        #   each (slave,master) and (master,slave) pairs will be
        #   bindable because the connections are sorted.
        # For example:        port_bind_indices
        #   left      right   left right
        #   a.b[0] -> d.f[1]  0    0 X
        #   a.b[1] -> e.g     0    0    BIND!
        #   e.g -> a.b[1]     1 X  0
        #   d.f[0] -> f.h     0    0    BIND!
        #   d.f[1] -> a.b[0]  1    0    BIND!
        connections_to_make = []
        for connection in sorted(connections):
            from_port, to_port = connection

            if (port_has_correct_index(from_port) and
                port_has_correct_index(to_port)):

                connections_to_make.append((from_port, to_port))

                increment_port_index(from_port)
                increment_port_index(to_port)

        # Exactly half of the connections (ie. all of them, one per
        #   direction) must now have been made
        if (len(connections_to_make) * 2) != len(connections):
            raise Exception('Port bindings can\'t be ordered')

        # Actually do the binding
        for from_port, to_port in connections_to_make:
            from_object = self.objects_by_name[from_port.object_name]
            to_object = self.objects_by_name[to_port.object_name]

            setattr(from_object, from_port.port_name,
                getattr(to_object, to_port.port_name))

    def find_all_objects(self):
        """Find and build all SimObjects from the config file and connect
        their ports together as described.  Does not instantiate system"""

        # Build SimObjects for all sections of the config file
        #   populating not-SimObject-valued parameters
        for object_name in self.config.get_all_object_names():
            self.find_object(object_name)

        # Add children to objects in the hierarchy from root
        self.fill_in_children('root', self.find_object('root'))

        # Now fill in SimObject-valued parameters in the knowledge that
        #   this won't be interpreted as becoming the parent of objects
        #   which are already in the root hierarchy
        for name, obj in self.objects_by_name.iteritems():
            self.fill_in_simobj_parameters(name, obj)

        # Gather a list of all port-to-port connections
        connections = []
        for name, obj in self.objects_by_name.iteritems():
            connections += self.gather_port_connections(name, obj)

        # Find an acceptable order to bind those port connections and
        #   bind them
        self.bind_ports(connections)

class ConfigFile(object):
    def get_flags(self):
        return set()

    def load(self, config_file):
        """Load the named config file"""
        pass

    def get_all_object_names(self):
        """Get a list of all the SimObject paths in the configuration"""
        pass

    def get_param(self, object_name, param_name):
        """Get a single param or SimObject reference from the configuration
        as a string"""
        pass

    def get_param_vector(self, object_name, param_name):
        """Get a vector param or vector of SimObject references from the
        configuration as a list of strings"""
        pass

    def get_object_children(self, object_name):
        """Get a list of (name, paths) for each child of this object.
        paths is either a single string object path or a list of object
        paths"""
        pass

    def get_port_peers(self, object_name, port_name):
        """Get the list of connected port names (in the string form
        object.port(\[index\])?) of the port object_name.port_name"""
        pass

class ConfigIniFile(ConfigFile):
    def __init__(self):
        self.parser = ConfigParser.ConfigParser()

    def load(self, config_file):
        self.parser.read(config_file)

    def get_all_object_names(self):
        return self.parser.sections()

    def get_param(self, object_name, param_name):
        return self.parser.get(object_name, param_name)

    def get_param_vector(self, object_name, param_name):
        return self.parser.get(object_name, param_name).split()

    def get_object_children(self, object_name):
        if self.parser.has_option(object_name, 'children'):
            children = self.parser.get(object_name, 'children')
            child_names = children.split()
        else:
            child_names = []

        def make_path(child_name):
            if object_name == 'root':
                return child_name
            else:
                return '%s.%s' % (object_name, child_name)

        return [ (name, make_path(name)) for name in child_names ]

    def get_port_peers(self, object_name, port_name):
        if self.parser.has_option(object_name, port_name):
            peer_string = self.parser.get(object_name, port_name)
            return peer_string.split()
        else:
            return []

class ConfigJsonFile(ConfigFile):
    def __init__(self):
        pass

    def is_sim_object(self, node):
        return isinstance(node, dict) and 'path' in node

    def find_all_objects(self, node):
        if self.is_sim_object(node):
            self.object_dicts[node['path']] = node

        if isinstance(node, list):
            for elem in node:
                self.find_all_objects(elem)
        elif isinstance(node, dict):
            for elem in node.itervalues():
                self.find_all_objects(elem)

    def load(self, config_file):
        root = json.load(open(config_file, 'r'))
        self.object_dicts = {}
        self.find_all_objects(root)

    def get_all_object_names(self):
        return sorted(self.object_dicts.keys())

    def parse_param_string(self, node):
        if node is None:
            return "Null"
        elif self.is_sim_object(node):
            return node['path']
        else:
            return str(node)

    def get_param(self, object_name, param_name):
        obj = self.object_dicts[object_name]

        return self.parse_param_string(obj[param_name])

    def get_param_vector(self, object_name, param_name):
        obj = self.object_dicts[object_name]

        return [ self.parse_param_string(p) for p in obj[param_name] ]

    def get_object_children(self, object_name):
        """It is difficult to tell which elements are children in the
        JSON file as there is no explicit 'children' node.  Take any
        element which is a full SimObject description or a list of
        SimObject descriptions.  This will not work with a mixed list of
        references and descriptions but that's a scenario that isn't
        possible (very likely?) with gem5's binding/naming rules"""
        obj = self.object_dicts[object_name]

        children = []
        for name, node in obj.iteritems():
            if self.is_sim_object(node):
                children.append((name, node['path']))
            elif isinstance(node, list) and node != [] and all([
                self.is_sim_object(e) for e in node ]):
                children.append((name, [ e['path'] for e in node ]))

        return children

    def get_port_peers(self, object_name, port_name):
        """Get the 'peer' element of any node with 'peer' and 'role'
        elements"""
        obj = self.object_dicts[object_name]

        peers = []
        if port_name in obj and 'peer' in obj[port_name] and \
            'role' in obj[port_name]:
            peers = to_list(obj[port_name]['peer'])

        return peers

parser = argparse.ArgumentParser()

parser.add_argument('config_file', metavar='config-file.ini',
    help='.ini configuration file to load and run')

args = parser.parse_args(sys.argv[1:])

if args.config_file.endswith('.ini'):
    config = ConfigIniFile()
    config.load(args.config_file)
else:
    config = ConfigJsonFile()
    config.load(args.config_file)

ticks.fixGlobalFrequency()

mgr = ConfigManager(config)

mgr.find_all_objects()

m5.instantiate()

exit_event = m5.simulate()
print 'Exiting @ tick %i because %s' % (
    m5.curTick(), exit_event.getCause())
