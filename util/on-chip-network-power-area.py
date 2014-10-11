# Copyright (c) 2014 Mark D. Hill and David A. Wood
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


from ConfigParser import ConfigParser
import string, sys, subprocess, os

# Compile DSENT to generate the Python module and then import it.
# This script assumes it is executed from the gem5 root.
print("Attempting compilation")
from subprocess import call

src_dir = 'ext/dsent'
build_dir = 'build/ext/dsent'

if not os.path.exists(build_dir):
    os.makedirs(build_dir)
os.chdir(build_dir)

error = call(['cmake', '../../../%s' % src_dir])
if error:
    print("Failed to run cmake")
    exit(-1)

error = call(['make'])
if error:
    print("Failed to run make")
    exit(-1)

print("Compiled dsent")
os.chdir("../../../")
sys.path.append("build/ext/dsent")
import dsent

# Parse gem5 config.ini file for the configuration parameters related to
# the on-chip network.
def parseConfig(config_file):
    config = ConfigParser()
    if not config.read(config_file):
        print("ERROR: config file '", config_file, "' not found")
        sys.exit(1)

    if not config.has_section("system.ruby.network"):
        print("ERROR: Ruby network not found in '", config_file)
        sys.exit(1)

    if config.get("system.ruby.network", "type") != "GarnetNetwork_d" :
        print("ERROR: Garnet network not used in '", config_file)
        sys.exit(1)

    number_of_virtual_networks = config.getint("system.ruby.network",
                                               "number_of_virtual_networks")
    vcs_per_vnet = config.getint("system.ruby.network", "vcs_per_vnet")

    buffers_per_data_vc = config.getint("system.ruby.network",
                                        "buffers_per_data_vc")
    buffers_per_control_vc = config.getint("system.ruby.network",
                                           "buffers_per_ctrl_vc")

    ni_flit_size_bits = 8 * config.getint("system.ruby.network",
                                          "ni_flit_size")

    routers = config.get("system.ruby.network", "routers").split()
    int_links = config.get("system.ruby.network", "int_links").split()
    ext_links = config.get("system.ruby.network", "ext_links").split()

    return (config, number_of_virtual_networks, vcs_per_vnet,
            buffers_per_data_vc, buffers_per_control_vc, ni_flit_size_bits,
            routers, int_links, ext_links)


def getClock(obj, config):
    if config.get(obj, "type") == "SrcClockDomain":
        return config.getint(obj, "clock")

    if config.get(obj, "type") == "DerivedClockDomain":
        source = config.get(obj, "clk_domain")
        divider = config.getint(obj, "clk_divider")
        return getClock(source, config)  / divider

    source = config.get(obj, "clk_domain")
    return getClock(source, config)


## Compute the power consumed by the given router
def computeRouterPowerAndArea(router, stats_file, config, int_links, ext_links,
                              number_of_virtual_networks, vcs_per_vnet,
                              buffers_per_data_vc, buffers_per_control_vc,
                              ni_flit_size_bits):
    frequency = getClock(router, config)
    num_ports = 0

    for int_link in int_links:
        if config.get(int_link, "node_a") == router or \
           config.get(int_link, "node_b") == router:
           num_ports += 1

    for ext_link in ext_links:
        if config.get(ext_link, "int_node") == router:
           num_ports += 1

    power = dsent.computeRouterPowerAndArea(frequency, num_ports, num_ports,
                                            number_of_virtual_networks,
                                            vcs_per_vnet, buffers_per_data_vc,
                                            ni_flit_size_bits)

    print("%s Power: " % router, power)


## Compute the power consumed by the given link
def computeLinkPower(link, stats_file, config, sim_seconds):
    frequency = getClock(link + ".nls0", config)
    power = dsent.computeLinkPower(frequency)
    print("%s.nls0 Power: " % link, power)

    frequency = getClock(link + ".nls1", config)
    power = dsent.computeLinkPower(frequency)
    print("%s.nls1 Power: " % link, power)


def parseStats(stats_file, config, router_config_file, link_config_file,
               routers, int_links, ext_links, number_of_virtual_networks,
               vcs_per_vnet, buffers_per_data_vc, buffers_per_control_vc,
               ni_flit_size_bits):

    # Open the stats.txt file and parse it to for the required numbers
    # and the number of routers.
    try:
        stats_handle = open(stats_file, 'r')
        stats_handle.close()
    except IOError:
        print("Failed to open ", stats_file, " for reading")
        exit(-1)

    # Now parse the stats
    pattern = "sim_seconds"
    lines = string.split(subprocess.check_output(
                ["grep", pattern, stats_file]), '\n', -1)
    assert len(lines) >= 1

    ## Assume that the first line is the one required
    [l1,l2,l3] = lines[0].partition(" ")
    l4 = l3.strip().partition(" ")
    simulation_length_in_seconds = float(l4[0])

    # Initialize DSENT with a configuration file
    dsent.initialize(router_config_file)

    # Compute the power consumed by the routers
    for router in routers:
        computeRouterPowerAndArea(router, stats_file, config, int_links,
                                  ext_links, number_of_virtual_networks,
                                  vcs_per_vnet, buffers_per_data_vc,
                                  buffers_per_control_vc, ni_flit_size_bits)

    # Finalize DSENT
    dsent.finalize()

    # Initialize DSENT with a configuration file
    dsent.initialize(link_config_file)

    # Compute the power consumed by the links
    for link in int_links:
        computeLinkPower(link, stats_file, config,
                         simulation_length_in_seconds)
    for link in ext_links:
        computeLinkPower(link, stats_file, config,
                         simulation_length_in_seconds)

    # Finalize DSENT
    dsent.finalize()

# This script parses the config.ini and the stats.txt from a run and
# generates the power and the area of the on-chip network using DSENT
def main():
    if len(sys.argv) != 5:
        print("Usage: ", sys.argv[0], " <gem5 root directory> " \
              "<simulation directory> <router config file> <link config file>")
        exit(-1)

    print("WARNING: configuration files for DSENT and McPAT are separate. " \
          "Changes made to one are not reflected in the other.")

    (config, number_of_virtual_networks, vcs_per_vnet, buffers_per_data_vc,
     buffers_per_control_vc, ni_flit_size_bits, routers, int_links,
     ext_links) = parseConfig("%s/%s/config.ini" % (sys.argv[1], sys.argv[2]))

    parseStats("%s/%s/stats.txt" % (sys.argv[1], sys.argv[2]), config,
               sys.argv[3], sys.argv[4], routers, int_links, ext_links,
               number_of_virtual_networks, vcs_per_vnet, buffers_per_data_vc,
               buffers_per_control_vc, ni_flit_size_bits)

if __name__ == "__main__":
    main()
