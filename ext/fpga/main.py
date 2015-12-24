#!/usr/bin/python

#
# A Python script to run MyHDL enabled FPGA experiments based on the specified configurations.
#
# Copyright (C) Min Cai 2015
#

from myhdl import Signal, delay, always, now, Simulation, instance


def clk_driver(clk, period=20):
    low_time = int(period / 2)
    high_time = period - low_time

    @instance
    def drive_clk():
        while True:
            yield delay(low_time)
            clk.next = 1
            yield delay(high_time)
            clk.next = 0

    return drive_clk


def hello_world(clk, to='World!'):
    @always(clk.posedge)
    def say_hello():
        print '%s Hello %s' % (now(), to)

    return say_hello


def greetings():
    clk1 = Signal(0)
    clk2 = Signal(0)

    clk_driver1 = clk_driver(clk1)
    clk_driver2 = clk_driver(clk=clk2, period=19)

    hello1 = hello_world(clk=clk1)
    hello2 = hello_world(clk=clk2, to='MyHDL')

    return clk_driver1, clk_driver2, hello1, hello2

inst = greetings()
sim = Simulation(inst)
sim.run(50)
