#!/usr/bin/python

#
# A Python script to run MyHDL enabled FPGA experiments based on the specified configurations.
#
# Copyright (C) Min Cai 2015
#

from myhdl import Signal, delay, now, Simulation, always, always_comb, always_seq, instance, intbv, bin, ResetSignal, StopSimulation
from random import randrange


ACTIVE_LOW, INACTIVE_HIGH = 0, 1


def inc(count, enable, clock, reset, n):
    @always_seq(clock.posedge, reset=reset)
    def inc_logic():
        if enable:
            count.next = (count + 1) % n

    return inc_logic


def test_inc():
    count, enable, clock = [Signal(intbv(0)) for i in range(3)]
    reset = ResetSignal(0, active=ACTIVE_LOW, async=True)

    inc_1 = inc(count, enable, clock, reset, n=4)

    half_period = delay(10)

    @always(half_period)
    def clock_gen():
        clock.next = not clock

    @instance
    def stimulus():
        reset.next = ACTIVE_LOW
        yield clock.negedge
        reset.next = INACTIVE_HIGH
        for i in range(12):
            enable.next = min(1, randrange(3))
            yield clock.negedge
        raise StopSimulation

    @instance
    def monitor():
        print 'enable count'
        yield reset.posedge
        while True:
            yield clock.posedge
            yield delay(1)
            print '%s %s' % (enable, count)

    return clock_gen, stimulus, inc_1, monitor


def mux(z, a, b, sel):
    @always_comb
    def mux_logic():
        if sel == 1:
            z.next = a
        else:
            z.next = b

    return mux_logic


def test_mux():
    print 'z a b sel'
    for i in range(8):
        a.next, b.next, sel.next = randrange(8), randrange(8), randrange(2)
        yield delay(10)
        print '%s %s %s %s' % (z, a, b, sel)


def bin2gray(b, g, width):
    @always_comb
    def logic():
        for i in range(width):
            g.next[i] = b[i + 1] ^ b[i]

    return logic


def test_bin2gray(width):
    B = Signal(intbv(0))
    G = Signal(intbv(0))

    dut = bin2gray(B, G, width)

    @instance
    def stimulus():
        for i in range(2 ** width):
            B.next = intbv(i)
            yield delay(10)
            print 'B: ' + bin(B, width) + '| G: ' + bin(G, width)

    return dut, stimulus


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


sim = Simulation(test_inc())
sim.run()

z, a, b, sel = [Signal(intbv(0)) for i in range(4)]
sim = Simulation(mux(z, a, b, sel), test_mux())
sim.run()

sim = Simulation(test_bin2gray(width=3))
sim.run()

inst = greetings()
sim = Simulation(inst)
sim.run(50)
