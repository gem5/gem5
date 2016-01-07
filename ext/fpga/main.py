#!/usr/bin/python

#
# A Python script to run MyHDL enabled FPGA experiments based on the specified configurations.
#
# Copyright (C) Min Cai 2015
#

from myhdl import *
from random import randrange


ACTIVE_LOW, INACTIVE_HIGH = 0, 1

FRAME_SIZE = 8
t_state = enum('SEARCH', 'CONFIRM', 'SYNC')


def framer_ctrl(sof, state, sync_flag, clk, reset):
    index = Signal(0)

    @always_seq(clk.posedge, reset=reset)
    def fsm():
        index.next = (index + 1) % FRAME_SIZE
        sof.next = 0

        if state == t_state.SEARCH:
            index.next = 1
            if sync_flag:
                state.next = t_state.CONFIRM
        elif state == t_state.CONFIRM:
            if index == 0:
                if sync_flag:
                    state.next = t_state.SYNC
                else:
                    state.next = t_state.SEARCH
        elif state == t_state.SYNC:
            if index == 0:
                if not sync_flag:
                    state.next = t_state.SEARCH
            sof.next = (index == FRAME_SIZE - 1)
        else:
            raise ValueError('Undefined state')

    return fsm


def test_framer_ctrl():
    sof = Signal(bool(0))
    sync_flag = Signal(bool(0))
    clk = Signal(bool(0))
    reset = ResetSignal(1, active=ACTIVE_LOW, async=True)
    state = Signal(t_state.SEARCH)

    framer_ctrl1 = framer_ctrl(sof, state, sync_flag, clk, reset)

    @always(delay(10))
    def clk_gen():
        clk.next = not clk

    @instance
    def stimulus():
        for i in range(3):
            yield clk.posedge
        for n in (12, 8, 8, 4):
            sync_flag.next = 1
            yield clk.posedge
            sync_flag.next = 0
            for i in range(n-1):
                yield clk.posedge
        raise StopSimulation

    @always_seq(clk.posedge, reset=reset)
    def monitor():
        print sync_flag, sof, state

    return framer_ctrl1, clk_gen, stimulus, monitor


def inc(count, enable, clk, reset, n):
    @always_seq(clk.posedge, reset=reset)
    def inc_logic():
        if enable:
            count.next = (count + 1) % n

    return inc_logic


def test_inc():
    count = Signal(intbv(0))
    enable = Signal(bool(0))
    clk = Signal(bool(0))

    reset = ResetSignal(0, active=ACTIVE_LOW, async=True)

    inc_1 = inc(count, enable, clk, reset, n=4)

    HALF_PERIOD = delay(10)

    @always(HALF_PERIOD)
    def clock_gen():
        clk.next = not clk

    @instance
    def stimulus():
        reset.next = ACTIVE_LOW
        yield clk.negedge
        reset.next = INACTIVE_HIGH
        for i in range(12):
            enable.next = min(1, randrange(3))
            yield clk.negedge
        raise StopSimulation

    @instance
    def monitor():
        print 'enable count'
        yield reset.posedge
        while True:
            yield clk.posedge
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


Simulation(traceSignals(test_framer_ctrl)).run()

Simulation(traceSignals(test_inc)).run()

z, a, b, sel = [Signal(intbv(0)) for i in range(4)]
Simulation(mux(z, a, b, sel), test_mux()).run()

Simulation(test_bin2gray(width=3)).run()

inst = greetings()
Simulation(inst).run(50)
