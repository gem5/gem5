/*
 * Copyright 2019 Google, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/arm/fastmodel/PL330_DMAC/pl330.hh"

#include <cctype>

#include "params/FastModelPL330.hh"
#include "sim/core.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(FastModel, fastmodel);
namespace fastmodel
{

PL330::PL330(const FastModelPL330Params &params,
             sc_core::sc_module_name _name) :
    scx_evs_PL330(_name), clockPeriod(params.clock),
    dma(amba_m, params.name + ".dma", -1),
    pioS(amba_s, params.name + ".pio_s", -1),
    pioNs(amba_s_ns, params.name + ".pio_ns", -1),
    irqAbortReceiver("irq_abort_receiver")
{
    set_parameter("pl330.fifo_size", params.fifo_size);
    set_parameter("pl330.max_transfer", params.max_transfer);
    set_parameter("pl330.generate_clear", params.generate_clear);
    set_parameter("pl330.activate_delay", params.activate_delay);
    set_parameter("pl330.revision", params.revision);
    set_parameter("pl330.p_max_irqs", params.max_irqs);
    set_parameter("pl330.p_buffer_depth", params.buffer_depth);
    set_parameter("pl330.p_lsq_read_size", params.lsq_read_size);
    set_parameter("pl330.p_lsq_write_size", params.lsq_write_size);
    set_parameter("pl330.p_read_issuing_capability",
            params.read_issuing_capability);
    set_parameter("pl330.p_write_issuing_capability",
            params.write_issuing_capability);
    set_parameter("pl330.p_axi_bus_width_param", params.axi_bus_width);
    set_parameter("pl330.p_cache_line_words", params.cache_line_words);
    set_parameter("pl330.p_cache_lines", params.cache_lines);
    set_parameter("pl330.p_max_channels", params.max_channels);
    set_parameter("pl330.p_controller_nsecure", params.controller_nsecure);
    set_parameter("pl330.p_irq_nsecure", params.irq_nsecure);
    set_parameter("pl330.p_periph_nsecure", params.periph_nsecure);
    set_parameter("pl330.p_controller_boots", params.controller_boots);
    set_parameter("pl330.p_reset_pc", params.reset_pc);
    set_parameter("pl330.p_max_periph", params.max_periph);
    set_parameter("pl330.p_perip_request_acceptance_0",
            params.perip_request_acceptance_0);
    set_parameter("pl330.p_perip_request_acceptance_1",
            params.perip_request_acceptance_1);
    set_parameter("pl330.p_perip_request_acceptance_2",
            params.perip_request_acceptance_2);
    set_parameter("pl330.p_perip_request_acceptance_3",
            params.perip_request_acceptance_3);
    set_parameter("pl330.p_perip_request_acceptance_4",
            params.perip_request_acceptance_4);
    set_parameter("pl330.p_perip_request_acceptance_5",
            params.perip_request_acceptance_5);
    set_parameter("pl330.p_perip_request_acceptance_6",
            params.perip_request_acceptance_6);
    set_parameter("pl330.p_perip_request_acceptance_7",
            params.perip_request_acceptance_7);
    set_parameter("pl330.p_perip_request_acceptance_8",
            params.perip_request_acceptance_8);
    set_parameter("pl330.p_perip_request_acceptance_9",
            params.perip_request_acceptance_9);
    set_parameter("pl330.p_perip_request_acceptance_10",
            params.perip_request_acceptance_10);
    set_parameter("pl330.p_perip_request_acceptance_11",
            params.perip_request_acceptance_11);
    set_parameter("pl330.p_perip_request_acceptance_12",
            params.perip_request_acceptance_12);
    set_parameter("pl330.p_perip_request_acceptance_13",
            params.perip_request_acceptance_13);
    set_parameter("pl330.p_perip_request_acceptance_14",
            params.perip_request_acceptance_14);
    set_parameter("pl330.p_perip_request_acceptance_15",
            params.perip_request_acceptance_15);
    set_parameter("pl330.p_perip_request_acceptance_16",
            params.perip_request_acceptance_16);
    set_parameter("pl330.p_perip_request_acceptance_17",
            params.perip_request_acceptance_17);
    set_parameter("pl330.p_perip_request_acceptance_18",
            params.perip_request_acceptance_18);
    set_parameter("pl330.p_perip_request_acceptance_19",
            params.perip_request_acceptance_19);
    set_parameter("pl330.p_perip_request_acceptance_20",
            params.perip_request_acceptance_20);
    set_parameter("pl330.p_perip_request_acceptance_21",
            params.perip_request_acceptance_21);
    set_parameter("pl330.p_perip_request_acceptance_22",
            params.perip_request_acceptance_22);
    set_parameter("pl330.p_perip_request_acceptance_23",
            params.perip_request_acceptance_23);
    set_parameter("pl330.p_perip_request_acceptance_24",
            params.perip_request_acceptance_24);
    set_parameter("pl330.p_perip_request_acceptance_25",
            params.perip_request_acceptance_25);
    set_parameter("pl330.p_perip_request_acceptance_26",
            params.perip_request_acceptance_26);
    set_parameter("pl330.p_perip_request_acceptance_27",
            params.perip_request_acceptance_27);
    set_parameter("pl330.p_perip_request_acceptance_28",
            params.perip_request_acceptance_28);
    set_parameter("pl330.p_perip_request_acceptance_29",
            params.perip_request_acceptance_29);
    set_parameter("pl330.p_perip_request_acceptance_30",
            params.perip_request_acceptance_30);
    set_parameter("pl330.p_perip_request_acceptance_31",
            params.perip_request_acceptance_31);

    // Plumb up the mechanism which lets us set the clock rate inside the EVS.
    clockRateControl.bind(this->clock_rate_s);

    // Allocate all the source pins for each interrupt port.
    allocateIrq(0, params.port_irq_0_connection_count);
    allocateIrq(1, params.port_irq_1_connection_count);
    allocateIrq(2, params.port_irq_2_connection_count);
    allocateIrq(3, params.port_irq_3_connection_count);
    allocateIrq(4, params.port_irq_4_connection_count);
    allocateIrq(5, params.port_irq_5_connection_count);
    allocateIrq(6, params.port_irq_6_connection_count);
    allocateIrq(7, params.port_irq_7_connection_count);
    allocateIrq(8, params.port_irq_8_connection_count);
    allocateIrq(9, params.port_irq_9_connection_count);
    allocateIrq(10, params.port_irq_10_connection_count);
    allocateIrq(11, params.port_irq_11_connection_count);
    allocateIrq(12, params.port_irq_12_connection_count);
    allocateIrq(13, params.port_irq_13_connection_count);
    allocateIrq(14, params.port_irq_14_connection_count);
    allocateIrq(15, params.port_irq_15_connection_count);
    allocateIrq(16, params.port_irq_16_connection_count);
    allocateIrq(17, params.port_irq_17_connection_count);
    allocateIrq(18, params.port_irq_18_connection_count);
    allocateIrq(19, params.port_irq_19_connection_count);
    allocateIrq(20, params.port_irq_20_connection_count);
    allocateIrq(21, params.port_irq_21_connection_count);
    allocateIrq(22, params.port_irq_22_connection_count);
    allocateIrq(23, params.port_irq_23_connection_count);
    allocateIrq(24, params.port_irq_24_connection_count);
    allocateIrq(25, params.port_irq_25_connection_count);
    allocateIrq(26, params.port_irq_26_connection_count);
    allocateIrq(27, params.port_irq_27_connection_count);
    allocateIrq(28, params.port_irq_28_connection_count);
    allocateIrq(29, params.port_irq_29_connection_count);
    allocateIrq(30, params.port_irq_30_connection_count);
    allocateIrq(31, params.port_irq_31_connection_count);

    // Plumb the interrupts from inside the EVS to any external sinks.
    for (int i = 0; i < 32; i++) {
        // Create a receiver to receive interrupts from the EVS.
        irqReceiver.emplace_back(
                new SignalReceiver(csprintf("irq_receiver[%d]", i)));

        // Attach the receiver to the socket coming out of the EVS.
        irq[i].bind(irqReceiver[i]->signal_in);

        // Set up a handler for when the signal changes state.
        auto on_change = [&port = irqPort[i]](bool status)
        {
            // Loop through all the connections and propogate the signal.
            for (auto &pin: port)
                status ? pin->raise() : pin->lower();
        };

        // Install the handler.
        irqReceiver[i]->onChange(on_change);
    }

    // Set up the abort IRQ pins.
    for (int i = 0; i < params.port_irq_abort_connection_count; i++) {
        irqAbortPort.emplace_back(new IntSourcePin<PL330>(
                    csprintf("%s.irq_abort_master_port[%d]", name(), i),
                    i, this));
    }

    // Attach the receiver (already set up) to the EVS.
    irq_abort.bind(irqAbortReceiver.signal_in);

    // Set up a handler.
    auto abort_change = [this](bool status) {
        for (auto &pin: irqAbortPort)
            status ? pin->raise() : pin->lower();
    };

    // And install it.
    irqAbortReceiver.onChange(abort_change);
}

void
PL330::allocateIrq(int idx, int count)
{
    for (int i = 0; i < count; i++) {
        irqPort[idx].emplace_back(new IntSourcePin<PL330>(
                    csprintf("%s.irq_master_port[%d][%d]", name(), idx, i),
                    i, this));
    }
}

gem5::Port &
PL330::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "dma") {
        return dma;
    } else if (if_name == "pio_s") {
        return pioS;
    } else if (if_name == "pio_ns") {
        return pioNs;
    } else if (if_name.substr(0, 4) == "irq_") {
        auto suffix = if_name.substr(4);

        if (suffix == "abort")
            return *irqAbortPort.at(idx);

        // Existing functions like stoull, and to_number which uses it, skip
        // leading whitespace and ignore non-numeric characters at the end of
        // the string. We're going to be more picky than that.
        int port = -1;
        if (suffix.size() == 1 && isdigit(suffix[0])) {
            port = suffix[0] - '0';
        } else if (suffix.size() == 2 && isdigit(suffix[0]) &&
                isdigit(suffix[1])) {
            port = (suffix[1] - '0') * 10 + (suffix[0] - '0');
        }
        if (port != -1 && port < irqPort.size())
            return *irqPort[port].at(idx);
    }

    return scx_evs_PL330::gem5_getPort(if_name, idx);
}

void
PL330::start_of_simulation()
{
    // Set the clock rate using the divider inside the EVS.
    clockRateControl->set_mul_div(sim_clock::as_int::s, clockPeriod);
}

} // namespace fastmodel
} // namespace gem5
