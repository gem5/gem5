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

#include "arch/arm/fastmodel/GIC/gic.hh"

#include "base/trace.hh"
#include "params/FastModelGIC.hh"
#include "params/SCFastModelGIC.hh"

namespace gem5
{

namespace fastmodel
{

int
SCGIC::Terminator::countUnbound(const Initiators &inits)
{
    int count = 0;
    for (auto &init : inits)
        if (!init.get_port_base().size())
            count++;
    return count;
}

SCGIC::Terminator::Terminator(sc_core::sc_module_name _name, Initiators &inits)
    : sc_core::sc_module(_name), targets("targets", countUnbound(inits))
{
    // For every unbound initiator socket, connected it to one
    // terminator target socket.
    int i = 0;
    for (auto &init : inits) {
        if (!init.get_port_base().size()) {
            auto &term = targets.at(i++);
            term.bind(*this);
            term.bind(init);
        }
    }
}

void
SCGIC::Terminator::sendTowardsCPU(uint8_t len, const uint8_t *data)
{
    panic("Call to terminated interface!");
}

SCGIC::SCGIC(const SCFastModelGICParams &params, sc_core::sc_module_name _name)
    : scx_evs_GIC(_name),
      _params(params),
      resetPort(params.name + ".reset", 0),
      poResetPort(params.name + ".po_reset", 0)
{
    signalInterrupt.bind(signal_interrupt);

    resetPort.signal_out.bind(scx_evs_GIC::normal_reset);
    poResetPort.signal_out.bind(scx_evs_GIC::po_reset);

    for (int i = 0; i < wake_request.size(); i++) {
        wakeRequests.emplace_back(
            new SignalReceiver(csprintf("%s.wakerequest[%d]", name(), i)));
        wake_request[i].bind(wakeRequests[i]->signal_in);
    }

    set_parameter("gic.enabled", params.enabled);
    set_parameter("gic.has-gicv3", params.has_gicv3);
    set_parameter("gic.has-gicv4.1", params.has_gicv4_1);
    set_parameter("gic.vPEID-bits", params.vPEID_bits);
    set_parameter("gic.print-memory-map", params.print_mmap);
    set_parameter("gic.monolithic", params.monolithic);
    set_parameter("gic.direct-lpi-support", params.direct_lpi_support);
    set_parameter("gic.CPU-affinities", params.cpu_affinities);
    set_parameter("gic.non-ARE-core-count", params.non_ARE_core_count);
    set_parameter("gic.reg-base", params.reg_base);
    set_parameter("gic.reg-base-per-redistributor",
                  params.reg_base_per_redistributor);
    set_parameter("gic.GICD-alias", params.gicd_alias);
    set_parameter("gic.has-two-security-states",
                  params.has_two_security_states);
    set_parameter("gic.DS-fixed-to-zero", params.DS_fixed_to_zero);
    set_parameter("gic.IIDR", params.IIDR);
    set_parameter("gic.gicv2-only", params.gicv2_only);
    set_parameter("gic.STATUSR-implemented", params.STATUSR_implemented);
    set_parameter("gic.priority-bits", params.priority_bits_implemented);
    set_parameter("gic.GICD_ITARGETSR-RAZWI", params.itargets_razwi);
    set_parameter("gic.ICFGR-SGI-mask", params.icfgr_sgi_mask);
    set_parameter("gic.ICFGR-PPI-mask", params.icfgr_ppi_mask);
    set_parameter("gic.ICFGR-SPI-mask", params.icfgr_spi_mask);
    set_parameter("gic.ICFGR-SGI-reset", params.icfgr_sgi_reset);
    set_parameter("gic.ICFGR-PPI-reset", params.icfgr_ppi_reset);
    set_parameter("gic.ICFGR-SPI-reset", params.icfgr_spi_reset);
    set_parameter("gic.ICFGR-rsvd-bit", params.icfgr_ppi_rsvd_bit);
    set_parameter("gic.IGROUP-SGI-mask", params.igroup_sgi_mask);
    set_parameter("gic.IGROUP-PPI-mask", params.igroup_ppi_mask);
    set_parameter("gic.IGROUP-SGI-reset", params.igroup_sgi_reset);
    set_parameter("gic.IGROUP-PPI-reset", params.igroup_ppi_reset);
    set_parameter("gic.PPI-implemented-mask", params.ppi_implemented_mask);
    set_parameter("gic.SPI-count", params.spi_count);
    set_parameter("gic.lockable-SPI-count", params.lockable_spi_count);
    set_parameter("gic.IRI-ID-bits", params.iri_id_bits);
    set_parameter("gic.delay-redistributor-accesses",
                  params.delay_redistributor_accesses);
    set_parameter("gic.GICD_PIDR", params.gicd_pidr);
    set_parameter("gic.GICR_PIDR", params.gicr_pidr);
    set_parameter("gic.ITS-count", params.its_count);
    set_parameter("gic.ITS0-base", params.its0_base);
    set_parameter("gic.ITS1-base", params.its1_base);
    set_parameter("gic.ITS2-base", params.its2_base);
    set_parameter("gic.ITS3-base", params.its3_base);
    set_parameter("gic.GITS_PIDR", params.gits_pidr);
    set_parameter("gic.GITS_BASER0-type", params.gits_baser0_type);
    set_parameter("gic.GITS_BASER1-type", params.gits_baser1_type);
    set_parameter("gic.GITS_BASER2-type", params.gits_baser2_type);
    set_parameter("gic.GITS_BASER3-type", params.gits_baser3_type);
    set_parameter("gic.GITS_BASER4-type", params.gits_baser4_type);
    set_parameter("gic.GITS_BASER5-type", params.gits_baser5_type);
    set_parameter("gic.GITS_BASER6-type", params.gits_baser6_type);
    set_parameter("gic.GITS_BASER7-type", params.gits_baser7_type);
    set_parameter("gic.GITS_BASER0-entry-bytes",
                  params.gits_baser0_entry_bytes);
    set_parameter("gic.GITS_BASER1-entry-bytes",
                  params.gits_baser1_entry_bytes);
    set_parameter("gic.GITS_BASER2-entry-bytes",
                  params.gits_baser2_entry_bytes);
    set_parameter("gic.GITS_BASER3-entry-bytes",
                  params.gits_baser3_entry_bytes);
    set_parameter("gic.GITS_BASER4-entry-bytes",
                  params.gits_baser4_entry_bytes);
    set_parameter("gic.GITS_BASER5-entry-bytes",
                  params.gits_baser5_entry_bytes);
    set_parameter("gic.GITS_BASER6-entry-bytes",
                  params.gits_baser6_entry_bytes);
    set_parameter("gic.GITS_BASER7-entry-bytes",
                  params.gits_baser7_entry_bytes);
    set_parameter("gic.GITS_BASER0-indirect-RAZ",
                  params.gits_baser0_indirect_raz);
    set_parameter("gic.GITS_BASER1-indirect-RAZ",
                  params.gits_baser1_indirect_raz);
    set_parameter("gic.GITS_BASER2-indirect-RAZ",
                  params.gits_baser2_indirect_raz);
    set_parameter("gic.GITS_BASER3-indirect-RAZ",
                  params.gits_baser3_indirect_raz);
    set_parameter("gic.GITS_BASER4-indirect-RAZ",
                  params.gits_baser4_indirect_raz);
    set_parameter("gic.GITS_BASER5-indirect-RAZ",
                  params.gits_baser5_indirect_raz);
    set_parameter("gic.GITS_BASER6-indirect-RAZ",
                  params.gits_baser6_indirect_raz);
    set_parameter("gic.GITS_BASER7-indirect-RAZ",
                  params.gits_baser7_indirect_raz);
    set_parameter("gic.ITS-BASER-force-page-alignement",
                  params.its_baser_force_page_alignement);
    set_parameter("gic.processor-numbers", params.processor_numbers);
    set_parameter("gic.supports-shareability", params.supports_shareability);
    set_parameter("gic.A3-affinity-supported", params.a3_affinity_supported);
    set_parameter("gic.sgi-range-selector-support", params.SGI_RSS_support);
    set_parameter("gic.GICR_PROPBASER-read-only",
                  params.gicr_propbaser_read_only);
    set_parameter("gic.GICR_PROPBASER-reset-value",
                  params.gicr_propbaser_reset);
    set_parameter("gic.ITS-device-bits", params.its_device_bits);
    set_parameter("gic.ITS-entry-size", params.its_entry_size);
    set_parameter("gic.ITS-ID-bits", params.its_id_bits);
    set_parameter("gic.ITS-collection-ID-bits", params.its_collection_id_bits);
    set_parameter("gic.ITS-cumulative-collection-tables",
                  params.its_cumulative_collection_tables);
    set_parameter("gic.delay-ITS-accesses", params.delay_ITS_accesses);
    set_parameter("gic.local-SEIs", params.local_SEIs);
    set_parameter("gic.local-VSEIs", params.local_VSEIs);
    set_parameter("gic.ITS-use-physical-target-addresses",
                  params.ITS_use_physical_target_addresses);
    set_parameter("gic.ITS-hardware-collection-count",
                  params.ITS_hardware_collection_count);
    set_parameter("gic.ITS-MOVALL-update-collections",
                  params.ITS_MOVALL_update_collections);
    set_parameter("gic.ITS-TRANSLATE64R", params.ITS_TRANSLATE64R);
    set_parameter("gic.enable_protocol_checking",
                  params.enable_protocol_checking);
    set_parameter("gic.fixed-routed-spis", params.fixed_routed_spis);
    set_parameter("gic.irouter-default-mask", params.irouter_default_mask);
    if (params.irouter_default_reset != "") {
        set_parameter("gic.irouter-default-reset",
                      params.irouter_default_reset);
    }
    if (params.irouter_reset_values != "")
        set_parameter("gic.irouter-reset-values", params.irouter_reset_values);
    set_parameter("gic.irouter-mask-values", params.irouter_mask_values);
    set_parameter("gic.ITS-threaded-command-queue",
                  params.ITS_threaded_command_queue);
    set_parameter("gic.ITS-legacy-iidr-typer-offset",
                  params.ITS_legacy_iidr_typer_offset);
    set_parameter("gic.redistributor-threaded-sync",
                  params.redistributor_threaded_command_queue);
    set_parameter("gic.ignore-generate-sgi-when-no-are",
                  params.ignore_generate_sgi_when_no_are);
    set_parameter("gic.trace-speculative-lpi-property-update",
                  params.trace_speculative_lpi_property_updates);
    set_parameter("gic.virtual-lpi-support", params.virtual_lpi_support);
    set_parameter("gic.virtual-priority-bits", params.virtual_priority_bits);
    set_parameter("gic.LPI-cache-type", params.LPI_cache_type);
    set_parameter("gic.LPI-cache-check-data", params.LPI_cache_check_data);
    set_parameter("gic.DPG-bits-implemented", params.DPG_bits_implemented);
    set_parameter("gic.DPG-ARE-only", params.DPG_ARE_only);
    set_parameter("gic.ARE-fixed-to-one", params.ARE_fixed_to_one);
    set_parameter("gic.legacy-sgi-enable-rao", params.legacy_sgi_enable_rao);
    set_parameter("gic.PA_SIZE", params.pa_size);
    set_parameter("gic.MSI_IIDR", params.MSI_IIDR);
    set_parameter("gic.MSI_NS-frame0-base", params.MSI_NS_frame0_base);
    set_parameter("gic.MSI_NS-frame0-max-SPI", params.MSI_NS_frame0_max_SPI);
    set_parameter("gic.MSI_NS-frame0-min-SPI", params.MSI_NS_frame0_min_SPI);
    set_parameter("gic.MSI_NS-frame1-base", params.MSI_NS_frame1_base);
    set_parameter("gic.MSI_NS-frame1-max-SPI", params.MSI_NS_frame1_max_SPI);
    set_parameter("gic.MSI_NS-frame1-min-SPI", params.MSI_NS_frame1_min_SPI);
    set_parameter("gic.MSI_NS-frame2-base", params.MSI_NS_frame2_base);
    set_parameter("gic.MSI_NS-frame2-max-SPI", params.MSI_NS_frame2_max_SPI);
    set_parameter("gic.MSI_NS-frame2-min-SPI", params.MSI_NS_frame2_min_SPI);
    set_parameter("gic.MSI_NS-frame3-base", params.MSI_NS_frame3_base);
    set_parameter("gic.MSI_NS-frame3-max-SPI", params.MSI_NS_frame3_max_SPI);
    set_parameter("gic.MSI_NS-frame3-min-SPI", params.MSI_NS_frame3_min_SPI);
    set_parameter("gic.MSI_NS-frame4-base", params.MSI_NS_frame4_base);
    set_parameter("gic.MSI_NS-frame4-max-SPI", params.MSI_NS_frame4_max_SPI);
    set_parameter("gic.MSI_NS-frame4-min-SPI", params.MSI_NS_frame4_min_SPI);
    set_parameter("gic.MSI_NS-frame5-base", params.MSI_NS_frame5_base);
    set_parameter("gic.MSI_NS-frame5-max-SPI", params.MSI_NS_frame5_max_SPI);
    set_parameter("gic.MSI_NS-frame5-min-SPI", params.MSI_NS_frame5_min_SPI);
    set_parameter("gic.MSI_NS-frame6-base", params.MSI_NS_frame6_base);
    set_parameter("gic.MSI_NS-frame6-max-SPI", params.MSI_NS_frame6_max_SPI);
    set_parameter("gic.MSI_NS-frame6-min-SPI", params.MSI_NS_frame6_min_SPI);
    set_parameter("gic.MSI_NS-frame7-base", params.MSI_NS_frame7_base);
    set_parameter("gic.MSI_NS-frame7-max-SPI", params.MSI_NS_frame7_max_SPI);
    set_parameter("gic.MSI_NS-frame7-min-SPI", params.MSI_NS_frame7_min_SPI);
    set_parameter("gic.MSI_PIDR", params.MSI_PIDR);
    set_parameter("gic.MSI_S-frame0-base", params.MSI_S_frame0_base);
    set_parameter("gic.MSI_S-frame0-max-SPI", params.MSI_S_frame0_max_SPI);
    set_parameter("gic.MSI_S-frame0-min-SPI", params.MSI_S_frame0_min_SPI);
    set_parameter("gic.MSI_S-frame1-base", params.MSI_S_frame1_base);
    set_parameter("gic.MSI_S-frame1-max-SPI", params.MSI_S_frame1_max_SPI);
    set_parameter("gic.MSI_S-frame1-min-SPI", params.MSI_S_frame1_min_SPI);
    set_parameter("gic.MSI_S-frame2-base", params.MSI_S_frame2_base);
    set_parameter("gic.MSI_S-frame2-max-SPI", params.MSI_S_frame2_max_SPI);
    set_parameter("gic.MSI_S-frame2-min-SPI", params.MSI_S_frame2_min_SPI);
    set_parameter("gic.MSI_S-frame3-base", params.MSI_S_frame3_base);
    set_parameter("gic.MSI_S-frame3-max-SPI", params.MSI_S_frame3_max_SPI);
    set_parameter("gic.MSI_S-frame3-min-SPI", params.MSI_S_frame3_min_SPI);
    set_parameter("gic.MSI_S-frame4-base", params.MSI_S_frame4_base);
    set_parameter("gic.MSI_S-frame4-max-SPI", params.MSI_S_frame4_max_SPI);
    set_parameter("gic.MSI_S-frame4-min-SPI", params.MSI_S_frame4_min_SPI);
    set_parameter("gic.MSI_S-frame5-base", params.MSI_S_frame5_base);
    set_parameter("gic.MSI_S-frame5-max-SPI", params.MSI_S_frame5_max_SPI);
    set_parameter("gic.MSI_S-frame5-min-SPI", params.MSI_S_frame5_min_SPI);
    set_parameter("gic.MSI_S-frame6-base", params.MSI_S_frame6_base);
    set_parameter("gic.MSI_S-frame6-max-SPI", params.MSI_S_frame6_max_SPI);
    set_parameter("gic.MSI_S-frame6-min-SPI", params.MSI_S_frame6_min_SPI);
    set_parameter("gic.MSI_S-frame7-base", params.MSI_S_frame7_base);
    set_parameter("gic.MSI_S-frame7-max-SPI", params.MSI_S_frame7_max_SPI);
    set_parameter("gic.MSI_S-frame7-min-SPI", params.MSI_S_frame7_min_SPI);
    set_parameter("gic.outer-cacheability-support",
                  params.outer_cacheability_support);
    set_parameter("gic.wakeup-on-reset", params.wakeup_on_reset);
    set_parameter("gic.SPI-message-based-support", params.SPI_MBIS);
    set_parameter("gic.SPI-unimplemented", params.SPI_unimplemented);
    set_parameter("gic.IROUTER-IRM-RAZ-WI", params.irm_razwi);
    set_parameter("gic.common-lpi-configuration",
                  params.common_LPI_configuration);
    set_parameter("gic.single-set-support", params.single_set_support);
    set_parameter("gic.has_mpam", params.has_mpam);
    set_parameter("gic.mpam_partid_max", params.mpam_max_partid);
    set_parameter("gic.mpam_pmg_max", params.mpam_max_pmg);
    set_parameter("gic.output_attributes", params.output_attributes);
    set_parameter("gic.has_VPENDBASER-dirty-flag-on-load",
                  params.has_DirtyVLPIOnLoad);
    set_parameter("gic.allow-LPIEN-clear", params.allow_LPIEN_clear);
    set_parameter("gic.GICD-legacy-registers-as-reserved",
                  params.GICD_legacy_reg_reserved);
    set_parameter("gic.extended-spi-count", params.extended_spi_count);
    set_parameter("gic.extended-ppi-count", params.extended_ppi_count);
    set_parameter("gic.consolidators", params.consolidators);
}

Port &
SCGIC::gem5_getPort(const std::string &if_name, int idx)
{
    if (if_name == "reset") {
        return resetPort;
    } else if (if_name == "po_reset") {
        return poResetPort;
    } else {
        return scx_evs_GIC::gem5_getPort(if_name, idx);
    }
}

void
SCGIC::before_end_of_elaboration()
{
    scx_evs_GIC::before_end_of_elaboration();
    terminator.reset(new Terminator("terminator", redistributor));
}

GIC::GIC(const FastModelGICParams &params)
    : BaseGic(params),
      ambaM(params.sc_gic->amba_m, params.name + ".amba_m", -1),
      ambaS(params.sc_gic->amba_s, params.name + ".amba_s", -1),
      redistributors(params.port_redistributor_connection_count),
      scGIC(params.sc_gic)
{
    for (int i = 0; i < params.port_wake_request_connection_count; i++) {
        wakeRequestPorts.emplace_back(new IntSourcePin<GIC>(
            csprintf("%s.wakerequestport[%d]", name(), i), i, this));
        auto handler = [this, i](bool status) {
            auto &port = wakeRequestPorts[i];
            status ? port->raise() : port->lower();
        };
        scGIC->wakeRequests[i]->onChange(handler);
    }
}

Port &
GIC::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "amba_m") {
        return ambaM;
    } else if (if_name == "amba_s") {
        return ambaS;
    } else if (if_name == "redistributor") {
        auto &ptr = redistributors.at(idx);
        if (!ptr) {
            ptr.reset(new TlmGicInitiator(
                scGIC->redistributor[idx],
                csprintf("%s.redistributor[%d]", name(), idx), idx));
        }
        return *ptr;
    } else if (if_name == "wake_request") {
        return *wakeRequestPorts.at(idx);
    } else if (if_name == "reset" || if_name == "po_reset") {
        return scGIC->gem5_getPort(if_name, idx);
    } else {
        return BaseGic::getPort(if_name, idx);
    }
}

void
GIC::sendInt(uint32_t num)
{
    scGIC->signalInterrupt->spi(num - 32, true);
}

void
GIC::clearInt(uint32_t num)
{
    scGIC->signalInterrupt->spi(num - 32, false);
}

void
GIC::sendPPInt(uint32_t num, uint32_t cpu)
{
    scGIC->signalInterrupt->ppi(cpu, num, true);
}

void
GIC::clearPPInt(uint32_t num, uint32_t cpu)
{
    scGIC->signalInterrupt->ppi(cpu, num, false);
}

bool
GIC::supportsVersion(GicVersion version)
{
    if (scGIC->params().gicv2_only)
        return version == GicVersion::GIC_V2;
    return (version == GicVersion::GIC_V3) ||
           (version == GicVersion::GIC_V4 && scGIC->params().has_gicv4_1);
}

} // namespace fastmodel
} // namespace gem5
