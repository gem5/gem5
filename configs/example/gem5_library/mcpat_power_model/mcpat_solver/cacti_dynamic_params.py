from math import (
    ceil,
    log2,
    sqrt,
)

"""
Port of CACTI's DynamicParameter (parameter.cc) -- ENERGY-ONLY.

Given a memory-array configuration and a partition (Nspd, Ndwl, Ndbl, Ndcm,
Ndsam_lev_1, Ndsam_lev_2), this derives the physical geometry of the array
(subarray rows/cols, number of mats/subarrays, muxing degrees, data-out widths,
number of active mats, ...) exactly as CACTI does -- with NO timing/area.

The partition itself is chosen by CACTI's optimizer (which needs timing+area);
per project scope we take the 5 partition integers from a one-time CACTI run and
reproduce everything downstream here.

Only the non-fully-associative, non-pure-CAM, SRAM (non-DRAM) path is modelled,
which covers the McPAT core arrays (caches, BTB, RF, TLB, predictors, buffers).
Fully-associative / CAM / DRAM raise NotImplementedError, matching the rest of
the port.
"""

VBITSENSEMIN = 0.08
NUM_BITS_PER_ECC_B = 8.0
ADDRESS_BITS = 42
EXTRA_TAG_BITS = 5
MINSUBARRAYROWS = 16
MAXSUBARRAYROWS = 262144
MINSUBARRAYCOLS = 2
MAXSUBARRAYCOLS = 262144


def _log2_floor(n):
    # CACTI _log2 is a bit-shift floor-log2 on an integer.
    return int(n).bit_length() - 1 if n > 0 else 0


class CactiArrayConfig:
    """The subset of CACTI's InputParameter (g_ip) needed for the geometry.
    Sizes are in bytes; out_w in bits. Defaults match a simple single-port,
    non-ECC, normal-access SRAM cache/RAM."""

    def __init__(
        self,
        capacity,
        block_sz,
        assoc,
        nbanks,
        out_w,
        is_cache=True,
        is_tag=False,
        tag_w=0,
        specific_tag=False,
        fast_access=False,
        is_seq_acc=False,
        is_main_mem=False,
        num_dies=1,
        int_prefetch_w=1,
        burst_len=1,
        add_ecc=False,
        num_rw_ports=1,
        num_rd_ports=0,
        num_wr_ports=0,
        num_se_rd_ports=0,
        num_search_ports=0,
        pure_cam=False,
        wire_is_mat_type=0,
        data_assoc=None,
        wire_os_mat_type=None,
        wt_overhead=0,
    ):
        self.capacity = capacity  # bytes
        self.block_sz = block_sz  # bytes
        self.assoc = assoc  # associativity (1 => RAM/direct)
        # g_ip->data_assoc (io.cc:error_checking): tag_assoc = data_assoc = assoc
        # for normal/fast access mode, but data_assoc = 1 for SEQUENTIAL access mode
        # (McPAT sets this for some shared caches, e.g. L2/L3). CACTI's
        # DynamicParameter energy path reads g_ip->data_assoc everywhere (never
        # g_ip->assoc directly) -- defaults to assoc, override for seq-access arrays.
        self.data_assoc = data_assoc if data_assoc is not None else assoc
        self.nbanks = nbanks
        self.out_w = out_w  # output/input bus width, bits
        # Port counts (widen the memory cell). RAMs/RFs often have many read/write
        # ports; a plain single-port cache uses one read/write port.
        self.num_rw_ports = num_rw_ports
        self.num_rd_ports = num_rd_ports
        self.num_wr_ports = num_wr_ports
        self.num_se_rd_ports = num_se_rd_ports
        # Search ports (g_ip->num_search_ports, SCHP) -- only used by FA/CAM
        # arrays. Callers pass the POST-error_checking port counts (io.cc:1476
        # mutates ERP=SCHP when RWP==ERP==0 and SCHP>0 for FA/CAM, e.g. ITLB/
        # DTLB); this port does not re-derive that mutation, matching the
        # project's "partition ints/ports taken as given" scope decision.
        self.num_search_ports = num_search_ports
        # g_ip->pure_cam: never set true anywhere in McPAT (grepped); kept only
        # as an explicit guard input, never actually exercised.
        self.pure_cam = pure_cam
        # Mat wire layer (CACTI g_ip->wire_is_mat_type, used inside a mat -- bit/
        # sa-mux decoder output, predecode output, precharge driver, subarray
        # output wire). McPAT sets this to 0 (local, 2.5F pitch) for core arrays
        # (processor.cc:813); standalone CACTI cfgs that request "semi-global" use
        # 1 (4F pitch), non-embedded configs use 2 (8F, global).
        self.wire_is_mat_type = wire_is_mat_type
        # H-tree (outside-mat) wire layer (CACTI g_ip->wire_os_mat_type), used by
        # CactiHtree's wire-model constant. Usually equals wire_is_mat_type
        # (core.cc:813/818), but McPAT's shared-cache path (sharedcache.cc:75-83)
        # sets wire_os_mat_type=1 (semi-global) for embedded L2/L3 while
        # wire_is_mat_type stays 0 (local) -- defaults to wire_is_mat_type,
        # override explicitly for shared-cache arrays.
        self.wire_os_mat_type = (
            wire_os_mat_type
            if wire_os_mat_type is not None
            else wire_is_mat_type
        )
        # H-tree wire delay-overhead point (CACTI g_ip->wt: Global=0, Global_30=30
        # %). McPAT sets this from the same Embedded flag as wire_is/os_mat_type
        # (core.cc:810-821, sharedcache.cc:75-83): Global_30 for embedded designs,
        # Global (0) otherwise.
        self.wt_overhead = wt_overhead
        self.is_cache = is_cache
        self.is_tag = is_tag
        self.tag_w = tag_w
        self.specific_tag = specific_tag
        self.fast_access = fast_access
        # g_ip->is_seq_acc (io.cc:1520-1533, set from access_mode==1 in
        # error_checking): SEQUENTIAL access -- the tag array is read first
        # and only then, on a hit, the data array. Purely a tag+data
        # COMBINATION property: it selects uca_org_t::find_delay's
        # `tag->access_time + data->access_time` branch (cacti_interface.cc:90)
        # instead of the normal-access max()-of-two-paths formula, and is read
        # nowhere in the geometry/energy path (which reads data_assoc, the
        # other thing access_mode==1 sets). McPAT's only is_cache && assoc>0
        # call site for it is the unified shared cache -- L2/L3/Directorycache,
        # sharedcache.cc:127 `interface_ip.access_mode = 1` -- every other
        # access_mode==1 site (core.cc's RRAT/free list/RegFile/ROB/IQ/LSQ,
        # memoryctrl.cc:373) is pure_ram or fully_assoc, where find_delay takes
        # the data-only branch and this flag cannot matter.
        self.is_seq_acc = is_seq_acc
        self.is_main_mem = is_main_mem
        self.num_dies = num_dies
        self.int_prefetch_w = int_prefetch_w
        self.burst_len = burst_len
        self.add_ecc = add_ecc


class CactiDynamicParameter:
    def __init__(
        self,
        cfg: CactiArrayConfig,
        Nspd,
        Ndwl,
        Ndbl,
        Ndcm,
        Ndsam_lev_1,
        Ndsam_lev_2,
    ):
        self.cfg = cfg
        self.Nspd = Nspd
        self.Ndwl = Ndwl
        self.Ndbl = Ndbl
        self.Ndcm = Ndcm
        self.Ndsam_lev_1 = Ndsam_lev_1
        self.Ndsam_lev_2 = Ndsam_lev_2
        self.valid = False

        # Fully associative arrays combine CAM tags with RAM data. Pure CAM
        # arrays contain only the CAM entries.
        self.pure_cam = cfg.pure_cam
        self.fully_assoc = cfg.assoc == 0 and not cfg.pure_cam
        # parameter.cc:218-229 -- for fully_assoc/pure_cam, Ndwl/Ndcm/Nspd/
        # Ndsam_lev_1/Ndsam_lev_2 are all fixed to 1 and Ndbl must be >= 2;
        # any other combination is invalid before ANY of the row/col/mats
        # logic below runs. This port's _init_fully_assoc previously never
        # checked this (each of those 5 dimensions is either read directly
        # downstream by CactiMat regardless of is_fa, e.g. Ndsam_lev_1, or
        # simply irrelevant to _init_fully_assoc's own field derivations,
        # e.g. Ndwl/Ndcm/Nspd), so every (Ndwl,Ndcm,Nspd,Ndsam1,Ndsam2)
        # combination silently passed as "valid" -- undetected by any prior
        # regression because every FA/CAM structure in the trace corpus is
        # always fed its single real (already-Ndwl=Ndcm=Nspd=Ndsam1=Ndsam2=1)
        # winning partition directly as a given input (never swept), so this
        # gap was invisible until cacti_partition_search.py's Phase-35+
        # exhaustive sweep actually tried Ndwl!=1 etc. combinations for an
        # FA array for the first time (measured: 361584/656100 (Nspd,Ndwl,
        # Ndbl,Ndcm,Ndsam1,Ndsam2) combos wrongly passed dp.valid for
        # InstIssueQueue@256 before this fix -- verified against 2899 real
        # FA/CAM trace records, all of which already satisfy this guard, so
        # this fix cannot regress any existing validated FA/CAM result).
        if (self.fully_assoc or self.pure_cam) and (
            Ndwl != 1
            or Ndcm != 1
            or Nspd < 1
            or Nspd > 1
            or Ndsam_lev_1 != 1
            or Ndsam_lev_2 != 1
            or Ndbl < 2
        ):
            return
        if cfg.is_main_mem:
            # Main-memory (DRAM) arrays branch separately at parameter.cc:497,
            # 575, 584, 647 and uca.cc:167/213 -- none of that is ported. Every
            # McPAT on-chip array validated so far has is_main_mem=False; guard
            # explicitly instead of silently taking the on-chip formula.
            raise NotImplementedError(
                "main-memory (is_main_mem) arrays are not modelled"
            )

        capacity_per_die = cfg.capacity / cfg.num_dies  # bytes

        # tag_assoc == assoc always (io.cc:error_checking: tag_assoc = A regardless
        # of seq_access; only data_assoc drops to 1 for sequential access).
        tag_assoc = cfg.assoc

        if self.fully_assoc or self.pure_cam:
            self._init_fully_assoc(
                cfg,
                Nspd,
                Ndwl,
                Ndbl,
                Ndcm,
                Ndsam_lev_1,
                Ndsam_lev_2,
                capacity_per_die,
            )
            return

        # ---- tagbits (parameter.cc:248-260) ----
        if cfg.is_tag:
            if cfg.specific_tag:
                self.tagbits = cfg.tag_w
            else:
                self.tagbits = (
                    ADDRESS_BITS
                    + EXTRA_TAG_BITS
                    - _log2_floor(capacity_per_die)
                    + _log2_floor(tag_assoc * 2 - 1)
                    - _log2_floor(cfg.nbanks)
                )
            self.tagbits = ((self.tagbits + 3) >> 2) << 2
        else:
            self.tagbits = 0

        # ---- subarray rows / cols (parameter.cc:262-271, tag vs data) ----
        if cfg.is_tag:
            self.num_r_subarray = int(
                ceil(
                    capacity_per_die
                    / (cfg.nbanks * cfg.block_sz * tag_assoc * Ndbl * Nspd)
                )
            )
            self.num_c_subarray = int(
                ceil(self.tagbits * tag_assoc * Nspd / Ndwl)
            )
        else:
            self.num_r_subarray = int(
                ceil(
                    capacity_per_die
                    / (
                        cfg.nbanks
                        * cfg.block_sz
                        * cfg.data_assoc
                        * Ndbl
                        * Nspd
                    )
                )
            )
            self.num_c_subarray = int(
                ceil(8 * cfg.block_sz * cfg.data_assoc * Nspd / Ndwl)
            )

        if (
            self.num_r_subarray < MINSUBARRAYROWS
            or self.num_r_subarray > MAXSUBARRAYROWS
        ):
            return
        if (
            self.num_c_subarray < MINSUBARRAYCOLS
            or self.num_c_subarray > MAXSUBARRAYCOLS
        ):
            return

        self.num_subarrays = Ndwl * Ndbl

        # ---- mats (parameter.cc:479) ----
        self.num_mats_h_dir = max(Ndwl // 2, 1)
        self.num_mats_v_dir = max(Ndbl // 2, 1)
        self.num_mats = self.num_mats_h_dir * self.num_mats_v_dir

        # ---- bitline / sense-amp muxing (SRAM, parameter.cc:418) ----
        self.deg_bl_muxing = Ndcm
        # V_b_sense uses sram_cell.Vdd; single-Vdd port -> peripheral Vdd, supplied
        # by the caller via set_v_b_sense (kept out of pure geometry).
        self.V_b_sense = None

        # data-out bits per mat (parameter.cc:482)
        self.num_do_b_mat = max(
            (self.num_subarrays // self.num_mats)
            * self.num_c_subarray
            // (self.deg_bl_muxing * Ndsam_lev_1 * Ndsam_lev_2),
            1,
        )
        if self.num_do_b_mat < (self.num_subarrays // self.num_mats):
            return

        # ---- sense-amp level-1 muxing accounting for associativity (parameter.cc:
        # 493-531, g_ip->data_assoc). is_tag is checked BEFORE fast_access -- the
        # tag branch never routes through fast_access at all. ----
        if cfg.is_tag:
            self.num_do_b_subbank = self.tagbits * tag_assoc
            # parameter.cc:525-528 -- a tag mat must be able to deliver a whole
            # tag; a geometry whose per-mat data-out width is narrower than
            # tagbits is rejected outright. Previously missing here, which let
            # this port enumerate tag candidates real CACTI never produces
            # (measured at 180nm/HP on the 1MiB L2: 2538 tag geometries vs real
            # CACTI's 1694). Those extra candidates are invisible to the ed=2
            # baseline ranking -- its cost reads only access_time and dynamic,
            # whose per-side minima were unaffected -- but they depress
            # min_cyc/min_leakage, which ONLY Layer 3 weights (cycle_time_wt
            # 1000, leakage_power_wt 10, area_wt 10; all zero at ed=2), so the
            # gap stayed latent until search_partition_layer3 exercised them.
            if self.num_do_b_mat < self.tagbits:
                return
            self.deg_sa_mux_l1_non_assoc = Ndsam_lev_1
        elif cfg.fast_access:
            self.num_do_b_subbank = cfg.out_w * cfg.data_assoc
            self.deg_sa_mux_l1_non_assoc = Ndsam_lev_1
        else:
            self.num_do_b_subbank = cfg.out_w
            self.deg_sa_mux_l1_non_assoc = Ndsam_lev_1 // cfg.data_assoc
            if self.deg_sa_mux_l1_non_assoc < 1:
                return
        self.deg_senseamp_muxing_non_associativity = (
            self.deg_sa_mux_l1_non_assoc
        )

        # ---- active mats in the horizontal (data-out) direction ----
        self.num_act_mats_hor_dir = self.num_do_b_subbank // self.num_do_b_mat
        if self.num_act_mats_hor_dir == 0:
            return
        if self.num_act_mats_hor_dir > self.num_mats_h_dir:
            return

        # num_do_b_mat is recomputed for tag arrays using the num_act_mats_hor_dir
        # just derived above from the generic formula (parameter.cc:566-573) --
        # a genuine two-pass computation, do not reorder relative to the block above.
        if cfg.is_tag:
            self.num_do_b_mat = tag_assoc // self.num_act_mats_hor_dir
            self.num_do_b_subbank = (
                self.num_act_mats_hor_dir * self.num_do_b_mat
            )

        # data-in bits per mat (parameter.cc:599-613, g_ip->data_assoc)
        if cfg.is_tag:
            self.num_di_b_mat = self.tagbits
        elif cfg.fast_access:
            self.num_di_b_mat = self.num_do_b_mat // cfg.data_assoc
        else:
            self.num_di_b_mat = self.num_do_b_mat

        # number of way-select signals per mat (parameter.cc:689-691,
        # g_ip->data_assoc). A normal (non-fast, non-tag) set-associative data
        # array routes the way-select signals into the sa-mux-level-1 decoder, so
        # that decoder exists even when Ndsam1/assoc == 1.
        if (not cfg.is_tag) and cfg.data_assoc > 1 and not cfg.fast_access:
            self.number_way_select_signals_mat = cfg.data_assoc
        else:
            self.number_way_select_signals_mat = 0

        self.num_subarrays_per_mat = self.num_subarrays // self.num_mats
        self.num_subarrays_per_row = max(1, Ndwl // self.num_mats_h_dir)

        # ---- bank-level address/data bit widths (parameter.cc:634-668), needed
        # for the intra-bank (mat) and inter-bank (UCA) H-trees. ----
        num_addr_b_row_dec = _log2_floor(self.num_r_subarray)
        number_subbanks = self.num_mats // self.num_act_mats_hor_dir
        self.number_subbanks_decode = _log2_floor(number_subbanks)
        self.number_addr_bits_mat = (
            num_addr_b_row_dec
            + _log2_floor(self.deg_bl_muxing)
            + _log2_floor(self.deg_senseamp_muxing_non_associativity)
            + _log2_floor(Ndsam_lev_2)
        )

        # parameter.cc:660-669. Note num_do_b_bank_per_port uses data_assoc here,
        # NOT tag_assoc, even in the is_tag branch (parameter.cc:663) -- unlike
        # every other tag formula above, which does use tag_assoc.
        if cfg.is_tag:
            self.num_di_b_bank_per_port = self.tagbits
            self.num_do_b_bank_per_port = cfg.data_assoc
        else:
            self.num_di_b_bank_per_port = cfg.out_w + cfg.data_assoc
            self.num_do_b_bank_per_port = cfg.out_w

        # ECC overhead (parameter.cc:695-702). McPAT enables ECC for every array
        # (processor.cc: add_ecc_b_ = true), adding one check bit per 8 data bits
        # to the data-out/-in widths. num_act_mats_hor_dir is computed ABOVE from
        # the pre-ECC widths (CACTI applies ECC afterwards), so it is unaffected.
        # The subarray column ECC is applied inside CactiSubarray.
        self.add_ecc = cfg.add_ecc
        if cfg.add_ecc:
            self.num_do_b_mat += int(
                ceil(self.num_do_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_di_b_mat += int(
                ceil(self.num_di_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_do_b_subbank += int(
                ceil(self.num_do_b_subbank / NUM_BITS_PER_ECC_B)
            )
            self.num_di_b_bank_per_port += int(
                ceil(self.num_di_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )
            self.num_do_b_bank_per_port += int(
                ceil(self.num_do_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )

        self.valid = True

    def _init_fully_assoc(
        self,
        cfg,
        Nspd,
        Ndwl,
        Ndbl,
        Ndcm,
        Ndsam_lev_1,
        Ndsam_lev_2,
        capacity_per_die,
    ):
        # Port of parameter.cc's `fully_assoc` (not pure_cam) branch: one combined
        # CAM(tag)+RAM(data) array, always is_tag==false (Ucache.cc:863-869 --
        # tag_array2 is NULL, everything goes through data_array2). e.g. ITLB/DTLB.

        # ---- tagbits (parameter.cc:310-318) ----
        if cfg.specific_tag:
            self.tagbits = cfg.tag_w
        else:
            self.tagbits = (
                ADDRESS_BITS + EXTRA_TAG_BITS - _log2_floor(cfg.block_sz)
            )
        self.tagbits = ((self.tagbits + 3) >> 2) << 2
        if self.pure_cam:
            self.tagbits = int(ceil(cfg.tag_w / 8.0) * 8)
        self.add_ecc = cfg.add_ecc

        # ---- tag / data subarray rows & cols (parameter.cc:320-333). Note
        # tag_num_r_subarray TRUNCATES (no ceil), unlike every other subarray-size
        # formula in this port, and MINSUBARRAYROWS is NOT checked for FA. ----
        self.tag_num_r_subarray = int(
            capacity_per_die / (cfg.nbanks * cfg.block_sz * Ndbl)
        )
        if self.pure_cam:
            self.tag_num_r_subarray = int(
                ceil(
                    capacity_per_die / (cfg.nbanks * self.tagbits / 8.0 * Ndbl)
                )
            )
        self.tag_num_c_subarray = int(ceil(self.tagbits * Nspd / Ndwl))
        if (
            self.tag_num_r_subarray == 0
            or self.tag_num_r_subarray > MAXSUBARRAYROWS
        ):
            return
        if (
            self.tag_num_c_subarray < MINSUBARRAYCOLS
            or self.tag_num_c_subarray > MAXSUBARRAYCOLS
        ):
            return

        self.data_num_r_subarray = self.tag_num_r_subarray
        self.data_num_c_subarray = 8 * cfg.block_sz
        if (
            self.data_num_r_subarray == 0
            or self.data_num_r_subarray > MAXSUBARRAYROWS
        ):
            return
        if (
            self.data_num_c_subarray < MINSUBARRAYCOLS
            or self.data_num_c_subarray > MAXSUBARRAYCOLS
        ):
            return

        if self.pure_cam:
            self.data_num_c_subarray = 0
        self.num_r_subarray = self.tag_num_r_subarray  # parameter.cc:333
        # Pre-ECC combined column count -- CactiSubarray's FA branch derives its
        # own (ECC'd, per-part) `_cols` from `tag_num_c_subarray`/`data_num_c_subarray`
        # directly and ignores this; kept only so set_dynamic_parameters/
        # CactiComponent.__init__ have a `num_c_subarray` to read uniformly with
        # the non-FA path.
        self.num_c_subarray = (
            self.tag_num_c_subarray + self.data_num_c_subarray
        )
        self.num_subarrays = Ndwl * Ndbl

        # ---- mats (parameter.cc:447-464): switch(Ndbl), NOT Ndwl/2, Ndbl/2. ----
        if Ndbl == 1 or Ndbl == 2:
            self.num_mats_h_dir = 1
            self.num_mats_v_dir = 1
        else:
            self.num_mats_h_dir = int(sqrt(Ndbl / 4.0))
            self.num_mats_v_dir = int(Ndbl / 4.0 / self.num_mats_h_dir)
        self.num_mats = self.num_mats_h_dir * self.num_mats_v_dir

        # deg_bl_muxing is fixed to 1 for FA (parameter.cc:431); Ndcm is fixed to 1
        # by the partition itself, so this is the same value either way.
        self.deg_bl_muxing = Ndcm
        self.V_b_sense = None

        # ---- data-out bits per mat (parameter.cc:468-469, fully_assoc case) ----
        self.num_so_b_mat = self.data_num_c_subarray
        self.num_do_b_mat = self.data_num_c_subarray + self.tagbits

        # ---- subbank widths + sa-mux-lev-1 muxing (parameter.cc:535-546) ----
        self.num_so_b_subbank = 8 * cfg.block_sz
        self.num_do_b_subbank = self.num_so_b_subbank + self.tag_num_c_subarray
        if self.pure_cam:
            self.num_so_b_mat = int(
                ceil(log2(self.num_r_subarray))
                + ceil(log2(self.num_subarrays))
            )
            self.num_do_b_mat = self.tagbits
            self.num_so_b_subbank = self.num_so_b_mat
            self.num_do_b_subbank = self.tag_num_c_subarray
        self.deg_sa_mux_l1_non_assoc = 1
        self.deg_senseamp_muxing_non_associativity = (
            self.deg_sa_mux_l1_non_assoc
        )

        # ---- active mats (parameter.cc:553): always 1 for fully_assoc/pure_cam ----
        self.num_act_mats_hor_dir = 1

        # ---- data-in / search-in bits per mat (parameter.cc:617-623) ----
        self.num_di_b_mat = self.num_do_b_mat
        self.num_si_b_mat = self.tagbits
        self.num_di_b_subbank = self.num_di_b_mat * self.num_act_mats_hor_dir
        self.num_si_b_subbank = self.num_si_b_mat

        # Way-select signals never apply to FA (is_tag is always false here, but
        # data_assoc defaults to 0 for a fully_assoc cfg, so the generic
        # `data_assoc > 1` guard below is false regardless).
        if (not cfg.is_tag) and cfg.data_assoc > 1 and not cfg.fast_access:
            self.number_way_select_signals_mat = cfg.data_assoc
        else:
            self.number_way_select_signals_mat = 0

        self.num_subarrays_per_mat = self.num_subarrays // self.num_mats
        self.num_subarrays_per_row = max(1, Ndwl // self.num_mats_h_dir)

        # ---- bank-level address bits (parameter.cc:635-656): FA/pure_cam adds
        # log2(num_subarrays/num_mats) to the row-decode address width (this is
        # DISTINCT from Mat's own `num_dec_signals += log2(num_subarrays_per_mat)`
        # bump in mat.cc:223-224 -- two separate additions). ----
        num_addr_b_row_dec = _log2_floor(self.num_r_subarray) + _log2_floor(
            self.num_subarrays // self.num_mats
        )
        number_subbanks = self.num_mats // self.num_act_mats_hor_dir
        self.number_subbanks_decode = _log2_floor(number_subbanks)
        self.number_addr_bits_mat = (
            num_addr_b_row_dec
            + _log2_floor(self.deg_bl_muxing)
            + _log2_floor(self.deg_senseamp_muxing_non_associativity)
            + _log2_floor(Ndsam_lev_2)
        )

        # ---- bank per-port widths (parameter.cc:673-679, fully_assoc case) ----
        self.num_di_b_bank_per_port = cfg.out_w + self.tagbits
        self.num_si_b_bank_per_port = self.tagbits
        self.num_do_b_bank_per_port = cfg.out_w + self.tagbits
        self.num_so_b_bank_per_port = cfg.out_w

        if self.pure_cam:
            self.num_di_b_bank_per_port = self.tagbits
            self.num_do_b_bank_per_port = self.tagbits
            self.num_so_b_bank_per_port = int(
                ceil(log2(self.num_r_subarray))
                + ceil(log2(self.num_subarrays))
            )

        # ---- ECC (parameter.cc:695-709): FA/CAM additionally bumps the search
        # in/out widths, which non-FA arrays never carry. ----
        if cfg.add_ecc:
            self.num_do_b_mat += int(
                ceil(self.num_do_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_di_b_mat += int(
                ceil(self.num_di_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_di_b_subbank += int(
                ceil(self.num_di_b_subbank / NUM_BITS_PER_ECC_B)
            )
            self.num_do_b_subbank += int(
                ceil(self.num_do_b_subbank / NUM_BITS_PER_ECC_B)
            )
            self.num_di_b_bank_per_port += int(
                ceil(self.num_di_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )
            self.num_do_b_bank_per_port += int(
                ceil(self.num_do_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )

            self.num_so_b_mat += int(
                ceil(self.num_so_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_si_b_mat += int(
                ceil(self.num_si_b_mat / NUM_BITS_PER_ECC_B)
            )
            self.num_si_b_subbank += int(
                ceil(self.num_si_b_subbank / NUM_BITS_PER_ECC_B)
            )
            self.num_so_b_subbank += int(
                ceil(self.num_so_b_subbank / NUM_BITS_PER_ECC_B)
            )
            self.num_si_b_bank_per_port += int(
                ceil(self.num_si_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )
            self.num_so_b_bank_per_port += int(
                ceil(self.num_so_b_bank_per_port / NUM_BITS_PER_ECC_B)
            )

        self.valid = True

    def set_v_b_sense(self, sram_cell_vdd):
        self.V_b_sense = max(0.05 * sram_cell_vdd, VBITSENSEMIN)
