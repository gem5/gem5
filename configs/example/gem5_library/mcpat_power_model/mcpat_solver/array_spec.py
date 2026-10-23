# SPDX-License-Identifier: BSD-3-Clause
"""Fixed array organization shared by CACTI and McPAT composition."""

from .cacti_component import (
    CactiArrayST,
    CactiMat,
    CactiUCA,
)
from .cacti_dynamic_params import (
    CactiArrayConfig,
    CactiDynamicParameter,
)


class CactiArraySpec:
    """One CACTI array (assoc=0 selects the FA/CAM path): configuration plus
    a fixed partition (Nspd/Ndwl/Ndbl/Ndcm/Ndsam_lev_1/2) and mat area given
    as inputs; cacti_partition_search.search_partition() finds them."""

    def __init__(
        self,
        name,
        capacity,
        block_sz,
        assoc,
        nbanks,
        out_w,
        *,
        add_ecc=True,
        wire_is_mat_type=0,
        wire_os_mat_type=None,
        wt_overhead=0,
        data_assoc=None,
        is_seq_acc=False,
        is_tag=False,
        tag_w=0,
        specific_tag=False,
        num_rw_ports=1,
        num_rd_ports=0,
        num_wr_ports=0,
        num_se_rd_ports=0,
        num_search_ports=0,
        Nspd=1,
        Ndwl=1,
        Ndbl=1,
        Ndcm=1,
        Ndsam_lev_1=1,
        Ndsam_lev_2=1,
        mat_area_w=0.0,
        mat_area_h=0.0,
        device_ty="core",
        core_ooo=False,
    ):
        self.name = name
        self._geometry = dict(
            capacity=capacity,
            block_sz=block_sz,
            assoc=assoc,
            nbanks=nbanks,
            out_w=out_w,
            is_cache=True,
            add_ecc=add_ecc,
            wire_is_mat_type=wire_is_mat_type,
            wire_os_mat_type=wire_os_mat_type,
            wt_overhead=wt_overhead,
            data_assoc=data_assoc,
            is_seq_acc=is_seq_acc,
            is_tag=is_tag,
            tag_w=tag_w,
            specific_tag=specific_tag,
            num_rw_ports=num_rw_ports,
            num_rd_ports=num_rd_ports,
            num_wr_ports=num_wr_ports,
            num_se_rd_ports=num_se_rd_ports,
            num_search_ports=num_search_ports,
        )
        self._organization = dict(
            Nspd=Nspd,
            Ndwl=Ndwl,
            Ndbl=Ndbl,
            Ndcm=Ndcm,
            Ndsam_lev_1=Ndsam_lev_1,
            Ndsam_lev_2=Ndsam_lev_2,
        )
        self._mat_area_w = mat_area_w
        self._mat_area_h = mat_area_h
        self._device_ty = device_ty
        self._core_ooo = core_ooo

    @property
    def _cfg_kwargs(self):
        return dict(self._geometry)

    @property
    def _dp_kwargs(self):
        return dict(self._organization)

    def build(self, cacti_params, core_ooo=None):
        key = (id(cacti_params), core_ooo)
        if not hasattr(self, "_built_cache"):
            self._built_cache = {}
        if key not in self._built_cache:
            cfg = CactiArrayConfig(**self._cfg_kwargs)
            self._built_cache[key] = (
                cacti_params,
                build_array(
                    self.name,
                    cfg,
                    cacti_params,
                    tuple(self._dp_kwargs.values()),
                    (self._mat_area_w, self._mat_area_h),
                    device_ty=self._device_ty,
                    core_ooo=self._core_ooo if core_ooo is None else core_ooo,
                ),
            )
        return self._built_cache[key][1]

    def replace(self, **cfg_changes):
        """Copy with the given config fields (capacity, tag_w, ...) changed."""
        cfg = dict(self._cfg_kwargs, **cfg_changes)
        del cfg["is_cache"]
        return CactiArraySpec(
            self.name,
            **cfg,
            **self._dp_kwargs,
            mat_area_w=self._mat_area_w,
            mat_area_h=self._mat_area_h,
            device_ty=self._device_ty,
            core_ooo=self._core_ooo,
        )


def build_array(
    name,
    config,
    parameters,
    integers,
    mat_area,
    *,
    device_ty="core",
    core_ooo=False,
):
    """One geometry-to-coefficients path; retain source evaluation order."""
    nspd, ndwl, ndbl, ndcm, ndsam1, ndsam2 = integers
    dp = CactiDynamicParameter(
        config,
        Nspd=nspd,
        Ndwl=ndwl,
        Ndbl=ndbl,
        Ndcm=ndcm,
        Ndsam_lev_1=ndsam1,
        Ndsam_lev_2=ndsam2,
    )
    if not dp.valid:
        raise ValueError(f"{name}: partition is invalid for geometry")
    mat = CactiMat(
        name, parameters, dp, mat_area_w=mat_area[0], mat_area_h=mat_area[1]
    )
    uca = CactiUCA(mat, dp, config.nbanks)
    return CactiArrayST(
        uca, config.nbanks, device_ty=device_ty, core_ooo=core_ooo
    )
