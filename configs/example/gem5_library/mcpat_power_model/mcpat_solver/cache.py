# SPDX-License-Identifier: BSD-3-Clause
"""McPAT cache, tag and buffer composition."""

from .mcpat_coefficients import _array_unit


class Cache:
    """A data CactiArraySpec (plus optional tag array and buffers) forming
    one leakage bucket. activation_energies() returns data-only Read/Write
    and separate TagRead/TagWrite (0.0 without a tag array) because callers
    use the tag energy both combined and standalone; leakage sums all parts."""

    def __init__(self, bucket_name, data_array, tag_array=None, buffers=None):
        self.bucket_name = bucket_name
        self.data_array = data_array
        self.tag_array = tag_array
        # Miss/fill/prefetch/writeback buffers: name -> CactiArraySpec, sharing
        # this cache's leakage bucket with their own Read/Write keys.
        self.buffers = buffers or {}

    def activation_energies(self, cacti_params):
        a = self.data_array.build(cacti_params)
        energies = {
            "Read": a.read,
            "Write": a.write,
            "TagRead": 0.0,
            "TagWrite": 0.0,
        }
        if self.tag_array is not None:
            t = self.tag_array.build(cacti_params)
            energies["TagRead"] = t.read
            energies["TagWrite"] = t.write
        for name, buf_array in self.buffers.items():
            b = buf_array.build(cacti_params)
            # CAM search energy; 0.0 for a non-FA/CAM buffer (.search exists
            # only when the array is fully associative).
            energies[name] = {
                "Read": b.read,
                "Write": b.write,
                "Search": getattr(b, "search", 0.0),
            }
        return energies

    def data_tag_energies(self, cacti_params):
        """Read/Write with the tag array folded in (BTB and predictor caches)."""
        e = self.activation_energies(cacti_params)
        return {
            "Read": e["Read"] + e["TagRead"],
            "Write": e["Write"] + e["TagWrite"],
        }

    def leakage_by_unit(self, cacti_params, core_ooo=None):
        units = {
            "Data": _array_unit(
                cacti_params,
                self.data_array.build(cacti_params, core_ooo=core_ooo),
            )
        }
        if self.tag_array is not None:
            units["Tag"] = _array_unit(
                cacti_params,
                self.tag_array.build(cacti_params, core_ooo=core_ooo),
            )
        for name, buf_array in self.buffers.items():
            units[name] = _array_unit(
                cacti_params, buf_array.build(cacti_params, core_ooo=core_ooo)
            )
        return units


# main.py's validated anchors, each with a companion *_TAG array.
