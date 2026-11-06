import bisect


class CactiBaseParams:
    def __init__(self, node):
        self._node = node
        self._node_um = node / 1000
        self._alpha_initialized = False

        """ Constants used by various components """
        self._cu_resistivity = 0.022
        self._bulk_cu_resistivity = 0.018
        self._perm_free_space = 8.854e-18
        self._supported_nodes = [22, 32, 45, 65, 90, 180]

    def validate_params(self):
        if self._node < 22 or self._node > 180:
            raise ValueError(f"Node {self._node}nm is not supported!\
                      Only technology nodes between 22nm and 180nm are\
                      supported by McPAT/CACTI!")

    def get_alpha(self):
        # validate_params() below has this same range check but is never
        # called on the CactiParams.__init__ -> init_tech_params -> get_alpha
        # path -- get_alpha() is the real choke point, and for node < 22 the
        # bisect branch silently negative-index-wraps _supported_nodes[-1]
        # (garbage, no exception), while node > 180 raises a bare IndexError.
        # Guard here so an out-of-range node fails loudly and consistently.
        if self._node < 22 or self._node > 180:
            raise ValueError(
                f"Node {self._node}nm is not supported! Only "
                "technology nodes between 22nm and 180nm are supported "
                "by McPAT/CACTI!"
            )
        if self._node in self._supported_nodes:
            alpha_lo = 1
            alpha_hi = 0
            tech_lo = self._node
            tech_hi = 0
        else:
            index = bisect.bisect_left(self._supported_nodes, self._node)
            tech_lo = int(self._supported_nodes[index])
            tech_hi = int(self._supported_nodes[index - 1])
            alpha_lo = (self._node - tech_hi) / (tech_lo - tech_hi)
            alpha_hi = (tech_lo - self._node) / (tech_lo - tech_hi)

        return alpha_lo, tech_lo, alpha_hi, tech_hi
