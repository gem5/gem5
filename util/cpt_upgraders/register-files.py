# Rename register files to their new systematic names.
def upgrader(cpt):
    is_arm = cpt.get("root", "isa", fallback="") == "arm"

    import re

    is_cpu = lambda sec: "intRegs" in cpt[sec]
    cpu_sections = filter(is_cpu, cpt.sections())

    for sec in cpu_sections:
        items = cpt[sec]

        # Almost all registers are 64 bits, except vectors and predicate
        # vectors in ARM.
        regval_bits = 64
        arm_vec_bits = 2048

        byte_bits = 8
        byte_mask = (0x1 << byte_bits) - 1

        # If there's vecRegs, create regs.vector_element from it.
        vec_regs = items.get("vecRegs")
        if vec_regs is not None:
            reg_vals = vec_regs.split()
            if is_arm:
                full_bits = arm_vec_bits
            else:
                full_bits = regval_bits
                reg_vals = ["0"]
            elem_bits = 32
            elem_mask = (0x1 << elem_bits) - 1

            bytes = []
            for full in reg_vals:
                full = int(full)
                for idx in range(full_bits // elem_bits):
                    # Extract one element.
                    elem = full & elem_mask
                    full = full >> elem_bits

                    # Treat the element as a RegVal value, even if it's
                    # fewer bits in the vector registers.
                    for chunk in range(regval_bits // byte_bits):
                        bytes.append(f"{elem & byte_mask}")
                        elem = elem >> byte_bits

            items["regs.vector_element"] = " ".join(bytes)

        name_map = {
            "floatRegs.i": "regs.floating_point",
            "vecRegs": "regs.vector",
            "vecPredRegs": "regs.vector_predicate",
            "intRegs": "regs.integer",
            "ccRegs": "regs.condition_code",
        }

        for old, new in name_map.items():
            if old in items:
                if is_arm and old in ("vecRegs", "vecPredRegs"):
                    reg_bits = 2048
                else:
                    reg_bits = regval_bits

                reg_vals = items[old].split()
                if not is_arm and old in ("vecRegs", "vecPredRegs"):
                    reg_vals = ["0"]

                bytes = []
                for reg in reg_vals:
                    reg = int(reg)
                    for chunk in range(reg_bits // byte_bits):
                        bytes.append(f"{reg & byte_mask}")
                        reg = reg >> byte_bits

                items[new] = " ".join(bytes)
                del items[old]

        items.setdefault("regs.condition_code", "")


legacy_version = 16
