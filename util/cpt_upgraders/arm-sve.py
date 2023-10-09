def upgrader(cpt):
    """
    Update the checkpoint to support initial SVE implemtation.
    The updater is taking the following steps.

    1) Set isa.haveSVE to false
    2) Set isa.sveVL to 1
    3) Add SVE misc registers in the checkpoint
    """
    if cpt.get("root", "isa", fallback="") == "arm":
        for sec in cpt.sections():
            import re

            # Search for all ISA sections
            if re.search(r".*sys.*\.cpu.*\.isa$", sec):
                # haveSVE = false
                cpt.set(sec, "haveSVE", "false")

                # sveVL (sve Vector Length in quadword) = 1
                # (This is a dummy value since haveSVE is set to false)
                cpt.set(sec, "sveVL", "1")

                # Updating SVE misc registers (dummy values)
                mr = cpt.get(sec, "miscRegs").split()
                if len(mr) == 820:
                    print(
                        "MISCREG_SVE registers already seems "
                        "to be inserted."
                    )
                else:
                    # Replace MISCREG_FREESLOT_1 with MISCREG_ID_AA64ZFR0_EL1
                    mr[-1] = 0

                    mr.append(0)
                    # Add dummy value for MISCREG_ZCR_EL3
                    mr.append(0)
                    # Add dummy value for MISCREG_ZCR_EL2
                    mr.append(0)
                    # Add dummy value for MISCREG_ZCR_EL12
                    mr.append(0)
                    # Add dummy value for MISCREG_ZCR_EL1
                    cpt.set(sec, "miscRegs", " ".join(str(x) for x in mr))


legacy_version = 15
