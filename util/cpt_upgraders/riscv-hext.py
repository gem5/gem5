def upgrader(cpt):
    """
    Update the checkpoint to support RVH implemtation.
    """
    import re

    for sec in cpt.sections():
        # # Upgrade interrupts to add HVIP reg
        res = re.search(r"(.*processor.*\.core.*)\.interrupts$", sec)
        if res and cpt.get(res.groups()[0] + ".isa", "isaName") == "riscv":
            # Only update for RISCV interrupts
            # Add hvip_ulong dummy value
            cpt.set(sec, "hvip_ulong", "0")

        res = re.search(r"(.*processor.*\.core.*)\.xc.*", sec)
        if res and cpt.get(res.groups()[0] + ".isa", "isaName") == "riscv":
            # Only update for RISCV XCs
            # Add hvip_ulong dummy value
            cpt.set(sec, "hvip_ulong", "0")

        # Update the miscregs
        if (
            re.search(r".*processor.*\.core.*\.isa$", sec)
            and cpt.get(sec, "isaName") == "riscv"
        ):
            # Updating RVH misc registers (dummy values)
            mr = cpt.get(sec, "miscRegFile").split()
            if len(mr) == 187:
                print(
                    "MISCREG_* RVH registers already seem " "to be inserted."
                )
            else:
                # Add dummy value for MISCREG_HVIP
                mr.insert(128, 0)
                # Add dummy value for MISCREG_MTINST
                mr.insert(128, 0)
                # Add dummy value for MISCREG_MTVAL2
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HSTATUS
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HEDELEG
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HIDELEG
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HCOUNTEREN
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HGEIE
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HTVAL
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HTINST
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HGEIP
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HENVCFG
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HGATP
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HCONTEXT
                mr.insert(128, 0)
                # Add dummy value for MISCREG_HTIMEDELTA
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSSTATUS
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSTVEC
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSSCRATCH
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSEPC
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSCAUSE
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSTVAL
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VSATP
                mr.insert(128, 0)
                # Add dummy value for MISCREG_VIRT
                mr.insert(128, 0)
                cpt.set(sec, "miscRegFile", " ".join(str(x) for x in mr))
