# The ISA is now a separate SimObject, which means that we serialize
# it in a separate section instead of as a part of the ThreadContext.
def upgrader(cpt):
    isa = cpt.get('root','isa')
    isa_fields = {
        "arm" : ( "miscRegs" ),
        "sparc" : ( "asi", "tick", "fprs", "gsr", "softint", "tick_cmpr",
                    "stick", "stick_cmpr", "tpc", "tnpc", "tstate", "tt",
                    "tba", "pstate", "tl", "pil", "cwp", "gl", "hpstate",
                    "htstate", "hintp", "htba", "hstick_cmpr",
                    "strandStatusReg", "fsr", "priContext", "secContext",
                    "partId", "lsuCtrlReg", "scratchPad",
                    "cpu_mondo_head", "cpu_mondo_tail",
                    "dev_mondo_head", "dev_mondo_tail",
                    "res_error_head", "res_error_tail",
                    "nres_error_head", "nres_error_tail",
                    "tick_intr_sched",
                    "cpu", "tc_num", "tick_cmp", "stick_cmp", "hstick_cmp"),
        "x86" : ( "regVal" ),
        }

    isa_fields = isa_fields.get(isa, [])
    isa_sections = []
    for sec in cpt.sections():
        import re

        re_cpu_match = re.match('^(.*sys.*\.cpu[^.]*)\.xc\.(.+)$', sec)
        # Search for all the execution contexts
        if not re_cpu_match:
            continue

        if re_cpu_match.group(2) != "0":
            # This shouldn't happen as we didn't support checkpointing
            # of in-order and O3 CPUs.
            raise ValueError("Don't know how to migrate multi-threaded CPUs "
                             "from version 1")

        isa_section = []
        for fspec in isa_fields:
            for (key, value) in cpt.items(sec, raw=True):
                if key in isa_fields:
                    isa_section.append((key, value))

        name = "%s.isa" % re_cpu_match.group(1)
        isa_sections.append((name, isa_section))

        for (key, value) in isa_section:
            cpt.remove_option(sec, key)

    for (sec, options) in isa_sections:
        # Some intermediate versions of gem5 have empty ISA sections
        # (after we made the ISA a SimObject, but before we started to
        # serialize into a separate ISA section).
        if not cpt.has_section(sec):
            cpt.add_section(sec)
        else:
            if cpt.items(sec):
                raise ValueError("Unexpected populated ISA section in old "
                                 "checkpoint")

        for (key, value) in options:
            cpt.set(sec, key, value)

legacy_version = 4
