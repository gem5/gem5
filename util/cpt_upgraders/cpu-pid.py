def upgrader(cpt):
    for sec in cpt.sections():
        import re

        # Search for a CPUs
        if re.search(".*sys.*cpu", sec):
            try:
                junk = cpt.get(sec, "instCnt")
                cpt.set(sec, "_pid", "0")
            except ConfigParser.NoOptionError:
                pass


legacy_version = 3
