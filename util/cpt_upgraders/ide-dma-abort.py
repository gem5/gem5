# Update IDE disk devices with dmaAborted flag
def upgrader(cpt):
    for sec in cpt.sections():
        # curSector only exists in IDE devices, so key on that attribute
        if cpt.has_option(sec, "curSector"):
            cpt.set(sec, "dmaAborted", "false")

legacy_version = 7
