# http://stackoverflow.com/questions/15069127/python-configparser-module-\
# rename-a-section
def rename_section(cp, section_from, section_to):
    items = cp.items(section_from)
    cp.add_section(section_to)
    for item in items:
        cp.set(section_to, item[0], item[1])
    cp.remove_section(section_from)

# Checkpoint version F renames an internal member of Process class.
def upgrader(cpt):
    import re
    for sec in cpt.sections():
        fdm = 'FdMap'
        fde = 'FDEntry'
        if re.match('.*\.%s.*' % fdm, sec):
            rename = re.sub(fdm, fde, sec)
            split = re.split(fde, rename)

            # rename the section and add the 'mode' field
            rename_section(cpt, sec, rename)
            cpt.set(rename, 'mode', "0") # no proper value to set :(

            # add in entries 257 to 1023
            if split[1] == "0":
                for x in range(257, 1024):
                    seq = (split[0], fde, "%s" % x)
                    section = "".join(seq)
                    cpt.add_section(section)
                    cpt.set(section, 'fd', '-1')

legacy_version = 15
