# This upgrader renames section "Globals" as "root.globals".
def upgrader(cpt):
    import re

    for sec in cpt.sections():
        if re.match("Globals", sec):
            # rename the section
            items = cpt.items(sec)
            cpt.add_section("root.globals")
            for item in items:
                cpt.set("root.globals", item[0], item[1])
            cpt.remove_section(sec)


legacy_version = 16
