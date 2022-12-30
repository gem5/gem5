# Add the perfLevel variable in the clock domain and voltage domain simObjects
def upgrader(cpt):
    for sec in cpt.sections():
        import re

        if re.match("^.*sys.*[._]clk_domain$", sec):
            # Make _perfLevel equal to 0 which means best performance
            cpt.set(sec, "_perfLevel", " ".join("0"))
        elif re.match("^.*sys.*[._]voltage_domain$", sec):
            # Make _perfLevel equal to 0 which means best performance
            cpt.set(sec, "_perfLevel", " ".join("0"))
        else:
            continue


legacy_version = 11
