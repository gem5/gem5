# Add the ARM MISCREG TEEHBR
def upgrader(cpt):
    if cpt.get('root','isa') == 'arm':
        for sec in cpt.sections():
            import re
            # Search for all ISA sections
            if re.search('.*sys.*\.cpu.*\.isa$', sec):
                mr = cpt.get(sec, 'miscRegs').split()
                if len(mr) == 161:
                    print "MISCREG_TEEHBR already seems to be inserted."
                else:
                    mr.insert(51,0); # Add dummy value for MISCREG_TEEHBR
                    cpt.set(sec, 'miscRegs', ' '.join(str(x) for x in mr))

legacy_version = 8
