# Add the ARM CONTEXTIDR_EL2 miscreg.
def upgrader(cpt):
    if cpt.get('root','isa') == 'arm':
        for sec in cpt.sections():
            import re
            # Search for all ISA sections
            if re.search('.*sys.*\.cpu.*\.isa$', sec):
                miscRegs = cpt.get(sec, 'miscRegs').split()
                # CONTEXTIDR_EL2 defaults to 0b11111100000000000001
                miscRegs[599:599] = [0xFC001]
                cpt.set(sec, 'miscRegs', ' '.join(str(x) for x in miscRegs))

legacy_version = 14
