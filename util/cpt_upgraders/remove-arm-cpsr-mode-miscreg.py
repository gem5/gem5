# Remove the MISCREG_CPSR_MODE register from the ARM register file
def upgrader(cpt):
    if cpt.get('root', 'isa', fallback='') == 'arm':
        for sec in cpt.sections():
            import re
            # Search for all ISA sections
            if re.search('.*sys.*\.cpu.*\.isa$', sec):
                mr = cpt.get(sec, 'miscRegs').split()
                # Remove MISCREG_CPSR_MODE
                del mr[137]
                cpt.set(sec, 'miscRegs', ' '.join(str(x) for x in mr))

legacy_version = 5
