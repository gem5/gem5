# Use condition code registers for the ARM architecture.
# Previously the integer register file was used for these registers.
def upgrader(cpt):
    if cpt.get('root', 'isa', fallback='') == 'arm':
        for sec in cpt.sections():
            import re

            re_cpu_match = re.match('^(.*sys.*\.cpu[^.]*)\.xc\.(.+)$', sec)
            # Search for all the execution contexts
            if not re_cpu_match:
                continue

            items = []
            for (item,value) in cpt.items(sec):
                items.append(item)
            if 'ccRegs' not in items:
                intRegs = cpt.get(sec, 'intRegs').split()

                # Move those 5 integer registers to the ccRegs register file
                ccRegs = intRegs[38:43]
                del      intRegs[38:43]

                ccRegs.append('0') # CCREG_ZERO

                cpt.set(sec, 'intRegs', ' '.join(intRegs))
                cpt.set(sec, 'ccRegs',  ' '.join(ccRegs))

legacy_version = 13
