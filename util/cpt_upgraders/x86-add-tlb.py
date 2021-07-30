# Add TLB to x86 checkpoints
def upgrader(cpt):
    if cpt.get('root', 'isa', fallback='') == 'x86':
        for sec in cpt.sections():
            import re
            # Search for all ISA sections
            if re.search('.*sys.*\.cpu.*\.dtb$', sec):
                cpt.set(sec, '_size', '0')
                cpt.set(sec, 'lruSeq', '0')

            if re.search('.*sys.*\.cpu.*\.itb$', sec):
                cpt.set(sec, '_size', '0')
                cpt.set(sec, 'lruSeq', '0')
    else:
        print("ISA is not x86")

legacy_version = 6
