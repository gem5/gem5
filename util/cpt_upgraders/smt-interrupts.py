# Upgrade single-threaded checkpoints to be properly supported with SMT.
# SMT adds per-thread interrupts.  Thus we must move the interrupt status
# from the CPU and into the execution context.
def upgrader(cpt):
    for sec in cpt.sections():
        import re

        re_cpu_match = re.match('^(.*sys.*\.cpu[^._]*)$', sec)
        if re_cpu_match != None:
            interrupts = cpt.get(sec, 'interrupts')
            intStatus = cpt.get(sec, 'intStatus')

            cpu_name = re_cpu_match.group(1)

            cpt.set(cpu_name + ".xc.0", 'interrupts', interrupts)
            cpt.set(cpu_name + ".xc.0", 'intStatus', intStatus)

            cpt.remove_option(sec, 'interrupts')
            cpt.remove_option(sec, 'intStatus')
