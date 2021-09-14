# This upgrader moves memory pools from the system object to the SE workload
# object.
def upgrader(cpt):
    systems = {}

    # Find sections with 'num_mem_pools' options, and assume those are system
    # objects which host MemPools.
    for sec in cpt.sections():
        num_mem_pools = cpt.get(sec, 'num_mem_pools', fallback=None)
        if num_mem_pools is not None:
            systems[sec] = num_mem_pools

    for sec, num_mem_pools in systems.items():
        # Transfer num_mem_pools to the new location.
        cpt.remove_option(sec, 'num_mem_pools')
        cpt.set(f'{sec}.workload', 'num_mem_pools', num_mem_pools)

        for idx in range(int(num_mem_pools)):
            old_name = f'{sec}.memPool{idx}'
            new_name = f'{sec}.workload.memPool{idx}'

            # Create the new section.
            cpt.add_section(new_name)

            # Copy items from the old section into it.
            for item in cpt.items(old_name):
                cpt.set(new_name, *item)

            # Delete the old section.
            cpt.remove_section(old_name)

depends = 'mempool-sections'
