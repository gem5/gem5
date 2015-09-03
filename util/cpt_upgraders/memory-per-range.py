# The backing store supporting the memories in the system has changed
# in that it is now stored globally per address range. As a result the
# actual storage is separate from the memory controllers themselves.
def upgrader(cpt):
    for sec in cpt.sections():
        import re
        # Search for a physical memory
        if re.search('.*sys.*\.physmem$', sec):
            # Add the number of stores attribute to the global physmem
            cpt.set(sec, 'nbr_of_stores', '1')

            # Get the filename and size as this is moving to the
            # specific backing store
            mem_filename = cpt.get(sec, 'filename')
            mem_size = cpt.get(sec, '_size')
            cpt.remove_option(sec, 'filename')
            cpt.remove_option(sec, '_size')

            # Get the name so that we can create the new section
            system_name = str(sec).split('.')[0]
            section_name = system_name + '.physmem.store0'
            cpt.add_section(section_name)
            cpt.set(section_name, 'store_id', '0')
            cpt.set(section_name, 'range_size', mem_size)
            cpt.set(section_name, 'filename', mem_filename)
        elif re.search('.*sys.*\.\w*mem$', sec):
            # Due to the lack of information about a start address,
            # this migration only works if there is a single memory in
            # the system, thus starting at 0
            raise ValueError("more than one memory detected (" + sec + ")")

legacy_version = 2
