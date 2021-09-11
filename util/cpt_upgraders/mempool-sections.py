# This upgrader moves memory pool attributes from a vector of pointers to the
# next free page and a vector of the limit of each pool to sections which each
# represent one pool and which hold the page shift amount, the next free page,
# and the total pages in the pool.
def upgrader(cpt):
    systems = {}
    for sec in cpt.sections():
        ptrs = cpt.get(sec, 'ptrs', fallback=None)
        limits = cpt.get(sec, 'limits', fallback=None)

        if ptrs and limits:
            systems[sec] = ptrs, limits

    for sec, (ptrs, limits) in systems.items():

        ptrs = list(map(int, ptrs.split()))
        limits = list(map(int, limits.split()))

        if len(ptrs) != len(limits):
            print(
                f"'{sec}ptrs' and '{limits}limits' were not the same length!")

        cpt.set(sec, 'num_mem_pools', str(len(ptrs)))

        cpt.remove_option(sec, 'ptrs')
        cpt.remove_option(sec, 'limits')

        # Assume the page shift is 12, for a 4KiB page.
        page_shift = 12

        for idx, (ptr, limit) in enumerate(zip(ptrs, limits)):
            new_sec = f'{sec}.memPool{idx}'
            cpt.add_section(new_sec)
            cpt.set(new_sec, 'page_shift', str(page_shift))
            # Since there's no way to tell where the pool actually started,
            # just assume it started wherever it is right now.
            cpt.set(new_sec, 'start_page', str(ptr >> page_shift))
            cpt.set(new_sec, 'free_page_num', str(ptr >> page_shift))
            cpt.set(new_sec, 'total_pages', str((limit - ptr) >> page_shift))
