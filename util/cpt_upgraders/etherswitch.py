def upgrader(cpt):
    for sec in cpt.sections():
        if sec == "system":
            options = cpt.items(sec)
            for it in options:
                opt_split = it[0].split(".")
                if len(opt_split) < 2:
                    continue
                new_sec_name = opt_split[1]
                old_opt_name = opt_split[len(opt_split) - 1]
                if "outputFifo" in new_sec_name:
                    new_sec_name = new_sec_name.rstrip("outputFifo")
                    new_sec_name += ".outputFifo"
                    new_sec_name = "system.system.%s" % (new_sec_name)
                    if not cpt.has_section(new_sec_name):
                        cpt.add_section(new_sec_name)
                    if old_opt_name == "size":
                        cpt.set(new_sec_name, "_size", it[1])
                    elif old_opt_name == "packets":
                        cpt.set(new_sec_name, "fifosize", it[1])
                    else:
                        cpt.set(new_sec_name, old_opt_name, it[1])
                    cpt.remove_option(sec, it[0])
