def get_command(proc):
    return " ".join(proc.info["cmdline"] or [])
