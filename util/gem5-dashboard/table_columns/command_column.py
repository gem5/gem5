def get_command(proc, gem5_data=None):
    """Get process command line. gem5_data parameter ignored (psutil only)."""
    return " ".join(proc.info["cmdline"] or [])
