def get_pid(proc, gem5_data=None):
    """Get process ID. gem5_data parameter ignored (psutil only)."""
    return str(proc.info["pid"])
