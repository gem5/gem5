def get_status(proc, gem5_data=None):
    """Get process status. gem5_data parameter ignored (psutil only)."""
    return proc.info["status"]
