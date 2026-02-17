def get_user(proc, gem5_data=None):
    """Get process username. gem5_data parameter ignored (psutil only)."""
    return proc.info["username"]
