def get_sim_ticks(proc, gem5_data=None):
    """
    Get the current simulation tick and simulated time from gem5.

    Args:
        proc: psutil Process object
        gem5_data: Dictionary containing gem5 data from DataManager

    Returns:
        Formatted string showing simulated nanoseconds, or "N/A" if unavailable
    """
    if gem5_data is None:
        gem5_data = {}

    data = gem5_data.get("sim_ticks", None)

    if data is None:
        return "N/A"

    ticks = data.get("ticks", None)
    if ticks is None:
        return "N/A"

    return f"{ticks} ns"
