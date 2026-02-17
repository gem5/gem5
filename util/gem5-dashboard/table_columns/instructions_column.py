def get_current_instructions(proc, gem5_data=None):
    """
    Get current instructions executed from gem5.

    Args:
        proc: psutil Process object
        gem5_data: Dictionary containing gem5 data from DataManager

    Returns:
        Formatted string of instructions executed, or "N/A" if unavailable
    """
    if gem5_data is None:
        gem5_data = {}

    instructions = gem5_data.get("curr_instructions_executed", None)

    if instructions is None:
        return "N/A"

    return f"{instructions:,}"
