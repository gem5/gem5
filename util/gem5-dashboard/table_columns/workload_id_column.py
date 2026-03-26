def get_workload_id(proc, gem5_data=None):
    """
    Get workload ID from gem5 data.

    Args:
        proc: psutil Process object
        gem5_data: Dictionary containing gem5 data from DataManager
    Returns:
        str: Workload ID
    """
    if gem5_data is None:
        gem5_data = {}

    workload_id = gem5_data.get("workload", None)

    if workload_id is None:
        return "N/A"

    return str(workload_id)
