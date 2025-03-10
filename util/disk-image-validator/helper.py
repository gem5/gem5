import json
import re
from pathlib import Path


def validate_npb_output(output_file: str, workload: str) -> bool:
    benchmark, class_name = extract_benchmark_and_class(workload)
    if not benchmark or not class_name:
        print("Invalid workload format.")
        return False

    benchmark_pattern = re.compile(
        rf"{benchmark} Benchmark Completed\.?", re.IGNORECASE
    )
    class_pattern = re.compile(rf"Class\s*=\s*{class_name}", re.IGNORECASE)

    try:
        with open(output_file) as file:
            content = file.read()

            if benchmark_pattern.search(content) and class_pattern.search(
                content
            ):
                print(
                    f"Validation successful: {benchmark} (Class {class_name}) completed successfully."
                )
                return True
            else:
                print(
                    "Validation failed: Required strings not found in output."
                )
                return False
    except FileNotFoundError:
        print(f"Error: Output file '{output_file}' not found.")
        return False


def extract_benchmark_and_class(workload: str):
    match = re.search(r"npb-(\w+)-([a-zA-Z])", workload)
    if match:
        benchmark, class_name = match.groups()
        return benchmark.upper(), class_name.upper()
    return None, None


def parse_stats(file_path: str):
    stats_to_track = {"simSeconds": 0.0, "hostSeconds": 0.0, "simInsts": 0}

    try:
        with open(file_path) as file:
            for line in file:
                for stat in stats_to_track:
                    match = re.match(rf"{stat}\s+([-+]?[0-9]*\.?[0-9]+)", line)
                    if match:
                        stats_to_track[stat] += float(match.group(1))
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")

    return stats_to_track


def update_json_with_stats(
    json_file: str,
    workload_id: str,
    version: str,
    stats: dict,
    output_file: str,
):
    try:
        with open(json_file) as file:
            data = json.load(file)

        updated_workload = {}
        updated = False

        for workload in data:
            if (
                workload.get("id") == workload_id
                and workload.get("resource_version") == version
            ):
                workload["simulation_run_results"] = {
                    "total_instructions": stats["simInsts"],
                    "host_seconds": stats["hostSeconds"],
                    "sim_seconds": stats["simSeconds"],
                }
                updated_workload = workload
                updated = True
                break

        if not updated:
            print(
                f"⚠️ WARNING: Workload {workload_id} (Version: {version}) not found in JSON."
            )

        # Append updated workload to output file
        output_path = Path(output_file)
        if output_path.exists():
            with open(output_file, "r+") as file:
                existing_data = json.load(file)
                existing_data.append(updated_workload)
                file.seek(0)
                json.dump(existing_data, file, indent=4)
        else:
            with open(output_file, "w") as file:
                json.dump([updated_workload], file, indent=4)
    except FileNotFoundError:
        print(f"Error: File '{json_file}' not found.")
    except json.JSONDecodeError:
        print(f"Error: Could not parse JSON from '{json_file}'.")
