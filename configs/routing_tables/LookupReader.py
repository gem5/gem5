import csv
import os

# Get the folder where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))


def read_csv(filename):
    # Build the full path to the CSV
    csv_file = os.path.join(script_dir, filename)
    print(csv_file)
    # Read the CSV
    with open(csv_file, newline="") as f:
        reader = csv.DictReader(f)
        array = []
        for row in reader:
            array.extend(
                [
                    int(row["src"]),
                    int(row["dst"]),
                    int(row["curr"]),
                    int(row["next_hop"]),
                ]
            )

    return array
