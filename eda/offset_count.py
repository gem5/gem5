from collections import (
    Counter,
    defaultdict,
)


def parse_address(line):
    return int(line.split()[1], 16)

def calculate_offsets(addresses):
    offsets = []
    for i in range(1, len(addresses)):
        for j in range(1, 2):
            offset = addresses[i] - addresses[i - j]
            offsets.append(offset)
    return offsets

def main():
    file_path = './eda/addr_pc.log'
    addresses = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith("Addr:"):
                address = parse_address(line)
                addresses.append(address)

    offsets = calculate_offsets(addresses)
    offset_counts = Counter(offsets)
    most_common_offsets = offset_counts.most_common(20)

    print("Top 20 Most Frequent Offsets:")
    for offset, count in most_common_offsets:
        print(f'Offset: {offset}, Count: {count}')

if __name__ == "__main__":
    main()