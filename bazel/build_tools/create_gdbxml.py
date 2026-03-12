import os
import sys

# Add build_tools to sys.path for blob.py
base_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
sys.path.append(os.path.join(base_path, "build_tools"))

from blob import bytesToCppArray
from code_formatter import code_formatter


def main():
    if len(sys.argv) != 6:
        print(
            "Usage: %s <xml_file> <symbol> <isa> <out_cc> <out_hh>"
            % sys.argv[0],
            file=sys.stderr,
        )
        sys.exit(1)

    xml_file, symbol, isa, out_cc, out_hh = sys.argv[1:]

    with open(xml_file, "rb") as f:
        data = f.read()

    # Generate CC file
    code = code_formatter()
    code('#include "${out_hh}"')
    code("namespace gem5 {")
    code("namespace Blobs {")
    bytesToCppArray(code, symbol, data)
    code("const int ${symbol}_len = sizeof(${symbol});")
    code("} // namespace Blobs")
    code("} // namespace gem5")
    code.write(out_cc)

    # Generate HH file
    code = code_formatter()
    code("#include <cstdint>")
    code("namespace gem5 {")
    code("namespace Blobs {")
    code("extern const std::uint8_t ${symbol}[];")
    code("extern const int ${symbol}_len;")
    code("} // namespace Blobs")
    code("} // namespace gem5")
    code.write(out_hh)


if __name__ == "__main__":
    main()
