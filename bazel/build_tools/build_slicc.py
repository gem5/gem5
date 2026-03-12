import argparse
import os
import sys

# Add src/mem to sys.path so we can import 'slicc'
base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.append(os.path.join(base_path, 'src/mem'))
sys.path.append(os.path.join(base_path, 'src'))
# Add build_tools for code_formatter and grammar
sys.path.append(os.path.join(base_path, 'build_tools'))
# Add ext/ply for ply.lex and ply.yacc
sys.path.append(os.path.join(base_path, 'ext/ply'))

from slicc.parser import SLICC

def main():
    parser = argparse.ArgumentParser(description='Wrapper for gem5 SLICC compiler')
    parser.add_argument('--protocol', help='The protocol .slicc file to parse')
    parser.add_argument('--includes', help='Comma separated list of shared .slicc files')
    parser.add_argument('--output_dir', '-C', help='Output directory for generated C++ files')
    parser.add_argument('--cc_includes', help='Comma separated list of C++ headers to include in generated files')
    parser.add_argument('positional_includes', nargs='*', help='Positional .slicc include files')
    
    args = parser.parse_args()
    
    protocol_base = os.path.join(os.getcwd(), 'src/mem/ruby/protocol')
    
    includes = []
    if args.includes:
        includes.extend(args.includes.split(','))
    if args.positional_includes:
        includes.extend(args.positional_includes)
    
    cc_includes = []
    if args.cc_includes:
        cc_includes.extend(args.cc_includes.split(','))
    
    if not args.protocol and not includes:
        print("Error: Either --protocol or at least one include file must be provided.", file=sys.stderr)
        sys.exit(1)
        
    slicc = SLICC(
        args.protocol,
        includes,
        protocol_base,
        verbose=True
    )
    
    slicc.process()
    slicc.writeCodeFiles(args.output_dir, cc_includes)

if __name__ == '__main__':
    main()
