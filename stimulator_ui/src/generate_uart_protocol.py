import re
from pathlib import Path
import argparse
import sys

def main():
    # ----------------------------
    # Argument parsing
    # ----------------------------
    parser = argparse.ArgumentParser(
        description="Generate Python UART protocol constants from a C header file"
    )

    parser.add_argument(
        "header_dir",
        type=Path,
        help="Directory containing uart_comms.h (or pass the header file itself)"
    )

    parser.add_argument(
        "--header-name",
        default="uart_comms.h",
        help="Header filename (default: uart_comms.h)"
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("uart_protocol.py"),
        help="Output Python file (default: uart_protocol.py)"
    )

    args = parser.parse_args()

    # Resolve header path: accept a directory or a direct file path
    header_path = args.header_dir
    if header_path.is_dir():
        header_path = header_path / args.header_name
        if not header_path.exists():
            # Try to find recursively within the directory
            candidates = list(args.header_dir.rglob(args.header_name))
            if candidates:
                header_path = candidates[0]
    if not header_path.exists() or not header_path.is_file():
        print(f"Error: Header file not found: {header_path}", file=sys.stderr)
        sys.exit(1)

    # ----------------------------
    # Regex patterns
    # ----------------------------
    define_re = re.compile(r"#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)")
    enum_re = re.compile(r"enum\s+(\w+).*?\{(.*?)\};", re.S)
    enum_entry_re = re.compile(r"(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)")

    # ----------------------------
    # Parse header
    # ----------------------------
    text = header_path.read_text()

    lines = []
    lines.append("# Auto-generated file — DO NOT EDIT")
    lines.append(f"# Source: {header_path}")
    lines.append("")

    # Extract #defines
    for name, value in define_re.findall(text):
        lines.append(f"{name} = {int(value, 0)}")

    lines.append("")

    # Extract enums
    for enum_name, body in enum_re.findall(text):
        lines.append(f"class {enum_name}:")
        for entry, value in enum_entry_re.findall(body):
            lines.append(f"    {entry} = {int(value, 0)}")
        lines.append("")

    # ----------------------------
    # Write output (support directory or file path)
    # ----------------------------
    output_path = args.output
    if output_path.is_dir():
        output_path = output_path / "uart_protocol.py"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines))

    print(f"Generated {output_path} from {header_path}")


if __name__ == "__main__":
    main()