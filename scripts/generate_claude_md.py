#!/usr/bin/env python3
"""
Auto-generate sections of CLAUDE.md from source code.

Extracts:
- TODO/FIXME comments
- Debug flags (WAGYU_DEBUG environment variable usage)
- Public API types from lib.rs exports

Updates CLAUDE.md between marker comments:
<!-- BEGIN AUTO-GENERATED: section-name -->
...
<!-- END AUTO-GENERATED: section-name -->
"""

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TodoItem:
    """A TODO or FIXME found in source code."""
    file: str
    line: int
    kind: str  # "TODO" or "FIXME"
    text: str


@dataclass
class DebugFlag:
    """A debug output flag found in source code."""
    prefix: str  # e.g., "[RING_NEW]"
    description: str
    file: str


def find_rust_files(root: Path) -> list[Path]:
    """Find all Rust source files in crates/."""
    return list(root.glob("crates/**/*.rs"))


def extract_todos(files: list[Path]) -> list[TodoItem]:
    """Extract TODO and FIXME comments from Rust files."""
    pattern = re.compile(r"//\s*(TODO|FIXME):?\s*(.+)", re.IGNORECASE)
    todos = []

    for file in files:
        try:
            content = file.read_text()
            for i, line in enumerate(content.splitlines(), 1):
                match = pattern.search(line)
                if match:
                    todos.append(TodoItem(
                        file=str(file.relative_to(Path.cwd())),
                        line=i,
                        kind=match.group(1).upper(),
                        text=match.group(2).strip(),
                    ))
        except Exception as e:
            print(f"Warning: Could not read {file}: {e}", file=sys.stderr)

    return sorted(todos, key=lambda t: (t.kind, t.file, t.line))


def extract_debug_flags(files: list[Path]) -> list[DebugFlag]:
    """Extract debug output prefixes from Rust files."""
    # Look for patterns like: eprintln!("[RING_NEW] ...")
    pattern = re.compile(r'eprintln!\s*\(\s*"(\[[A-Z_]+\])[^"]*"')
    flags = []
    seen = set()

    for file in files:
        try:
            content = file.read_text()
            for match in pattern.finditer(content):
                prefix = match.group(1)
                if prefix not in seen:
                    seen.add(prefix)
                    # Try to extract description from nearby comment
                    start = max(0, match.start() - 200)
                    context = content[start:match.start()]
                    desc_match = re.search(r"//\s*(.+)$", context, re.MULTILINE)
                    description = desc_match.group(1).strip() if desc_match else ""
                    flags.append(DebugFlag(
                        prefix=prefix,
                        description=description,
                        file=str(file.relative_to(Path.cwd())),
                    ))
        except Exception as e:
            print(f"Warning: Could not read {file}: {e}", file=sys.stderr)

    return sorted(flags, key=lambda f: f.prefix)


def extract_public_api(root: Path) -> list[str]:
    """Extract public API items from lib.rs exports."""
    lib_rs = root / "crates" / "core" / "src" / "lib.rs"
    if not lib_rs.exists():
        return []

    content = lib_rs.read_text()
    # Look for pub use statements
    pattern = re.compile(r"pub use\s+(?:crate::)?(\w+)(?:::(\w+))?")
    items = []

    for match in pattern.finditer(content):
        module = match.group(1)
        item = match.group(2)
        if item:
            items.append(f"{module}::{item}")
        else:
            items.append(module)

    return sorted(set(items))


def generate_todos_section(todos: list[TodoItem]) -> str:
    """Generate markdown for TODO/FIXME section."""
    if not todos:
        return "_No open TODOs or FIXMEs._\n"

    lines = []
    current_kind = None

    for todo in todos:
        if todo.kind != current_kind:
            if current_kind is not None:
                lines.append("")
            lines.append(f"**{todo.kind}s:**")
            current_kind = todo.kind

        lines.append(f"- `{todo.file}:{todo.line}` - {todo.text}")

    return "\n".join(lines) + "\n"


def generate_debug_section(flags: list[DebugFlag]) -> str:
    """Generate markdown for debug flags section."""
    if not flags:
        return "_No debug flags found._\n"

    lines = ["Enable with `WAGYU_DEBUG=1`:", ""]
    for flag in flags:
        desc = f" - {flag.description}" if flag.description else ""
        lines.append(f"- `{flag.prefix}`{desc}")

    return "\n".join(lines) + "\n"


def update_claude_md(root: Path, sections: dict[str, str], update: bool) -> bool:
    """Update CLAUDE.md with generated sections.

    Returns True if file was modified (or would be modified in check mode).
    """
    claude_md = root / "CLAUDE.md"
    if not claude_md.exists():
        print(f"Error: {claude_md} not found", file=sys.stderr)
        return False

    content = claude_md.read_text()
    original = content
    modified = False

    for section_name, section_content in sections.items():
        begin_marker = f"<!-- BEGIN AUTO-GENERATED: {section_name} -->"
        end_marker = f"<!-- END AUTO-GENERATED: {section_name} -->"

        if begin_marker not in content:
            print(f"Warning: Marker '{begin_marker}' not found in CLAUDE.md", file=sys.stderr)
            continue

        pattern = re.compile(
            re.escape(begin_marker) + r".*?" + re.escape(end_marker),
            re.DOTALL
        )
        new_section = f"{begin_marker}\n{section_content}{end_marker}"
        content, count = pattern.subn(new_section, content)

        if count > 0:
            modified = True

    if modified:
        if update:
            claude_md.write_text(content)
            print("Updated CLAUDE.md")
        else:
            if content != original:
                print("CLAUDE.md needs updating. Run with --update to apply changes.")
                return True

    return modified


def main():
    parser = argparse.ArgumentParser(description="Generate CLAUDE.md sections from source")
    parser.add_argument("--update", action="store_true", help="Update CLAUDE.md in place")
    parser.add_argument("--check", action="store_true", help="Check if updates are needed (exit 1 if so)")
    args = parser.parse_args()

    root = Path.cwd()
    rust_files = find_rust_files(root)

    if not rust_files:
        print("No Rust files found in crates/", file=sys.stderr)
        sys.exit(1)

    print(f"Scanning {len(rust_files)} Rust files...")

    # Extract information
    todos = extract_todos(rust_files)
    debug_flags = extract_debug_flags(rust_files)
    public_api = extract_public_api(root)

    print(f"Found: {len(todos)} TODOs/FIXMEs, {len(debug_flags)} debug flags, {len(public_api)} API exports")

    # Generate sections
    sections = {
        "todos": generate_todos_section(todos),
        "debug-flags": generate_debug_section(debug_flags),
    }

    # Update or check
    if args.update or args.check:
        needs_update = update_claude_md(root, sections, update=args.update)
        if args.check and needs_update:
            sys.exit(1)
    else:
        # Just print what would be generated
        for name, content in sections.items():
            print(f"\n=== {name} ===")
            print(content)


if __name__ == "__main__":
    main()
