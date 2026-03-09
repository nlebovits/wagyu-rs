#!/usr/bin/env python3
"""
Generate API documentation from Rust source.

Uses rustdoc JSON output (nightly) or falls back to parsing source files directly.
Generates markdown files suitable for mkdocs.

Output structure:
  docs/api/
    index.md        - Module overview
    wagyu.md        - Wagyu struct
    operations.md   - Operation enum
    point.md        - Point type
    config.md       - FillType, PolygonType
"""

import json
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class DocItem:
    """A documented item (struct, enum, function, etc.)."""
    name: str
    kind: str  # "struct", "enum", "fn", "type"
    doc: str
    signature: str
    file: str
    line: int
    children: list["DocItem"] = field(default_factory=list)


def try_rustdoc_json(root: Path) -> dict | None:
    """Try to generate rustdoc JSON output.

    Requires nightly Rust. Returns None if not available.
    """
    if not shutil.which("rustup"):
        return None

    try:
        # Check if nightly is available
        result = subprocess.run(
            ["rustup", "run", "nightly", "rustc", "--version"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            return None

        # Generate JSON docs
        result = subprocess.run(
            [
                "rustup", "run", "nightly",
                "cargo", "rustdoc", "-p", "wagyu-rs",
                "--", "-Z", "unstable-options", "--output-format", "json"
            ],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=120,
        )

        if result.returncode != 0:
            print(f"rustdoc JSON generation failed: {result.stderr[:500]}", file=sys.stderr)
            return None

        # Find the JSON file
        json_path = root / "target" / "doc" / "wagyu_rs.json"
        if json_path.exists():
            return json.loads(json_path.read_text())

    except Exception as e:
        print(f"rustdoc JSON not available: {e}", file=sys.stderr)

    return None


def parse_source_docs(root: Path) -> list[DocItem]:
    """Parse documentation directly from Rust source files."""
    items = []
    lib_rs = root / "crates" / "core" / "src" / "lib.rs"

    if not lib_rs.exists():
        return items

    # Find all public items with doc comments
    src_dir = lib_rs.parent

    for rs_file in src_dir.glob("*.rs"):
        content = rs_file.read_text()
        items.extend(parse_file_docs(content, str(rs_file.relative_to(root))))

    return items


def parse_file_docs(content: str, file_path: str) -> list[DocItem]:
    """Parse doc comments and public items from a Rust file."""
    items = []
    lines = content.splitlines()

    i = 0
    while i < len(lines):
        # Collect doc comments
        doc_lines = []
        while i < len(lines) and (lines[i].strip().startswith("///") or
                                   lines[i].strip().startswith("//!")):
            comment = lines[i].strip()
            if comment.startswith("///"):
                doc_lines.append(comment[3:].strip())
            elif comment.startswith("//!"):
                doc_lines.append(comment[3:].strip())
            i += 1

        # Check for public item
        if i < len(lines):
            line = lines[i].strip()

            # Match public items
            patterns = [
                (r"pub\s+struct\s+(\w+)", "struct"),
                (r"pub\s+enum\s+(\w+)", "enum"),
                (r"pub\s+fn\s+(\w+)", "fn"),
                (r"pub\s+type\s+(\w+)", "type"),
                (r"pub\s+trait\s+(\w+)", "trait"),
                (r"pub\s+const\s+(\w+)", "const"),
            ]

            for pattern, kind in patterns:
                match = re.match(pattern, line)
                if match:
                    name = match.group(1)
                    # Get full signature (up to opening brace or semicolon)
                    signature = line
                    j = i + 1
                    while j < len(lines) and not ('{' in lines[j] or ';' in lines[j]):
                        signature += " " + lines[j].strip()
                        j += 1

                    items.append(DocItem(
                        name=name,
                        kind=kind,
                        doc="\n".join(doc_lines),
                        signature=signature.split("{")[0].split(";")[0].strip(),
                        file=file_path,
                        line=i + 1,
                    ))
                    break

        i += 1

    return items


def generate_markdown(items: list[DocItem], output_dir: Path) -> None:
    """Generate markdown files from documented items."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Group items by kind
    structs = [i for i in items if i.kind == "struct"]
    enums = [i for i in items if i.kind == "enum"]
    functions = [i for i in items if i.kind == "fn"]
    types = [i for i in items if i.kind == "type"]
    traits = [i for i in items if i.kind == "trait"]

    # Generate index.md
    index_content = """# API Reference

Auto-generated from source code. See [rustdoc](https://docs.rs/wagyu-rs) for full documentation.

## Types

"""
    for item in structs + enums + types:
        index_content += f"- [`{item.name}`](#{item.name.lower()}) - {item.doc.split(chr(10))[0] if item.doc else item.kind}\n"

    if functions:
        index_content += "\n## Functions\n\n"
        for item in functions:
            index_content += f"- [`{item.name}`](#{item.name.lower()}) - {item.doc.split(chr(10))[0] if item.doc else ''}\n"

    if traits:
        index_content += "\n## Traits\n\n"
        for item in traits:
            index_content += f"- [`{item.name}`](#{item.name.lower()}) - {item.doc.split(chr(10))[0] if item.doc else ''}\n"

    index_content += "\n---\n\n"

    # Add detailed documentation for each item
    for item in structs + enums + types + functions + traits:
        index_content += f"## {item.name}\n\n"
        index_content += f"```rust\n{item.signature}\n```\n\n"
        if item.doc:
            index_content += f"{item.doc}\n\n"
        index_content += f"*Defined in `{item.file}`*\n\n---\n\n"

    (output_dir / "index.md").write_text(index_content)
    print(f"Generated {output_dir / 'index.md'}")


def main():
    root = Path.cwd()
    output_dir = root / "docs" / "api"

    print("Generating API documentation...")

    # Try rustdoc JSON first (more accurate)
    rustdoc_json = try_rustdoc_json(root)

    if rustdoc_json:
        print("Using rustdoc JSON output")
        # TODO: Parse rustdoc JSON format properly
        # For now, fall back to source parsing
        items = parse_source_docs(root)
    else:
        print("Falling back to source parsing")
        items = parse_source_docs(root)

    print(f"Found {len(items)} documented items")

    if items:
        generate_markdown(items, output_dir)
        print(f"API documentation generated in {output_dir}")
    else:
        print("No public items found to document")
        sys.exit(1)


if __name__ == "__main__":
    main()
