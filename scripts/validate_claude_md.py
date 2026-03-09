#!/usr/bin/env python3
"""
Validate CLAUDE.md references to prevent documentation drift.

Checks:
- File paths referenced in CLAUDE.md exist
- Function/type names exist in codebase
- Freshness markers are not stale (> 30 days)
"""

import re
import sys
from datetime import datetime, timedelta
from pathlib import Path


def find_file_references(content: str) -> list[str]:
    """Extract file path references from markdown content."""
    # Match patterns like: `path/to/file.rs`, `context/ARCHITECTURE.md`
    # Also match paths in code blocks
    patterns = [
        r"`([a-zA-Z0-9_/.-]+\.(rs|md|toml|json|sh))`",  # backtick paths
        r"\[.*?\]\(([a-zA-Z0-9_/.-]+\.(rs|md|toml|json|sh))\)",  # markdown links
    ]

    paths = []
    for pattern in patterns:
        for match in re.finditer(pattern, content):
            paths.append(match.group(1))

    return list(set(paths))


def find_rust_references(content: str) -> list[str]:
    """Extract Rust function/type references from markdown content."""
    # Match patterns like: `function_name`, `TypeName`, `module::function`
    # Filter out obvious non-Rust things
    pattern = r"`([a-z_][a-z0-9_]*(?:::[a-z_][a-z0-9_]*)*)`"

    refs = []
    for match in re.finditer(pattern, content, re.IGNORECASE):
        ref = match.group(1)
        # Skip common non-Rust patterns
        if ref in ("true", "false", "None", "Some"):
            continue
        if "/" in ref or "." in ref:  # file paths
            continue
        if ref.startswith("WAGYU_"):  # env vars
            continue
        if len(ref) < 3:  # too short
            continue
        refs.append(ref)

    return list(set(refs))


def find_freshness_markers(content: str) -> list[tuple[str, datetime]]:
    """Extract freshness markers and their dates."""
    pattern = r"<!--\s*freshness:\s*(\d{4}-\d{2}-\d{2})\s*-->"
    markers = []

    for match in re.finditer(pattern, content):
        date_str = match.group(1)
        try:
            date = datetime.strptime(date_str, "%Y-%m-%d")
            markers.append((date_str, date))
        except ValueError:
            pass

    return markers


def check_file_exists(path: str, root: Path) -> bool:
    """Check if a file path exists relative to project root."""
    # Handle some common patterns
    if path.startswith("../"):
        return True  # External references, skip
    if path.startswith("crates/") or path.startswith("context/") or path.startswith("tools/"):
        return (root / path).exists()
    if path.endswith(".md"):
        return (root / path).exists()
    return True  # Assume exists for other patterns


def check_rust_reference(ref: str, rust_files: list[Path]) -> bool:
    """Check if a Rust function/type reference exists in codebase."""
    # Build search patterns
    patterns = [
        f"fn {ref}",
        f"pub fn {ref}",
        f"struct {ref}",
        f"pub struct {ref}",
        f"enum {ref}",
        f"pub enum {ref}",
        f"type {ref}",
        f"pub type {ref}",
        f"mod {ref}",
        f"pub mod {ref}",
        f"const {ref}",
        f"pub const {ref}",
    ]

    for file in rust_files:
        try:
            content = file.read_text()
            for pattern in patterns:
                if pattern in content:
                    return True
        except Exception:
            pass

    return False


def validate_claude_md(root: Path) -> list[str]:
    """Validate CLAUDE.md and return list of errors."""
    errors = []
    claude_md = root / "CLAUDE.md"

    if not claude_md.exists():
        return ["CLAUDE.md not found"]

    content = claude_md.read_text()

    # Check file references
    file_refs = find_file_references(content)
    for ref in file_refs:
        if not check_file_exists(ref, root):
            errors.append(f"File reference not found: {ref}")

    # Check Rust references (optional - can be noisy)
    rust_files = list(root.glob("crates/**/*.rs"))
    rust_refs = find_rust_references(content)

    # Only check explicitly documented functions (those in code blocks explaining code)
    # Skip general references to be less noisy
    documented_refs = [
        "mark_as_merged",
        "clear_merged_rings",
        "correct_self_intersections",
        "correct_collinear_edges",
        "correct_topology",
        "merge_rings_at_intersection",
    ]

    for ref in documented_refs:
        if ref in rust_refs and not check_rust_reference(ref, rust_files):
            errors.append(f"Rust reference not found in codebase: {ref}")

    # Check freshness markers
    freshness_markers = find_freshness_markers(content)
    stale_threshold = datetime.now() - timedelta(days=30)

    for date_str, date in freshness_markers:
        if date < stale_threshold:
            errors.append(f"Stale freshness marker: {date_str} (older than 30 days)")

    return errors


def main():
    root = Path.cwd()
    errors = validate_claude_md(root)

    if errors:
        print("CLAUDE.md validation errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)
    else:
        print("CLAUDE.md validation passed")
        sys.exit(0)


if __name__ == "__main__":
    main()
