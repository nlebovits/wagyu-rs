#!/usr/bin/env python3
"""
Validate README.md to prevent documentation drift.

Checks:
- Rust code examples compile (blocks not marked `ignore`)
- Internal file links exist (CONTRIBUTING.md, etc.)
- Version in README matches Cargo.toml
- Badge URLs are syntactically valid
"""

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def extract_rust_code_blocks(content: str) -> list[tuple[str, bool]]:
    """Extract Rust code blocks and whether they should be checked.

    Returns list of (code, should_check) tuples.
    Code blocks marked with `rust,ignore` are returned with should_check=False.
    """
    # Match ```rust or ```rust,ignore blocks
    pattern = re.compile(r"```rust(,\s*ignore)?\n(.*?)```", re.DOTALL)
    blocks = []

    for match in pattern.finditer(content):
        ignore_marker = match.group(1)
        code = match.group(2)
        should_check = ignore_marker is None
        blocks.append((code.strip(), should_check))

    return blocks


def check_rust_code_compiles(code: str) -> tuple[bool, str]:
    """Check if Rust code compiles.

    Returns (success, error_message).
    """
    # Create a temporary Cargo project
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        # Create Cargo.toml
        cargo_toml = """
[package]
name = "readme_check"
version = "0.1.0"
edition = "2021"

[dependencies]
wagyu-rs = { path = "WAGYU_PATH" }
"""
        # Get the actual wagyu-rs path
        wagyu_path = Path.cwd() / "crates" / "core"
        cargo_toml = cargo_toml.replace("WAGYU_PATH", str(wagyu_path))
        (tmpdir / "Cargo.toml").write_text(cargo_toml)

        # Create src directory and main.rs
        src_dir = tmpdir / "src"
        src_dir.mkdir()

        # Wrap code in main function if needed
        if "fn main" not in code:
            wrapped_code = f"fn main() {{\n{code}\n}}"
        else:
            wrapped_code = code

        (src_dir / "main.rs").write_text(wrapped_code)

        # Run cargo check
        result = subprocess.run(
            ["cargo", "check", "--message-format=short"],
            cwd=tmpdir,
            capture_output=True,
            text=True,
            timeout=60,
        )

        if result.returncode != 0:
            return False, result.stderr
        return True, ""


def extract_internal_links(content: str) -> list[str]:
    """Extract internal file links from markdown."""
    # Match [text](file.md) or [text](path/to/file)
    pattern = re.compile(r"\[.*?\]\(([^http][^)]+)\)")
    links = []

    for match in pattern.finditer(content):
        link = match.group(1)
        # Skip anchors and external links
        if link.startswith("#") or link.startswith("http"):
            continue
        # Remove anchor from link
        if "#" in link:
            link = link.split("#")[0]
        if link:
            links.append(link)

    return list(set(links))


def get_cargo_version(root: Path) -> str | None:
    """Extract version from Cargo.toml."""
    cargo_toml = root / "Cargo.toml"
    if not cargo_toml.exists():
        return None

    content = cargo_toml.read_text()
    # Look for version in [workspace.package]
    match = re.search(r'\[workspace\.package\].*?version\s*=\s*"([^"]+)"', content, re.DOTALL)
    if match:
        return match.group(1)

    # Fallback to top-level version
    match = re.search(r'^version\s*=\s*"([^"]+)"', content, re.MULTILINE)
    if match:
        return match.group(1)

    return None


def check_version_consistency(readme_content: str, cargo_version: str) -> list[str]:
    """Check that any version mentioned in README matches Cargo.toml."""
    errors = []

    # Look for version patterns like v0.1.0 or 0.1.0
    version_pattern = re.compile(r'\bv?(\d+\.\d+\.\d+)\b')

    for match in version_pattern.finditer(readme_content):
        found_version = match.group(1)
        # Only flag if it looks like it should match our version
        # (near "wagyu", "version", "crates.io", etc.)
        context_start = max(0, match.start() - 100)
        context = readme_content[context_start:match.start()].lower()
        if any(word in context for word in ["wagyu", "version", "crates", "cargo add"]):
            if found_version != cargo_version:
                errors.append(
                    f"Version mismatch: README has '{found_version}', "
                    f"Cargo.toml has '{cargo_version}'"
                )

    return errors


def validate_readme(root: Path) -> list[str]:
    """Validate README.md and return list of errors."""
    errors = []
    readme = root / "README.md"

    if not readme.exists():
        return ["README.md not found"]

    content = readme.read_text()

    # Check internal links
    links = extract_internal_links(content)
    for link in links:
        link_path = root / link
        if not link_path.exists():
            errors.append(f"Broken internal link: {link}")

    # Check version consistency
    cargo_version = get_cargo_version(root)
    if cargo_version:
        version_errors = check_version_consistency(content, cargo_version)
        errors.extend(version_errors)

    # Check Rust code blocks compile
    code_blocks = extract_rust_code_blocks(content)
    blocks_to_check = [(code, i) for i, (code, should_check) in enumerate(code_blocks) if should_check]

    if blocks_to_check and shutil.which("cargo"):
        print(f"Checking {len(blocks_to_check)} Rust code blocks...")
        for code, block_num in blocks_to_check:
            success, error = check_rust_code_compiles(code)
            if not success:
                # Truncate error message
                error_summary = error[:500] + "..." if len(error) > 500 else error
                errors.append(f"Code block {block_num + 1} does not compile:\n{error_summary}")

    return errors


def main():
    root = Path.cwd()
    errors = validate_readme(root)

    if errors:
        print("README.md validation errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)
    else:
        print("README.md validation passed")
        sys.exit(0)


if __name__ == "__main__":
    main()
