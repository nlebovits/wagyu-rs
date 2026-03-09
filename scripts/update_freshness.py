#!/usr/bin/env python3
"""
Update freshness markers in documentation.

When key source files change, update the freshness timestamp in CLAUDE.md
to indicate documentation may need review.

Freshness marker format:
<!-- freshness: 2024-01-15 -->
"""

import re
import sys
from datetime import datetime
from pathlib import Path


# Files that trigger freshness updates when changed
TRIGGER_FILES = [
    "crates/core/src/wagyu.rs",
    "crates/core/src/config.rs",
    "crates/core/src/point.rs",
    "crates/core/src/error.rs",
    "crates/core/src/vatti.rs",
    "crates/core/src/topology_correction.rs",
]


def update_freshness_marker(content: str, today: str) -> tuple[str, bool]:
    """Update freshness marker in content.

    Returns (new_content, was_modified).
    """
    pattern = r"<!--\s*freshness:\s*\d{4}-\d{2}-\d{2}\s*-->"
    replacement = f"<!-- freshness: {today} -->"

    new_content, count = re.subn(pattern, replacement, content)
    return new_content, count > 0


def main():
    """Update freshness markers based on changed files."""
    # Get files from command line (passed by pre-commit)
    changed_files = sys.argv[1:] if len(sys.argv) > 1 else []

    if not changed_files:
        print("No files specified, nothing to update")
        return

    # Check if any trigger files changed
    trigger_hit = False
    for changed in changed_files:
        changed_path = Path(changed)
        for trigger in TRIGGER_FILES:
            if str(changed_path).endswith(trigger) or trigger in str(changed_path):
                trigger_hit = True
                print(f"Trigger file changed: {changed}")
                break
        if trigger_hit:
            break

    if not trigger_hit:
        print("No trigger files changed, skipping freshness update")
        return

    # Update CLAUDE.md freshness marker
    root = Path.cwd()
    claude_md = root / "CLAUDE.md"

    if not claude_md.exists():
        print("CLAUDE.md not found")
        return

    today = datetime.now().strftime("%Y-%m-%d")
    content = claude_md.read_text()
    new_content, modified = update_freshness_marker(content, today)

    if modified:
        claude_md.write_text(new_content)
        print(f"Updated freshness marker to {today}")
    else:
        print("No freshness marker found in CLAUDE.md")


if __name__ == "__main__":
    main()
