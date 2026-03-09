#!/usr/bin/env python3
"""
Validate architecture rules defined in .architecture.toml.

Checks module boundaries and import rules to enforce architectural constraints.
Uses regex-based analysis of Rust source files.

Rule types:
- forbidden: Source modules cannot import forbidden modules
- independence: Modules must not import any other internal modules
- forbidden_pattern: Source modules cannot contain forbidden patterns
"""

import re
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import tomllib  # Python 3.11+
except ImportError:
    import tomli as tomllib  # Fallback for older Python


@dataclass
class Contract:
    """An architecture contract/rule."""
    id: str
    name: str
    contract_type: str
    description: str
    source_modules: list[str] | None = None
    forbidden_modules: list[str] | None = None
    modules: list[str] | None = None
    forbidden_patterns: list[str] | None = None
    exceptions: list[str] | None = None


def load_architecture_rules(root: Path) -> list[Contract]:
    """Load architecture rules from .architecture.toml."""
    config_path = root / ".architecture.toml"
    if not config_path.exists():
        print("No .architecture.toml found, skipping architecture check")
        return []

    content = config_path.read_text()
    config = tomllib.loads(content)

    contracts = []
    for contract_data in config.get("contracts", []):
        contracts.append(Contract(
            id=contract_data["id"],
            name=contract_data["name"],
            contract_type=contract_data["type"],
            description=contract_data.get("description", ""),
            source_modules=contract_data.get("source_modules"),
            forbidden_modules=contract_data.get("forbidden_modules"),
            modules=contract_data.get("modules"),
            forbidden_patterns=contract_data.get("forbidden_patterns"),
            exceptions=contract_data.get("exceptions", []),
        ))

    return contracts


def find_rust_files_for_module(root: Path, module: str) -> list[Path]:
    """Find Rust files belonging to a module."""
    # Module format: crate_name or crate_name::submodule
    parts = module.replace("::", "/").split("/")

    if parts[0] in ("wagyu_rs", "wagyu-rs"):
        base = root / "crates" / "core" / "src"
    elif parts[0] in ("wagyu_cli", "wagyu_rs_cli", "wagyu-rs-cli"):
        base = root / "crates" / "cli" / "src"
    else:
        return []

    if len(parts) == 1:
        # Return all files in the crate
        return list(base.glob("**/*.rs"))
    else:
        # Return specific module file or directory
        submodule = parts[1]
        module_file = base / f"{submodule}.rs"
        module_dir = base / submodule

        files = []
        if module_file.exists():
            files.append(module_file)
        if module_dir.exists():
            files.extend(module_dir.glob("**/*.rs"))
        return files


def extract_imports(content: str) -> list[str]:
    """Extract use statements from Rust source."""
    # Match various use patterns
    patterns = [
        r"use\s+crate::(\w+)",  # use crate::module
        r"use\s+super::(\w+)",  # use super::module
        r"use\s+wagyu_rs::(\w+)",  # use wagyu_rs::module
        r"use\s+wagyu_cli::(\w+)",  # use wagyu_cli::module
        r"crate::(\w+)::",  # crate::module::
    ]

    imports = []
    for pattern in patterns:
        for match in re.finditer(pattern, content):
            imports.append(match.group(1))

    return list(set(imports))


def check_forbidden_contract(contract: Contract, root: Path) -> list[str]:
    """Check a 'forbidden' type contract."""
    errors = []

    for source in contract.source_modules or []:
        files = find_rust_files_for_module(root, source)
        for file in files:
            # Check exceptions
            if contract.exceptions:
                if any(exc in str(file) for exc in contract.exceptions):
                    continue

            content = file.read_text()
            imports = extract_imports(content)

            for forbidden in contract.forbidden_modules or []:
                # Check if any import matches forbidden module
                forbidden_name = forbidden.split("::")[-1]
                for imp in imports:
                    if imp == forbidden_name or forbidden in content:
                        rel_path = file.relative_to(root)
                        errors.append(
                            f"[{contract.id}] {rel_path} imports forbidden module '{forbidden}'"
                        )

    return errors


def check_independence_contract(contract: Contract, root: Path) -> list[str]:
    """Check an 'independence' type contract."""
    errors = []

    # Get list of all internal modules
    internal_modules = set()
    core_src = root / "crates" / "core" / "src"
    for f in core_src.glob("*.rs"):
        if f.name != "lib.rs":
            internal_modules.add(f.stem)

    for module in contract.modules or []:
        files = find_rust_files_for_module(root, module)
        module_name = module.split("::")[-1]

        for file in files:
            content = file.read_text()
            imports = extract_imports(content)

            # Check if importing any other internal module
            for imp in imports:
                if imp in internal_modules and imp != module_name:
                    rel_path = file.relative_to(root)
                    errors.append(
                        f"[{contract.id}] {rel_path} imports internal module '{imp}' "
                        f"(violates independence)"
                    )

    return errors


def check_forbidden_pattern_contract(contract: Contract, root: Path) -> list[str]:
    """Check a 'forbidden_pattern' type contract."""
    errors = []

    for source in contract.source_modules or []:
        files = find_rust_files_for_module(root, source)
        for file in files:
            # Check exceptions
            if contract.exceptions:
                if any(exc in str(file) for exc in contract.exceptions):
                    continue

            content = file.read_text()

            for pattern in contract.forbidden_patterns or []:
                if pattern in content:
                    rel_path = file.relative_to(root)
                    errors.append(
                        f"[{contract.id}] {rel_path} contains forbidden pattern '{pattern}'"
                    )

    return errors


def validate_architecture(root: Path) -> list[str]:
    """Validate all architecture rules."""
    contracts = load_architecture_rules(root)
    if not contracts:
        return []

    print(f"Checking {len(contracts)} architecture contracts...")
    errors = []

    for contract in contracts:
        if contract.contract_type == "forbidden":
            errors.extend(check_forbidden_contract(contract, root))
        elif contract.contract_type == "independence":
            errors.extend(check_independence_contract(contract, root))
        elif contract.contract_type == "forbidden_pattern":
            errors.extend(check_forbidden_pattern_contract(contract, root))
        else:
            print(f"Warning: Unknown contract type '{contract.contract_type}'")

    return errors


def main():
    root = Path.cwd()
    errors = validate_architecture(root)

    if errors:
        print("Architecture validation errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)
    else:
        print("Architecture validation passed")
        sys.exit(0)


if __name__ == "__main__":
    main()
