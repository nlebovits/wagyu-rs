# Contributing to wagyu-rs

## Development Setup

```bash
git clone https://github.com/nlebovits/wagyu-rs.git
cd wagyu-rs
git config core.hooksPath .githooks
cargo build && cargo test
```

## Commit Convention

[Conventional Commits](https://www.conventionalcommits.org/):

| Type | Description |
|------|-------------|
| `feat` | New feature (bumps minor) |
| `fix` | Bug fix (bumps patch) |
| `port` | Porting code from C++ |
| `docs` | Documentation only |
| `perf` | Performance improvement |
| `refactor` | Code change (no feature/fix) |
| `test` | Tests only |
| `chore` | Maintenance |

## Pull Request Process

1. Branch from `main`
2. `cargo test && cargo fmt --all && cargo clippy`
3. Submit PR

## Porting from wagyu C++

When porting code:

1. Reference the original C++ file in comments
2. Write tests first (TDD)
3. Document any divergences from C++
4. Preserve original comments where helpful

## Releasing (Maintainers)

### Prerequisites

1. **Commitizen** installed: `uv tool install commitizen`
2. **GitHub secrets**: `CARGO_REGISTRY_TOKEN`

### Release Workflow

```bash
git checkout main && git pull
git checkout -b release/vX.Y.Z

cz bump --increment MINOR --changelog   # or PATCH/MAJOR
cargo check

git push -u origin release/vX.Y.Z
gh pr create --title "Release vX.Y.Z" --body "Automated release"
# Merge PR -> release.yml auto-publishes
```
