#!/usr/bin/env python3
"""Copy publishable working files to a new directory, preserving source Git repos.

Uses the workspace's .gitignore rules, including nested rules. Requires Git.
The destination contains source files, with no nested Git metadata.
This script does not initialize, commit, or push the exported repository.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = ("README.md", ".gitignore", "dependencies.repos", "UPSTREAM.md")
ROOT_DIRS = ("src", "scripts", "config", "tests", "windows_vlm_bridge", "robot_bridge", "docs")


def git_ignored(git_dir: Path, paths: list[str]) -> set[str]:
    if not paths:
        return set()
    result = subprocess.run(
        [
            "git", "-c", "core.excludesFile=/dev/null",
            f"--git-dir={git_dir}", f"--work-tree={ROOT}",
            "check-ignore", "--no-index", "-z", "--stdin",
        ],
        cwd=ROOT,
        input=b"\0".join(os.fsencode(path) for path in paths) + b"\0",
        capture_output=True,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.decode(errors="replace"))
    return {os.fsdecode(path) for path in result.stdout.split(b"\0") if path}


def publishable_files(git_dir: Path) -> list[str]:
    files = list(ROOT_FILES)
    for name in ROOT_DIRS:
        for directory, subdirs, filenames in os.walk(ROOT / name):
            base = Path(directory)
            subdirs[:] = [item for item in subdirs if item != ".git"]
            candidates = [
                str((base / item).relative_to(ROOT))
                for item in subdirs + filenames if item != ".git"
            ]
            ignored = git_ignored(git_dir, candidates)
            subdirs[:] = [
                item for item in subdirs
                if str((base / item).relative_to(ROOT)) not in ignored
            ]
            for item in subdirs:
                if (base / item).is_symlink():
                    raise RuntimeError(f"Review directory symlink before export: {base / item}")
            files.extend(
                str((base / item).relative_to(ROOT))
                for item in filenames
                if item != ".git"
                and str((base / item).relative_to(ROOT)) not in ignored
            )
    selected = {ROOT / relative for relative in files}
    for relative in files:
        source = ROOT / relative
        if not source.is_file():
            raise RuntimeError(f"Expected a source file: {source}")
        if source.is_symlink():
            if Path(os.readlink(source)).is_absolute() or source.resolve() not in selected:
                raise RuntimeError(f"Symlink target must also be exported: {source}")
    return sorted(files)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="New directory to create")
    args = parser.parse_args()
    destination = args.destination.expanduser().resolve()
    if destination.exists():
        parser.error("destination already exists; choose a new directory")
    if destination == ROOT or (
        ROOT in destination.parents and ROOT / "exports" not in destination.parents
    ):
        parser.error("a destination inside this workspace must be under exports/")
    with tempfile.TemporaryDirectory(prefix="fr3-export-git-") as temporary:
        git_dir = Path(temporary) / "repository.git"
        subprocess.run(["git", "init", "--bare", "--quiet", str(git_dir)], check=True)
        files = publishable_files(git_dir)
    destination.mkdir(parents=True)
    size = 0
    for relative in files:
        source = ROOT / relative
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target, follow_symlinks=False)
        size += source.stat().st_size
    print(f"Exported {len(files)} files ({size / 1024 / 1024:.2f} MiB) to {destination}")
    print("Includes current working files, including uncommitted source changes.")
    print("No source Git metadata changed; no repository initialized or pushed.")


if __name__ == "__main__":
    main()
